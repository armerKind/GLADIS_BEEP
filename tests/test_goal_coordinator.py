"""Offline goal/lease handoff contracts; no bridge imports or motor dispatch."""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from beep_agent.coordinator import CoordinatorContext, GoalCoordinator, CoordinatorError
from beep_agent.session import AgentSession
from beep_agent.strategy import GoalProposal, StrategicOwner

FIXTURES = Path(__file__).parent / "fixtures"


def context(**changes):
    data = dict(boot_id="b", map_epoch=1, map_revision=2, map_snapshot_id="grid",
                map_verified=True, sensors_fresh=True, now_robot=2.0,
                mission_id=None, lease_id=None, local_motion_active=False)
    data.update(changes)
    return CoordinatorContext(**data)


def ready():
    session = AgentSession(robot_id="beep", mission_id=None, boot_id="b", session_id="s")
    session.update(json.loads((FIXTURES / "box_far_response.json").read_text()),
                   observed_at=1.0, packet_id="first")
    proposal = GoalProposal.from_dict(dict(schema_version=1, proposal_id="self-goal",
        session_id="s", boot_id="b", map_epoch=1, map_revision=2,
        base_goal_revision=0, observed_world_sequence=session.world.sequence,
        goal_name="explore", target_track_id=None, issued_at_robot=1.0,
        expires_at_robot=3.0, source_event_ids=[],
        reason="Inspect the known-free parts of this place."))
    StrategicOwner(session).submit(proposal, now_robot=2.0, map_epoch=1,
                                   map_revision=2)
    return session, GoalCoordinator(session)


class CoordinatorTests(unittest.TestCase):
    def test_goal_survives_two_exact_lease_handoffs_without_motor_authority(self):
        session, owner = ready()
        suggestion = owner.tick(context())
        self.assertEqual(suggestion.action, "plan_next_segment")
        self.assertFalse(suggestion.motor_authority)
        self.assertEqual(suggestion.goal_name, "explore")
        self.assertFalse(hasattr(suggestion, "route"))
        owner.bind_confirmed(context(mission_id="m1", lease_id="l1", local_motion_active=True),
                             goal_revision=suggestion.goal_revision)
        self.assertEqual(owner.tick(context(mission_id="m1", lease_id="l1",
                                            local_motion_active=True)).action, "continue_local")
        # Inference and perception continue while local locomotion owns the lease.
        session.update(json.loads((FIXTURES / "box_far_response.json").read_text()),
                       observed_at=2.1, packet_id="second", mission_id="m1", lease_id="l1")
        owner.complete_segment(context(mission_id="m1", lease_id="l1"),
                               result="completed", release_id="release-1")
        self.assertEqual(session.strategy["goal_revision"], 1)
        self.assertEqual(owner.tick(context()).action, "plan_next_segment")
        owner.bind_confirmed(context(mission_id="m2", lease_id="l2", local_motion_active=True),
                             goal_revision=1)
        self.assertEqual(owner.tick(context(mission_id="m2", lease_id="l2",
                                            local_motion_active=True)).action, "continue_local")
        assert session.selection is not None
        self.assertEqual(session.selection.goal.name, "explore")
        self.assertEqual(len(session.execution["lease_history"]), 3)

    def test_no_handoff_without_exact_release_or_when_map_and_sensors_unverified(self):
        session, owner = ready()
        owner.bind_confirmed(context(mission_id="m1", lease_id="l1", local_motion_active=True),
                             goal_revision=1)
        self.assertEqual(owner.tick(context(mission_id="m1", lease_id="l1")).action,
                         "await_owner_release")
        with self.assertRaises(CoordinatorError):
            owner.complete_segment(context(mission_id="m1", lease_id="wrong"),
                                   result="completed", release_id="release-1")
        with self.assertRaises(CoordinatorError):
            owner.complete_segment(context(mission_id="m1", lease_id="l1",
                                           local_motion_active=True),
                                   result="completed", release_id="release-1")
        self.assertEqual(session.execution["lease_id"], "l1")
        owner.complete_segment(context(mission_id="m1", lease_id="l1"),
                               result="completed", release_id="release-1")
        self.assertEqual(owner.tick(context(map_verified=False)).action, "revalidate_map")
        self.assertEqual(owner.tick(context(sensors_fresh=False)).action, "revalidate_sensors")
        self.assertEqual(owner.tick(context(map_revision=3)).action, "plan_next_segment")
        self.assertEqual(owner.tick(context(mission_id="foreign", lease_id="other",
                                            local_motion_active=True)).action, "owner_mismatch")

    def test_stale_revisions_and_blocked_recovery_do_not_auto_retry(self):
        session, owner = ready()
        with self.assertRaises(CoordinatorError):
            owner.bind_confirmed(context(mission_id="m1", lease_id="l1", local_motion_active=True),
                                 goal_revision=0)
        owner.bind_confirmed(context(mission_id="m1", lease_id="l1", local_motion_active=True),
                             goal_revision=1)
        self.assertEqual(owner.tick(context(mission_id="m1", lease_id="l1", map_revision=3,
                                            local_motion_active=True)).action, "local_revalidate")
        self.assertEqual(owner.tick(context(mission_id="m1", lease_id="l1",
                                            sensors_fresh=False,
                                            local_motion_active=True)).action, "local_revalidate")
        owner.complete_segment(context(mission_id="m1", lease_id="l1"),
                               result="blocked", release_id="release-1")
        self.assertEqual(owner.tick(context()).action, "blocked_await_new_evidence")
        self.assertEqual(owner.tick(context(map_revision=3)).action, "plan_next_segment")
        with self.assertRaises(CoordinatorError):
            owner.complete_segment(context(mission_id="m1", lease_id="l1"),
                                   result="completed", release_id="release-1")
        self.assertEqual(session.execution["lease_id"], None)

    def test_strategic_revision_is_advisory_during_active_lease_and_rest_is_stationary(self):
        session, owner = ready()
        owner.bind_confirmed(context(mission_id="m1", lease_id="l1", local_motion_active=True),
                             goal_revision=1)
        revision = GoalProposal.from_dict(dict(schema_version=1, proposal_id="rest-now",
            session_id="s", boot_id="b", map_epoch=1, map_revision=2,
            base_goal_revision=1, observed_world_sequence=session.world.sequence,
            goal_name="rest", target_track_id=None, issued_at_robot=2.0,
            expires_at_robot=3.0, source_event_ids=[],
            reason="Rest intentionally after completing the current safe transition."))
        StrategicOwner(session).submit(revision, now_robot=2.1,
                                       map_epoch=1, map_revision=2)
        changed = owner.tick(context(mission_id="m1", lease_id="l1",
                                     local_motion_active=True))
        self.assertEqual(changed.action, "goal_revision_changed")
        self.assertFalse(changed.motor_authority)
        owner.complete_segment(context(mission_id="m1", lease_id="l1"),
                               result="cancelled", release_id="release-1")
        self.assertEqual(owner.tick(context()).action, "intentional_stationary")
        self.assertEqual(owner.tick(context(boot_id="foreign")).action, "identity_mismatch")

    def test_coordinator_checkpoint_survives_session_restart_and_rejects_other_identity(self):
        session, owner = ready()
        owner.bind_confirmed(context(mission_id="m1", lease_id="l1", local_motion_active=True),
                             goal_revision=1)
        checkpoint = owner.to_dict()
        restored_session = AgentSession.from_dict(session.to_dict(), expected_boot_id="b")
        restored = GoalCoordinator.from_dict(restored_session, checkpoint)
        self.assertEqual(restored.tick(context(mission_id="m1", lease_id="l1",
                                               local_motion_active=True)).action, "continue_local")
        restored.complete_segment(context(mission_id="m1", lease_id="l1"),
                                  result="blocked", release_id="release-1")
        reloaded = GoalCoordinator.from_dict(
            AgentSession.from_dict(restored_session.to_dict(), expected_boot_id="b"),
            restored.to_dict())
        self.assertEqual(reloaded.tick(context()).action, "blocked_await_new_evidence")
        with self.assertRaises(CoordinatorError):
            GoalCoordinator.from_dict(session, {**checkpoint, "session_id": "other"})

    def test_recorded_trace_runner_writes_no_dispatch_report(self):
        root = FIXTURES.parents[1]
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / "replay.json"
            completed = subprocess.run([sys.executable,
                str(root / "scripts/replay_goal_coordinator.py"),
                "--output", str(report)], cwd=root, capture_output=True,
                text=True, timeout=10, check=True)
            written = json.loads(report.read_text())
            self.assertEqual(json.loads(completed.stdout), written)
        self.assertEqual(written["decisions"], ["plan_next_segment", "continue_local",
            "continue_local", "blocked_await_new_evidence", "plan_next_segment"])
        self.assertEqual([entry["frame_id"] for entry in written["recorded_observations"]],
                         ["m000000", "m000012"])
        self.assertFalse(written["dispatch_performed"])
        self.assertFalse(written["route_validated"])
        self.assertEqual(written["map_and_lease_context"], "synthetic_not_recorded_provenance")

    def test_recorded_cap_observations_are_qualitative_not_routes(self):
        session, owner = ready()
        suggestion = owner.tick(context())
        owner.bind_confirmed(context(mission_id="m", lease_id="l", local_motion_active=True),
                             goal_revision=suggestion.goal_revision)
        for index in (0, 1):
            saved = json.loads((FIXTURES / f"cap_confirmation_window{index}.json").read_text())
            self.assertNotIn("bbox", saved["response"]["scene"]["objects"][0])
            session.update(saved["response"], observed_at=2.1 + index,
                           packet_id=f"recorded-{index}", mission_id="m", lease_id="l")
            decision = owner.tick(context(mission_id="m", lease_id="l",
                                          now_robot=2.1 + index, local_motion_active=True))
            self.assertEqual(decision.action, "continue_local")
            self.assertFalse(decision.motor_authority)
            self.assertFalse(hasattr(decision, "target_x_m"))
        self.assertEqual(session.strategy["goal_revision"], 1)
        assert session.selection is not None
        self.assertEqual(session.selection.goal.name, "explore")


if __name__ == "__main__":
    unittest.main()
