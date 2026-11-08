"""Phase B strategic owner is one shadow goal planner, not a motor driver."""
import json
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from threading import Barrier

from beep_agent.session import AgentSession
from beep_agent.strategy import GoalProposal, StrategicOwner, StrategyError, LIFETIME_CHARTER

FIXTURE = Path(__file__).parent / "fixtures" / "box_far_response.json"


def proposal(name="p1", **changes):
    data = {"schema_version": 1, "proposal_id": name, "session_id": "s", "boot_id": "b",
            "map_epoch": 1, "map_revision": 0, "base_goal_revision": 0,
            "observed_world_sequence": 1, "goal_name": "explore", "target_track_id": None,
            "issued_at_robot": 10., "expires_at_robot": 12.,
            "source_event_ids": [], "reason": "Investigate a previously unseen part of this room."}
    data.update(changes)
    return GoalProposal.from_dict(data)


class StrategyTests(unittest.TestCase):
    def setUp(self):
        self.session = AgentSession(robot_id="beep", mission_id=None, boot_id="b", session_id="s")
        self.owner = StrategicOwner(self.session)
        self.session.update(json.loads(FIXTURE.read_text()), observed_at=1., packet_id="f1")

    def submit(self, p, *, now=10.1, epoch=1, revision=0):
        return self.owner.submit(p, now_robot=now, map_epoch=epoch, map_revision=revision)

    def test_lifetime_charter_and_single_revisable_goal_survive_lease_handoff(self):
        self.assertIn("GLADIS", LIFETIME_CHARTER)
        target = next(iter(self.session.world.tracks))
        selection = self.submit(proposal(goal_name="investigate", target_track_id=target))
        self.assertEqual(selection.goal.name, "investigate")
        self.assertEqual(self.session.strategy["goal_revision"], 1)
        self.session.bind_motion(mission_id="m", lease_id="l", boot_id="b")
        self.session.release_motion(mission_id="m", lease_id="l")
        self.session.update(json.loads(FIXTURE.read_text()), observed_at=2., packet_id="f2")
        self.assertEqual(self.session.selection.goal.name, "investigate")
        self.assertEqual(self.session.strategy["goal_revision"], 1)
        restored = AgentSession.from_dict(self.session.to_dict(), expected_boot_id="b")
        self.assertEqual(restored.strategy["goal_revision"], 1)
        self.assertEqual(restored.selection.goal, self.session.selection.goal)
        self.assertTrue(restored.decide().requires_physical_authority)
        # Physical intent remains a shadow decision with no dispatcher.
        self.assertEqual(self.submit(proposal(goal_name="investigate", target_track_id=target)),
                         self.session.selection)  # Idempotent retry.

    def test_delayed_or_conflicting_proposals_cannot_retarget(self):
        for p, now, epoch, revision in (
            (proposal("old", observed_world_sequence=0), 10.1, 1, 0),
            (proposal("expired"), 12.1, 1, 0),
            (proposal("wrong-boot", boot_id="other"), 10.1, 1, 0),
            (proposal("wrong-map"), 10.1, 2, 0),
            (proposal("wrong-revision"), 10.1, 1, 1),
            (proposal("lost-target", goal_name="investigate", target_track_id="object-missing"), 10.1, 1, 0),
        ):
            with self.subTest(p=p.proposal_id), self.assertRaises(StrategyError):
                self.submit(p, now=now, epoch=epoch, revision=revision)
        self.assertEqual(self.session.strategy["goal_revision"], 0)
        self.submit(proposal())
        with self.assertRaises(StrategyError):
            self.submit(proposal("stale", base_goal_revision=0))
        with self.assertRaises(StrategyError):
            self.submit(proposal(goal_name="rest", base_goal_revision=0))  # Conflicting same ID.
        self.submit(proposal("next", base_goal_revision=1, goal_name="rest"))
        with self.assertRaises(StrategyError):
            self.submit(proposal())  # Old retry must not claim the newer goal.
        self.assertEqual(self.session.decide().skill_call.name, "observe")

    def test_marking_requires_current_geometry_and_tactical_evidence_is_verified(self):
        target = next(iter(self.session.world.tracks))
        with self.assertRaises(StrategyError):
            self.submit(proposal(goal_name="investigate_and_mark", target_track_id=target))
        with self.assertRaises(StrategyError):
            self.submit(proposal(source_event_ids=["unseen-tactical-id"]))

    def test_fixed_operator_goal_cannot_be_silently_replaced(self):
        from beep_agent.executive import EmbodiedGoal
        self.session.fixed_goal = EmbodiedGoal("investigate", "white box")
        with self.assertRaises(StrategyError):
            self.submit(proposal())

    def test_context_is_bounded_and_does_not_include_full_journal(self):
        context = self.owner.build_context(episodes=[{"id": i} for i in range(50)],
                                           places=[{"id": i} for i in range(50)])
        self.assertEqual(len(context["episodes"]), 8)
        self.assertEqual(len(context["places"]), 8)
        self.assertIn("charter", context)
        self.assertNotIn("journal", context)

    def test_parallel_same_revision_proposals_admit_exactly_one(self):
        gate = Barrier(2)
        def apply(name):
            gate.wait(timeout=2)
            try:
                self.submit(proposal(name))
                return True
            except StrategyError:
                return False
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(apply, ("alpha", "beta")))
        self.assertEqual(results.count(True), 1)
        self.assertEqual(self.session.strategy["goal_revision"], 1)


if __name__ == "__main__":
    unittest.main()
