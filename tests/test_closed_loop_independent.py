"""Independent, offline safety regressions for the mission-owned feedback loop.

No robot connection, SDK initialization, motor backend or worker thread is used.
The transport and motor boundaries are blocked even if a regression reaches them.
"""

import importlib.util
import json
from pathlib import Path
import tempfile
import time
import unittest
from typing import Any
from unittest.mock import MagicMock, patch

from beep_agent import BodyDispatchError, BridgeBodyAdapter, skill
from beep_agent.executor import ClosedLoopExecutor
from beep_agent.executive import EmbodiedGoal
from beep_agent.session import AgentSession, SessionIdentityError
from beep_bridge.semantic_intent import IntentMailbox, sensor_gate, validate_intent


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "beep_bridge_independent_closed_loop", ROOT / "beep_bridge" / "beep_bridge.py"
)
assert SPEC is not None and SPEC.loader is not None
bridge: Any = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


class OfflineCase(unittest.TestCase):
    motor_send: MagicMock
    stop_burst: MagicMock

    def setUp(self):
        bridge._reset_motion_state_for_tests()
        bridge._reset_mission_state_for_tests()
        for target in ("socket.create_connection", "urllib.request.urlopen"):
            blocker = patch(target, side_effect=AssertionError("offline test attempted network I/O"))
            blocker.start()
            self.addCleanup(blocker.stop)
        for name in ("motor_send", "stop_burst"):
            blocker = patch.object(bridge, name)
            setattr(self, name, blocker.start())
            self.addCleanup(blocker.stop)
        self.stop_burst.return_value = None
        self.addCleanup(bridge._reset_motion_state_for_tests)
        self.addCleanup(bridge._reset_mission_state_for_tests)

    def install_mission(self, mission_id="mission-independent", lease_id="lease-independent"):
        lease = bridge.MotionLease(lease_id, "independent-test", time.monotonic() + 60)
        mission = {
            "id": mission_id, "lease_id": lease_id, "lease": lease,
            "mode": "coverage", "state": "running", "duration_s": 60,
            "started_at": time.time(), "finished_at": None,
            "intent_mailbox": bridge.IntentMailbox(),
        }
        bridge._active_motion_lease = lease
        bridge._active_mission = mission
        bridge.motion_context.lease = lease
        return mission, lease


class IndependentOwnershipTests(OfflineCase):
    def test_stop_for_old_mission_cannot_cancel_replacement(self):
        mission, lease = self.install_mission(mission_id="mission-B")
        result = bridge.cancel_autonomous_mission("mission-A")
        self.assertFalse(result["ok"])
        self.assertFalse(result["cancelled"])
        self.assertFalse(lease.cancel_event.is_set())
        self.assertIs(bridge._active_mission, mission)
        self.stop_burst.assert_not_called()
        self.motor_send.assert_not_called()

    def test_missing_mission_identity_must_not_cancel_current_owner(self):
        _, lease = self.install_mission()
        for identity in (None, ""):
            with self.subTest(identity=identity):
                result = bridge.cancel_autonomous_mission(identity)
                self.assertFalse(result["ok"], "identity-free stop must fail closed")
                self.assertFalse(result["cancelled"])
                self.assertFalse(lease.cancel_event.is_set())
        self.stop_burst.assert_not_called()

    def test_matching_stop_cancels_exact_lease_and_reports_backend_failure(self):
        mission, lease = self.install_mission()
        self.stop_burst.return_value = "backend unavailable"
        result = bridge.cancel_autonomous_mission(mission["id"])
        self.assertTrue(lease.cancel_event.is_set())
        self.assertFalse(result["ok"], "failed physical stop must not be reported as success")
        self.assertEqual(result["error"], "backend unavailable")
        self.stop_burst.assert_called_once()

    def test_old_watchdog_and_release_do_not_touch_replacement_lease(self):
        _, current = self.install_mission()
        expired = bridge.MotionLease("old-lease", "old", time.monotonic() - 1)
        bridge.motion_lease_watchdog(expired)
        bridge.end_motion(expired)
        self.assertIs(bridge.active_motion_lease(), current)
        self.assertFalse(current.cancel_event.is_set())
        self.stop_burst.assert_not_called()

    def test_current_watchdog_still_has_veto_power(self):
        _, lease = self.install_mission()
        lease.deadline = time.monotonic() - 1
        bridge.motion_lease_watchdog(lease)
        self.assertTrue(lease.cancel_event.is_set())
        self.assertTrue(bridge.motion_is_cancelled())
        self.stop_burst.assert_called_once()
        self.motor_send.assert_not_called()


class IndependentAdapterTests(OfflineCase):
    def test_shadow_never_calls_transport_even_for_stop(self):
        adapter = BridgeBodyAdapter("http://robot.invalid")
        calls = [skill("explore", duration_s=30, reason="test"),
                 skill("stop", reason="test"), skill("observe", reason="test")]
        with patch.object(adapter, "_json", side_effect=AssertionError("shadow transport")):
            for call in calls:
                with self.subTest(skill=call.name):
                    result = adapter.dispatch(call)
                    self.assertFalse(result.dispatched)

    def test_unsupported_target_skills_never_fall_back_to_movement(self):
        adapter = BridgeBodyAdapter("http://robot.invalid")
        calls = [
            skill("approach_target", target_id="object-1", max_step_m=0.1,
                  stop_distance_m=0.4, reason="test"),
            skill("inspect_target", target_id="object-1", strategy="left_view",
                  max_turn_deg=10, reason="test"),
            skill("mark_target", target_id="object-1", feature="left_outward_corner",
                  dry_run=False, reason="test"),
        ]
        with patch.object(adapter, "_json") as transport:
            for call in calls:
                with self.subTest(skill=call.name), self.assertRaises(BodyDispatchError):
                    adapter.dispatch(call, rollout_mode="supervised")
        transport.assert_not_called()


def intent(**changes):
    value = {
        "mission_id": "mission-independent", "lease_id": "lease-independent",
        "session_id": "session-independent", "decision_id": "decision-1",
        "executor_instance_id": "executor-independent", "executor_generation": 1,
        "frame_id": "window-1", "observed_at": 100.0,
        "latest_frame_at": 101.0, "expires_at": 105.0, "kind": "course_left",
    }
    value.update(changes)
    return value


def safe_status():
    return {
        "moving": True, "motion_lease_id": "lease-independent",
        "scan_seen": True, "scan_age_s": 0.01,
        "slam": {"usable": True, "pose_valid": True,
                 "pose_age_s": 0.01, "map_age_s": 0.01},
        "sectors": dict.fromkeys(
            ("front", "front_left", "front_right", "left", "right", "rear"), 2.0),
    }


class IndependentMailboxTests(unittest.TestCase):
    def test_duplicate_retry_returns_outcome_without_requeue(self):
        mailbox = IntentMailbox()
        packet = intent()
        self.assertEqual(mailbox.submit(packet, 102.0)["state"], "accepted")
        self.assertEqual(mailbox.take(102.0), packet)
        mailbox.outcome(packet["decision_id"], "applied", "test_boundary")
        mailbox.outcome(packet["decision_id"], "completed", "test_completed")
        retry = mailbox.submit(packet, 110.0)
        self.assertEqual(retry["state"], "completed")
        self.assertIsNone(mailbox.take(110.0))
        self.assertEqual(len(mailbox.snapshot()), 1)

    def test_retry_with_mutated_intent_cannot_replace_original(self):
        mailbox = IntentMailbox()
        packet = intent()
        mailbox.submit(packet, 102.0)
        result = mailbox.submit(intent(kind="course_right"), 102.0)
        self.assertFalse(result["ok"])
        self.assertEqual(mailbox.take(102.0), packet)

    def test_newer_window_supersedes_pending_intent(self):
        mailbox = IntentMailbox()
        mailbox.submit(intent(), 102.0)
        newer = intent(decision_id="decision-2", frame_id="window-2",
                       observed_at=101.0, latest_frame_at=102.0, expires_at=106.0)
        mailbox.submit(newer, 103.0)
        self.assertEqual(mailbox.take(103.0), newer)
        self.assertIsNone(mailbox.take(103.0))
        old = mailbox.submit(intent(), 103.0)
        self.assertEqual(old["state"], "rejected")
        self.assertEqual(old["reason"], "superseded_intent")

    def test_equal_or_older_frame_cannot_masquerade_as_new_decision(self):
        for timestamp in (100.5, 101.0):
            with self.subTest(timestamp=timestamp):
                mailbox = IntentMailbox()
                mailbox.submit(intent(), 102.0)
                result = mailbox.submit(intent(decision_id="forged-new-id",
                                                latest_frame_at=timestamp), 102.0)
                self.assertFalse(result["ok"])
                selected = mailbox.take(102.0)
                assert selected is not None
                self.assertEqual(selected["decision_id"], "decision-1")

    def test_expired_at_boundary_cannot_execute_even_if_accepted_earlier(self):
        mailbox = IntentMailbox()
        self.assertTrue(mailbox.submit(intent(), 102.0)["ok"])
        self.assertIsNone(mailbox.take(105.0))
        self.assertEqual(mailbox.snapshot()[0]["state"], "rejected")

    def test_oldest_frame_not_inference_completion_limits_evidence_lifetime(self):
        for payload in (
            intent(observed_at=90.0, latest_frame_at=101.0),
            intent(expires_at=106.0),
            intent(latest_frame_at=103.0),
            intent(observed_at=102.0, latest_frame_at=101.0),
        ):
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                validate_intent(payload, 102.0)

    def test_wrong_executor_session_cannot_supersede_owner(self):
        mailbox = IntentMailbox()
        mailbox.submit(intent(), 102.0)
        response = mailbox.submit(intent(session_id="intruder", decision_id="decision-2",
                                         latest_frame_at=102.0), 102.0)
        self.assertFalse(response["ok"])
        selected = mailbox.take(102.0)
        assert selected is not None
        self.assertEqual(selected["session_id"], "session-independent")

    def test_envelope_rejects_raw_motor_fields_and_unsupported_skills(self):
        for changes in ({"velocity": 1}, {"kind": "approach_target"},
                        {"kind": "turnleft"}, {"kind": "eval"},
                        {"observed_at": True}, {"expires_at": float("nan")},
                        {"latest_frame_at": float("inf")}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_intent(intent(**changes), 102.0)

    def test_snapshots_cannot_mutate_pending_authority(self):
        mailbox = IntentMailbox()
        response = mailbox.submit(intent(), 102.0)
        response["intent"]["kind"] = "stop"
        mailbox.snapshot()[0]["intent"]["kind"] = "course_right"
        selected = mailbox.take(102.0)
        assert selected is not None
        self.assertEqual(selected["kind"], "course_left")

    def test_close_revokes_pending_intent_and_retry_does_not_revive_it(self):
        mailbox = IntentMailbox()
        mailbox.submit(intent(), 102.0)
        mailbox.close("mission_replaced")
        self.assertIsNone(mailbox.take(102.0))
        self.assertEqual(mailbox.submit(intent(), 102.0)["state"], "rejected")
        self.assertIsNone(mailbox.take(102.0))

    def test_sensor_gate_requires_all_finite_positive_ranges(self):
        self.assertTrue(sensor_gate(safe_status()))
        for sector in safe_status()["sectors"]:
            for bad in (None, True, 0, -1, float("nan"), float("inf"), "2.0"):
                with self.subTest(sector=sector, bad=bad):
                    state = safe_status()
                    state["sectors"][sector] = bad
                    self.assertFalse(sensor_gate(state))

    def test_sensor_gate_rejects_stale_or_invalid_localization(self):
        for field, bad in (("usable", False), ("pose_valid", False),
                           ("pose_age_s", 7.0), ("map_age_s", 3.0),
                           ("pose_age_s", True), ("map_age_s", float("nan"))):
            with self.subTest(field=field, bad=bad):
                state = safe_status()
                state["slam"][field] = bad
                self.assertFalse(sensor_gate(state))
        for bad in (None, True, -0.1, 0.61, float("nan"), float("inf")):
            with self.subTest(scan_age_s=bad):
                state = safe_status()
                state["scan_age_s"] = bad
                self.assertFalse(sensor_gate(state))


class BoundaryCase(OfflineCase):
    def setUp(self):
        super().setUp()
        self.mission, self.lease = self.install_mission()
        self.mission["intent_mailbox"] = bridge.IntentMailbox(require_fencing=True)
        clock = patch.object(bridge.time, "time", return_value=102.0)
        self.clock = clock.start()
        self.addCleanup(clock.stop)
        armed = patch.object(bridge, "SEMANTIC_INTENTS_ENABLED", True)
        armed.start()
        self.addCleanup(armed.stop)
        if not isinstance(self, IndependentExecutorIntegrationTests):
            self.assertTrue(bridge.claim_navigation_executor({
                "mission_id": self.mission["id"], "lease_id": self.lease.lease_id,
                "session_id": "session-independent", "executor_instance_id": "executor-independent",
                "expected_generation": 0, "takeover": False})["ok"])


class IndependentNavigationBoundaryTests(BoundaryCase):
    def test_submission_only_queues_and_compatible_boundary_never_stops(self):
        response = bridge.submit_navigation_intent(intent(kind="continue"))
        self.assertEqual(response["state"], "accepted")
        self.motor_send.assert_not_called()
        self.stop_burst.assert_not_called()
        action, selected, reason = bridge.navigation_intent_boundary(
            safe_status(), "forward", "forward")
        self.assertEqual(action, "forward")
        self.assertEqual(selected["decision_id"], "decision-1")
        self.assertIsNone(reason)
        bridge.finish_navigation_intent(selected, "applied", "register_update")
        bridge.finish_navigation_intent(selected, "completed", "segment_completed")
        self.assertEqual(bridge.submit_navigation_intent(intent(kind="continue"))["state"], "completed")
        self.assertEqual(bridge.navigation_intent_boundary(safe_status(), "forward", "forward"),
                         (None, None, None))
        self.assertIs(bridge._active_mission, self.mission)
        self.assertIs(bridge.active_motion_lease(), self.lease)
        self.stop_burst.assert_not_called()

    def test_onboard_arm_and_exact_mission_lease_both_required(self):
        with patch.object(bridge, "SEMANTIC_INTENTS_ENABLED", False):
            self.assertFalse(bridge.submit_navigation_intent(intent())["ok"])
        for changes in ({"mission_id": "replacement"}, {"lease_id": "replacement"}):
            with self.subTest(changes=changes):
                self.assertFalse(bridge.submit_navigation_intent(intent(**changes))["ok"])
        self.assertEqual(self.mission["intent_mailbox"].snapshot(), [])
        self.motor_send.assert_not_called()
        self.stop_burst.assert_not_called()

    def test_local_sensors_are_rechecked_after_acceptance(self):
        self.assertTrue(bridge.submit_navigation_intent(intent())["ok"])
        state = safe_status()
        state["sectors"]["rear"] = float("nan")
        action, selected, reason = bridge.navigation_intent_boundary(state, "forward", "forward")
        self.assertIsNone(action)
        self.assertIsNone(selected)
        self.assertIsNotNone(reason)
        self.assertEqual(self.mission["intent_mailbox"].snapshot()[0]["state"], "rejected")
        self.motor_send.assert_not_called()

    def test_local_navigation_veto_wins_over_visual_course(self):
        bridge.submit_navigation_intent(intent())
        action, selected, reason = bridge.navigation_intent_boundary(safe_status(), "back", "forward")
        self.assertIsNone(action)
        self.assertIsNone(selected)
        self.assertIsNotNone(reason)
        self.motor_send.assert_not_called()

    def test_expiry_between_acceptance_and_boundary_revokes_motion(self):
        bridge.submit_navigation_intent(intent())
        self.clock.return_value = 105.0
        action, selected, reason = bridge.navigation_intent_boundary(safe_status(), "forward", "forward")
        self.assertIsNone(action)
        self.assertIsNone(selected)
        self.assertIsNotNone(reason)
        self.motor_send.assert_not_called()

    def test_lease_replaced_after_acceptance_cannot_dispatch_course(self):
        bridge.submit_navigation_intent(intent())
        replacement = bridge.MotionLease("replacement", "other", time.monotonic() + 60)
        bridge._active_motion_lease = replacement
        action, selected, reason = bridge.navigation_intent_boundary(safe_status(), "forward", "forward")
        self.assertIsNone(action)
        self.assertIsNone(selected)
        self.assertIsNotNone(reason)
        self.motor_send.assert_not_called()
        self.stop_burst.assert_not_called()

    def test_lease_replaced_after_acceptance_cannot_receive_stale_stop(self):
        bridge.submit_navigation_intent(intent(kind="stop"))
        replacement = bridge.MotionLease("replacement", "other", time.monotonic() + 60)
        bridge._active_motion_lease = replacement
        action, selected, reason = bridge.navigation_intent_boundary(safe_status(), "forward", "forward")
        self.assertIsNone(action)
        self.assertIsNone(selected)
        self.assertIsNotNone(reason)
        self.stop_burst.assert_not_called()
        self.assertFalse(replacement.cancel_event.is_set())
        self.assertEqual(self.mission["intent_mailbox"].snapshot()[0]["state"], "rejected")

    def test_stop_backend_failure_never_reports_completed(self):
        bridge.submit_navigation_intent(intent(kind="stop"))
        self.stop_burst.return_value = "backend failure"
        bridge.navigation_intent_boundary(safe_status(), "forward", "forward")
        record = self.mission["intent_mailbox"].snapshot()[0]
        self.assertEqual(record["state"], "rejected")
        self.assertNotIn("completed", [item["state"] for item in record["history"]])
        self.motor_send.assert_not_called()


class IndependentExecutorIntegrationTests(BoundaryCase):
    def setUp(self):
        super().setUp()
        directory = tempfile.TemporaryDirectory(prefix="beep-independent-")
        self.addCleanup(directory.cleanup)
        self.state_path = Path(directory.name) / "session.json"
        self.session = AgentSession(
            robot_id="offline-beep", mission_id=self.mission["id"],
            session_id="session-independent", created_at=100.0,
            fixed_goal=EmbodiedGoal("explore", "safe space"))
        self.body = BridgeBodyAdapter("http://robot.invalid")
        claim = patch.object(self.body, "claim_executor", side_effect=bridge.claim_navigation_executor)
        claim.start()
        self.addCleanup(claim.stop)
        transport = patch.object(self.body, "_json", side_effect=self.transport)
        self.requests = transport.start()
        self.addCleanup(transport.stop)
        self.executor = ClosedLoopExecutor(self.session, self.body, mode="supervised",
                                           armed=True, state_path=self.state_path)

    def transport(self, path, body=None):
        self.assertEqual(path, "/mission/intent", "executor must never start/restart/cancel a mission")
        assert body is not None
        saved = json.loads(self.state_path.read_text())
        self.assertEqual(saved["execution"]["issued"][body["decision_id"]]["state"], "sending")
        return bridge.submit_navigation_intent(body)

    def perception(self, direction="left"):
        response = json.loads((ROOT / "tests" / "fixtures" / "eyes_response.json").read_text())
        response["scene"].update(people=[], objects=[], hazards=[])
        response["attention"] = {"target_type": "none", "target_id": None, "reason": "exploration"}
        response["plan"]["steps"] = [
            {"skill": "advance", "distance_m": 0.2, "reason": "clear route"}]
        return response

    def frames(self, latest=101.0):
        return [{"captured_at": latest - 0.5, "frame_id": "first-" + str(latest),
                 "motion": {"motion_lease_id": self.lease.lease_id}},
                {"captured_at": latest, "frame_id": "last-" + str(latest),
                 "motion": {"motion_lease_id": self.lease.lease_id}}]

    def step(self, latest=101.0, direction="left", **changes):
        args = {"packet_id": "packet-" + str(latest), "frames": self.frames(latest),
                "mission": bridge.mission_snapshot(), "status": safe_status(),
                "now": self.clock.return_value}
        args.update(changes)
        return self.executor.step(self.perception(direction), **args)

    def complete_boundary(self, expected_action):
        action, selected, reason = bridge.navigation_intent_boundary(safe_status(), "forward", "forward")
        self.assertEqual(action, expected_action)
        self.assertIsNone(reason)
        bridge.finish_navigation_intent(selected, "applied", "fake_register_update")
        bridge.finish_navigation_intent(selected, "completed", "fake_segment_completed")

    def test_successive_windows_feed_outcomes_into_same_session_and_mission(self):
        first = self.step()
        self.assertEqual(first["state"], "accepted")
        self.complete_boundary("forward")
        self.clock.return_value = 103.0
        second = self.step(latest=102.0, direction="right")
        self.assertEqual(second["state"], "accepted")
        self.assertEqual(self.session.execution["issued"][first["intent"]["decision_id"]]["state"], "completed")
        self.complete_boundary("forward")
        self.executor.reconcile(bridge.mission_snapshot())
        self.assertEqual(self.session.world.sequence, 2)
        self.assertEqual(self.requests.call_count, 2)
        self.assertIs(bridge._active_mission, self.mission)
        self.assertEqual(self.session.mission_id, self.mission["id"])
        self.assertEqual(self.session.execution["last_outcome"]["state"], "completed")
        self.stop_burst.assert_not_called()

    def test_look_is_not_translational_authority_even_with_central_glass(self):
        response = self.perception()
        response["plan"]["steps"] = [{"skill": "orient", "direction": "left", "degrees": 10, "reason": "look"}]
        response["scene"]["hazards"] = [{"type": "glass", "position": "center", "confidence": 0.95, "description": "glass"}]
        result = self.executor.step(response, packet_id="glass", frames=self.frames(),
                                    mission=bridge.mission_snapshot(), status=safe_status(), now=102.0)
        self.assertNotIn((result.get("intent") or {}).get("kind"), {"continue", "course_left", "course_right"})
        self.motor_send.assert_not_called()

    def test_unknown_undelivered_expires_and_clears_pending_without_false_success(self):
        self.requests.side_effect = BodyDispatchError("reply lost")
        first = self.step()
        self.assertEqual(first["state"], "unknown")
        self.clock.return_value = 107.0
        self.executor.reconcile(bridge.mission_snapshot())
        record = self.session.execution["issued"][first["intent"]["decision_id"]]
        self.assertEqual(record["state"], "expired_unconfirmed")
        self.assertFalse(record["ok"])

    def test_shadow_executor_runs_perception_without_any_action_request(self):
        self.executor = ClosedLoopExecutor(self.session, self.body)
        result = self.step()
        self.assertEqual(result["state"], "shadow")
        self.assertFalse(result["dispatch_performed"])
        self.requests.assert_not_called()
        self.assertEqual(self.mission["intent_mailbox"].snapshot(), [])

    def test_delayed_inference_cannot_refresh_old_frames(self):
        self.clock.return_value = 110.0
        result = self.step()
        self.assertEqual(result["state"], "rejected")
        self.assertFalse(result["dispatch_performed"])
        self.assertEqual(self.session.updated_at, 101.0)
        self.requests.assert_not_called()

    def test_one_frame_from_another_lease_rejects_whole_window(self):
        frames = self.frames()
        frames[0]["motion"]["motion_lease_id"] = "previous-lease"
        result = self.step(frames=frames)
        self.assertEqual(result["state"], "rejected")
        self.assertFalse(result["dispatch_performed"])
        self.requests.assert_not_called()

    def test_resume_and_duplicate_packet_do_not_reissue_completed_intent(self):
        first = self.step()
        self.complete_boundary("forward")
        restored = AgentSession.load(self.state_path, expected_robot_id="offline-beep",
                                     expected_mission_id=self.mission["id"])
        self.executor = ClosedLoopExecutor(restored, self.body, mode="supervised",
                                           armed=True, state_path=self.state_path)
        self.executor.acquire(bridge.mission_snapshot(), takeover=True)
        with self.assertRaisesRegex(ValueError, "duplicate perception packet"):
            self.step()
        repeated = self.step(packet_id="same-frames-new-packet-id")
        self.assertFalse(repeated["dispatch_performed"])
        self.assertEqual(self.requests.call_count, 1)
        self.assertEqual(self.executor.session.execution["issued"][first["intent"]["decision_id"]]["state"], "completed")
        self.assertIsNone(self.mission["intent_mailbox"].take(102.0))

    def test_rejected_execution_forces_reobservation_before_further_dispatch(self):
        first = self.step()
        invalid = safe_status()
        invalid["scan_age_s"] = 2.0
        bridge.navigation_intent_boundary(invalid, "forward", "forward")
        self.clock.return_value = 103.0
        second = self.step(latest=102.0)
        self.assertFalse(second["dispatch_performed"])
        self.assertEqual(second["decision"]["skill_call"]["name"], "observe")
        self.assertEqual(self.session.execution["issued"][first["intent"]["decision_id"]]["state"], "rejected")
        self.assertEqual(self.requests.call_count, 1)
        self.clock.return_value = 104.0
        third = self.step(latest=103.0, direction="right")
        self.assertTrue(third["dispatch_performed"])
        self.assertEqual(self.requests.call_count, 2)

    def test_resume_with_changed_ownership_raises_without_action(self):
        self.step()
        changed = bridge.mission_snapshot()
        changed["lease_id"] = "new-lease"
        with self.assertRaises(SessionIdentityError):
            self.step(latest=102.0, mission=changed)
        self.assertEqual(self.requests.call_count, 1)

    def test_uncertain_transport_resume_does_not_create_second_intent(self):
        self.requests.side_effect = BodyDispatchError("reply lost")
        first = self.step()
        self.assertEqual(first["state"], "unknown")
        restored = AgentSession.load(self.state_path)
        self.executor = ClosedLoopExecutor(restored, self.body, mode="supervised",
                                           armed=True, state_path=self.state_path)
        self.executor.acquire(bridge.mission_snapshot(), takeover=True)
        for latest in (102.0, 103.0, 104.0):
            self.clock.return_value = latest + 0.5
            self.assertFalse(self.step(latest=latest)["dispatch_performed"])
        self.assertEqual(self.requests.call_count, 1)


if __name__ == "__main__":
    unittest.main()
