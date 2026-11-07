"""Offline regressions for the semantic transport fail-closed interlock.

Semantic execution deliberately remains disabled, even when admission is armed.
No test connects to a robot, initializes an SDK, or runs an autonomous worker.
"""
import importlib.util
from pathlib import Path
import threading
import time
import unittest
from typing import Any
from unittest.mock import MagicMock, patch


SPEC = importlib.util.spec_from_file_location(
    "beep_semantic_transport_safety",
    Path(__file__).resolve().parents[1] / "beep_bridge" / "beep_bridge.py",
)
assert SPEC is not None and SPEC.loader is not None
bridge: Any = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


class SemanticTransportSafetyTests(unittest.TestCase):
    motor_send: MagicMock
    start_or_update_fluent_motion: MagicMock
    stop_burst: MagicMock

    def setUp(self):
        bridge._reset_motion_state_for_tests()
        bridge._reset_mission_state_for_tests()
        self.addCleanup(bridge._reset_motion_state_for_tests)
        self.addCleanup(bridge._reset_mission_state_for_tests)
        for name in ("socket.create_connection", "urllib.request.urlopen", "sdk_init"):
            blocker = patch.object(bridge, name, create=False) if "." not in name else patch(
                name, side_effect=AssertionError("hardware/network forbidden"))
            mock = blocker.start()
            mock.side_effect = AssertionError("hardware/network forbidden")
            self.addCleanup(blocker.stop)
        for name in ("motor_send", "start_or_update_fluent_motion", "stop_burst"):
            blocker = patch.object(bridge, name, return_value=None)
            setattr(self, name, blocker.start())
            self.addCleanup(blocker.stop)
        armed = patch.object(bridge, "SEMANTIC_INTENTS_ENABLED", True)
        armed.start()
        self.addCleanup(armed.stop)
        self.lease = bridge.MotionLease("lease-test", "offline", time.monotonic() + 60)
        self.mission = {
            "id": "mission-test", "lease_id": self.lease.lease_id, "lease": self.lease,
            "state": "running", "duration_s": 60, "started_at": time.time(),
            "finished_at": None, "intent_mailbox": bridge.IntentMailbox(require_fencing=True),
        }
        bridge._active_motion_lease = self.lease
        bridge._active_mission = self.mission
        bridge.motion_context.lease = self.lease
        now = time.time()
        self.intent = {
            "mission_id": self.mission["id"], "lease_id": self.lease.lease_id,
            "session_id": "session-test", "decision_id": "decision-test",
            "executor_instance_id": "executor-test", "executor_generation": 1,
            "frame_id": "frame-test", "observed_at": now - 0.2,
            "latest_frame_at": now - 0.1, "expires_at": now + 2,
            "kind": "course_left",
        }
        self.assertTrue(bridge.claim_navigation_executor({
            "mission_id": self.mission["id"], "lease_id": self.lease.lease_id,
            "session_id": "session-test", "executor_instance_id": "executor-test",
            "expected_generation": 0, "takeover": False})["ok"])
        self.status = {
            "scan_seen": True, "scan_age_s": 0.01,
            "slam": {"usable": True, "pose_valid": True,
                     "pose_age_s": 0.01, "map_age_s": 0.01},
            "sectors": dict.fromkeys(
                ("front", "front_left", "front_right", "left", "right", "rear"), 2.0),
        }

    def enroll(self):
        self.assertEqual(bridge.submit_navigation_intent(self.intent)["state"], "accepted")
        self.start_or_update_fluent_motion.assert_not_called()

    def assert_no_dispatch(self, current, desired, intent):
        result = bridge.apply_navigation_course(current, desired, intent)
        self.assertEqual(result, (current, False, bridge.SEMANTIC_TRANSPORT_DISABLED_REASON))
        self.start_or_update_fluent_motion.assert_not_called()
        self.motor_send.assert_not_called()
        self.assertTrue(self.lease.cancel_event.is_set())

    def test_armed_valid_intent_is_rejected_before_either_backend(self):
        self.enroll()
        for backend in ("app", "sdk"):
            for desired in ("forward", "curveleft", "curveright"):
                with self.subTest(backend=backend, desired=desired), patch.object(
                        bridge, "MOTOR_BACKEND", backend):
                    self.assert_no_dispatch(None, desired, self.intent)
        record = self.mission["intent_mailbox"].snapshot()[0]
        self.assertEqual(record["state"], "rejected")
        self.assertEqual(record["reason"], bridge.SEMANTIC_TRANSPORT_DISABLED_REASON)

    def test_enrolled_continuation_and_unchanged_gait_are_disabled(self):
        self.enroll()
        for current, desired in (("curveleft", "forward"), ("forward", "forward"),
                                 ("forward", "curveright")):
            with self.subTest(current=current, desired=desired):
                self.assert_no_dispatch(current, desired, None)

    def test_intent_without_mission_cannot_fall_through_to_legacy_motion(self):
        bridge._active_mission = None
        self.assert_no_dispatch(None, "forward", self.intent)

    def test_guard_prevents_enrolled_recovery_from_bypassing_interlock(self):
        self.enroll()
        self.assertEqual(bridge.navigation_intent_guard(self.status),
                         bridge.SEMANTIC_TRANSPORT_DISABLED_REASON)

    def test_snapshot_distinguishes_admission_from_disabled_execution(self):
        result = bridge.mission_snapshot()
        self.assertTrue(result["semantic_intents_enabled"])
        self.assertIs(result["semantic_execution_enabled"], False)
        self.assertEqual(result["semantic_execution_disabled_reason"],
                         bridge.SEMANTIC_TRANSPORT_DISABLED_REASON)

    def test_expired_after_wait_for_ownership_cannot_send_any_packet(self):
        self.enroll()
        entered = threading.Event()
        results = []

        def dispatch():
            bridge.motion_context.lease = self.lease
            entered.set()
            results.append(bridge.apply_navigation_course(None, "curveleft", self.intent))

        with bridge.motion_owner_lock:
            worker = threading.Thread(target=dispatch, daemon=True)
            worker.start()
            self.assertTrue(entered.wait(1))
            self.lease.deadline = time.monotonic() - 1
            self.mission["semantic_deadline"] = time.monotonic() - 1
            self.intent["expires_at"] = time.time() - 1
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(results, [(None, False, bridge.SEMANTIC_TRANSPORT_DISABLED_REASON)])
        self.start_or_update_fluent_motion.assert_not_called()
        self.motor_send.assert_not_called()

    def test_watchdog_latches_while_ownership_and_state_locks_are_held(self):
        self.lease.deadline = time.monotonic() - 1
        with bridge.mission_lock, bridge.motion_owner_lock, bridge.state_lock:
            watchdog = threading.Thread(target=bridge.motion_lease_watchdog,
                                        args=(self.lease,), daemon=True)
            watchdog.start()
            latched = self.lease.cancel_event.wait(1)
            self.stop_burst.assert_not_called()
        watchdog.join(1)
        self.assertTrue(latched, "watchdog waited for a worker lock before latching")
        self.assertFalse(watchdog.is_alive())
        self.stop_burst.assert_called_once_with(3)

    def test_watchdog_latches_before_blocking_stop_transport(self):
        entered, release = threading.Event(), threading.Event()

        def blocked_stop(*_):
            entered.set()
            release.wait(2)

        self.stop_burst.side_effect = blocked_stop
        self.lease.deadline = time.monotonic() - 1
        watchdog = threading.Thread(target=bridge.motion_lease_watchdog,
                                    args=(self.lease,), daemon=True)
        watchdog.start()
        try:
            self.assertTrue(entered.wait(1))
            self.assertTrue(self.lease.cancel_event.is_set())
        finally:
            release.set()
            watchdog.join(1)
        self.assertFalse(watchdog.is_alive())

    def test_obsolete_watchdog_does_not_cancel_or_stop_replacement(self):
        replacement = bridge.MotionLease("replacement", "offline", time.monotonic() + 60)
        bridge._active_motion_lease = replacement
        self.lease.deadline = time.monotonic() - 1
        bridge.motion_lease_watchdog(self.lease)
        self.assertTrue(self.lease.cancel_event.is_set())
        self.assertFalse(replacement.cancel_event.is_set())
        self.assertFalse(bridge.state["motion_cancelled"])
        self.stop_burst.assert_not_called()

    def test_cancel_is_latched_before_stop_and_cannot_dispatch_after_return(self):
        self.enroll()
        observed = []
        self.stop_burst.side_effect = lambda *_: observed.append(self.lease.cancel_event.is_set())
        result = bridge.cancel_autonomous_mission(self.mission["id"], lease_id=self.lease.lease_id)
        self.assertTrue(result["cancelled"])
        self.assertEqual(observed, [True])
        self.assert_no_dispatch("forward", "curveleft", self.intent)

    def test_stale_cancel_identity_leaves_current_lease_untouched(self):
        for kwargs in ({"mission_id": "old-mission"},
                       {"mission_id": self.mission["id"], "lease_id": "old-lease"}):
            with self.subTest(kwargs=kwargs):
                self.assertFalse(bridge.cancel_autonomous_mission(**kwargs)["cancelled"])
        self.assertFalse(self.lease.cancel_event.is_set())
        self.stop_burst.assert_not_called()


if __name__ == "__main__":
    unittest.main()
