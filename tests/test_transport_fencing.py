"""Actual offline navigation worker + event-controlled app sockets.

Only observations and external I/O are faked. Navigation, packet construction,
leases, cancellation, watchdog and stop transactions execute production code.
The final test documents why host-side hardening does NOT lift the interlock.
"""
import importlib.util
from pathlib import Path
import threading
import time
import unittest
from typing import Any
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location(
    "beep_transport_fencing", Path(__file__).resolve().parents[1] / "beep_bridge/beep_bridge.py")
assert SPEC is not None and SPEC.loader is not None
bridge: Any = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


class SocketRig:
    def __init__(self, stage=None, behavior="block"):
        self.stage = stage
        self.behavior = behavior
        self.entered = threading.Event()
        self.release = threading.Event()
        self.writes = []
        self.connections = 0
        self.triggered = False
        self.nonblocking = []

    def gate(self, stage):
        if self.triggered or self.stage != stage:
            return False
        self.triggered = True
        self.entered.set()
        if not self.release.wait(3):
            raise AssertionError("test failed to release transport event")
        if self.behavior == "timeout":
            raise TimeoutError("injected transport timeout")
        return True

    def connect(self, *_args, **_kwargs):
        self.connections += 1
        number = self.connections
        if number == 1:
            self.gate("connect")
        rig = self

        class FakeSocket:
            def __enter__(self):
                return self

            def __exit__(self, *_):
                return False

            def settimeout(self, _value):
                pass

            def setblocking(self, value):
                rig.nonblocking.append(value)

            def recv(self, _size):
                if number == 1:
                    rig.gate("recv")
                return b"ready"

            def sendall(self, data):
                rig.writes.append((number, bytes(data)))

            def send(self, data):
                stage = "mode" if data == bridge.pkt(0x0F, [1]) else "motion"
                gated = number == 1 and rig.gate(stage)
                if gated and rig.behavior == "block":
                    # A full send buffer did not accept any bytes. Production
                    # must revalidate before retrying, not silently sendall.
                    raise BlockingIOError()
                if gated and rig.behavior == "partial":
                    rig.writes.append((number, bytes(data[:2])))
                    return 2
                rig.writes.append((number, bytes(data)))
                return len(data)

        return FakeSocket()


class TransportFencingTests(unittest.TestCase):
    def setUp(self):
        bridge._reset_motion_state_for_tests()
        bridge._reset_mission_state_for_tests()
        self.addCleanup(bridge._reset_motion_state_for_tests)
        self.addCleanup(bridge._reset_mission_state_for_tests)
        self.status = {
            "scan_seen": True, "scan_age_s": 0.01,
            "sectors": dict.fromkeys(
                ("front", "front_left", "front_right", "left", "right", "rear"), 2.0),
            "slam": {"usable": True, "pose_valid": True,
                     "pose_age_s": 0.01, "map_age_s": 0.01},
            "pose": {"x": 0, "y": 0, "yaw": 0},
        }
        self.lease = bridge.MotionLease("transport-lease", "offline", time.monotonic() + 60)
        self.mission = {
            "id": "transport-mission", "lease_id": self.lease.lease_id,
            "lease": self.lease, "mode": "explore", "state": "running",
            "duration_s": 0.15, "started_at": None, "finished_at": None,
            "intent_mailbox": bridge.IntentMailbox(),
        }
        bridge._active_motion_lease = self.lease
        bridge._active_mission = self.mission
        self.workers = []
        self.rig = SocketRig()
        patches: Any = (
            ("snapshot", {"side_effect": lambda *a, **k: self.status}),
            ("MOTOR_BACKEND", {"new": "app"}),
            ("SEMANTIC_INTENTS_ENABLED", {"new": True}),
            ("sdk_init", {"side_effect": AssertionError("SDK hardware forbidden")}),
        )
        for target, kwargs in patches:
            p = patch.object(bridge, target, **kwargs)
            p.start()
            self.addCleanup(p.stop)
        for target in ("socket.create_connection", "urllib.request.urlopen"):
            p = patch(target, side_effect=AssertionError("network forbidden"))
            p.start()
            self.addCleanup(p.stop)
        p = patch.object(bridge.socket, "create_connection", side_effect=lambda *a, **k: self.rig.connect(*a, **k))
        p.start()
        self.addCleanup(p.stop)
        self.addCleanup(self.cleanup_workers)

    def cleanup_workers(self):
        self.lease.cancel_event.set()
        self.rig.release.set()
        for worker in self.workers:
            worker.join(4)
            self.assertFalse(worker.is_alive(), "transport/ownership lock deadlock")

    def start(self, stage=None, behavior="block", curve=False):
        self.rig = SocketRig(stage, behavior)
        if curve:
            self.status["sectors"]["right"] = 1.0
        worker = threading.Thread(target=bridge._autonomous_mission_worker,
                                  args=(self.mission, self.lease, {"save": False}), daemon=True)
        self.workers.append(worker)
        worker.start()
        if stage:
            self.assertTrue(self.rig.entered.wait(2), "worker did not reach actual transport")
        return worker

    def finish(self, worker):
        self.rig.release.set()
        worker.join(4)
        self.assertFalse(worker.is_alive(), "worker failed to drain")
        self.assertIsNotNone(self.mission["finished_at"])

    def first_writes(self):
        return [packet for connection, packet in self.rig.writes if connection == 1]

    def enroll(self, expiry=None):
        # Set mailbox authority directly, independent of the parallel worker's
        # evolving admission schema. All execution/interlock paths are real.
        self.mission["intent_mailbox"].expires_at = expiry or time.time() + 10
        self.mission["semantic_deadline"] = time.monotonic() + 10

    def test_worker_happy_path_reaches_nonblocking_transport(self):
        worker = self.start()
        self.finish(worker)
        self.assertEqual(self.first_writes(), [bridge.pkt(0x0F, [1]), bridge.pkt(0x11, [0, 100])])
        self.assertTrue(self.rig.nonblocking)
        self.assertEqual(set(self.rig.nonblocking), {False})

    def test_cancel_during_connect_has_no_motion_transaction_writes(self):
        worker = self.start("connect")
        self.lease.cancel_event.set()
        self.finish(worker)
        self.assertEqual(self.first_writes(), [])

    def test_expiry_during_greeting_has_no_motion_transaction_writes(self):
        worker = self.start("recv")
        self.lease.deadline = time.monotonic() - 1
        self.finish(worker)
        self.assertEqual(self.first_writes(), [])
        self.assertTrue(self.lease.cancel_event.is_set())

    def test_greeting_timeout_aborts_without_writes(self):
        worker = self.start("recv", "timeout")
        self.finish(worker)
        self.assertEqual(self.first_writes(), [])
        self.assertTrue(self.lease.cancel_event.is_set())

    def test_cancel_during_mode_backpressure_does_not_retry(self):
        worker = self.start("mode")
        self.lease.cancel_event.set()
        self.finish(worker)
        self.assertEqual(self.first_writes(), [])

    def test_cancel_during_curve_translation_suppresses_translation_and_yaw(self):
        worker = self.start("motion", curve=True)
        self.lease.cancel_event.set()
        self.finish(worker)
        self.assertEqual(self.first_writes(), [bridge.pkt(0x0F, [1])])

    def test_write_timeout_suppresses_remaining_packets(self):
        worker = self.start("motion", "timeout", curve=True)
        self.finish(worker)
        self.assertEqual(self.first_writes(), [bridge.pkt(0x0F, [1])])
        self.assertTrue(self.lease.cancel_event.is_set())

    def test_partial_write_is_not_retried_after_cancellation(self):
        worker = self.start("motion", "partial", curve=True)
        self.lease.cancel_event.set()
        self.finish(worker)
        # Bytes accepted by an in-flight call cannot be recalled. No remainder
        # or later yaw packet may be sent after that call returns.
        self.assertEqual(self.first_writes(), [bridge.pkt(0x0F, [1]), b"$0"])

    def test_collision_change_during_backpressure_vetoes_retry(self):
        worker = self.start("motion")
        self.status["sectors"]["front"] = 0.1
        self.finish(worker)
        self.assertEqual(self.first_writes(), [bridge.pkt(0x0F, [1])])
        self.assertTrue(self.lease.cancel_event.is_set())

    def test_stale_sensors_after_connect_veto_all_packets(self):
        worker = self.start("connect")
        self.status["scan_age_s"] = 30
        self.finish(worker)
        self.assertEqual(self.first_writes(), [])

    def test_replaced_owner_neither_moves_nor_stops_replacement(self):
        worker = self.start("recv")
        replacement = bridge.MotionLease("new-lease", "offline", time.monotonic() + 60)
        bridge._active_motion_lease = replacement
        self.finish(worker)
        self.assertEqual(self.rig.writes, [])
        self.assertIs(bridge.active_motion_lease(), replacement)
        self.assertFalse(replacement.cancel_event.is_set())

    def test_enrollment_during_connect_closes_transport_interlock(self):
        worker = self.start("connect")
        self.enroll()
        self.finish(worker)
        self.assertEqual(self.first_writes(), [])
        self.assertFalse(bridge.mission_snapshot()["semantic_execution_enabled"])

    def test_expired_semantic_evidence_after_connect_cannot_write(self):
        worker = self.start("connect")
        self.enroll(time.time() - 1)
        self.finish(worker)
        self.assertEqual(self.first_writes(), [])

    def test_watchdog_latches_without_waiting_for_blocked_worker_transport(self):
        worker = self.start("recv")
        self.lease.deadline = time.monotonic() - 1
        watchdog = threading.Thread(target=bridge.motion_lease_watchdog, args=(self.lease,), daemon=True)
        self.workers.append(watchdog)
        watchdog.start()
        self.assertTrue(self.lease.cancel_event.wait(1), "watchdog latch blocked behind I/O")
        self.assertTrue(watchdog.is_alive())
        self.finish(worker)
        watchdog.join(4)
        self.assertFalse(watchdog.is_alive())
        self.assertEqual(self.first_writes(), [])

    def test_mission_cancel_latches_and_drains_without_lock_inversion(self):
        worker = self.start("recv")
        results = []
        cancel = threading.Thread(target=lambda: results.append(
            bridge.cancel_autonomous_mission(self.mission["id"], lease_id=self.lease.lease_id)), daemon=True)
        self.workers.append(cancel)
        cancel.start()
        self.assertTrue(self.lease.cancel_event.wait(1))
        self.finish(worker)
        cancel.join(4)
        self.assertFalse(cancel.is_alive())
        self.assertTrue(results[0]["cancelled"])
        self.assertEqual(self.first_writes(), [])

    def test_stop_preempts_stalled_greeting_before_motion_writes(self):
        """Host stop I/O must not queue behind a stalled motion greeting."""
        worker = self.start("recv")
        completed = threading.Event()
        result = []
        stopper = threading.Thread(target=lambda: (result.append(bridge.request_stop("phase_c_probe", n=1)),
                                                   completed.set()), daemon=True)
        self.workers.append(stopper)
        stopper.start()
        self.assertTrue(self.lease.cancel_event.wait(1), "stop cancellation latch delayed")
        self.assertTrue(completed.wait(.6), "stop packet queued behind greeting")
        self.assertTrue(worker.is_alive(), "movement greeting was not held during stop")
        self.assertIsNone(result[0])
        self.assertEqual(self.first_writes(), [])
        self.assertEqual(bridge.state["last_command"], "app:stop")
        self.assertFalse(bridge.state["moving"])
        self.finish(worker)
        self.assertEqual(bridge.state["last_command"], "app:stop")
        self.assertFalse(bridge.state["moving"])
        stopper.join(4)
        self.assertFalse(stopper.is_alive())

    def test_stop_preempts_stalled_connect_before_motion_writes(self):
        worker = self.start("connect")
        completed = threading.Event()
        stopper = threading.Thread(target=lambda: (bridge.request_stop("phase_c_connect", n=1),
                                                   completed.set()), daemon=True)
        self.workers.append(stopper)
        stopper.start()
        self.assertTrue(self.lease.cancel_event.wait(1))
        self.assertTrue(completed.wait(.6), "stop packet queued behind connect")
        self.assertEqual(self.first_writes(), [])
        self.finish(worker)

    def test_stop_still_waits_for_already_entered_write_known_limit(self):
        worker = self.start("motion", "late_accept", curve=True)
        completed = threading.Event()
        stopper = threading.Thread(target=lambda: (bridge.request_stop("phase_c_inflight", n=1),
                                                   completed.set()), daemon=True)
        self.workers.append(stopper)
        stopper.start()
        self.assertTrue(self.lease.cancel_event.wait(1))
        self.assertFalse(completed.wait(.05), "in-flight wire call was not serialized")
        self.finish(worker)
        self.assertTrue(completed.wait(2))
        self.assertEqual(self.first_writes(), [bridge.pkt(0x0F, [1]),
                                               bridge.pkt(0x11, [0, 100])])
        self.assertEqual(bridge.state["last_command"], "app:stop")

    def test_inflight_write_is_not_revocable_so_semantic_interlock_stays_closed(self):
        worker = self.start("motion", "late_accept", curve=True)
        self.lease.cancel_event.set()
        self.finish(worker)
        # Deliberately adversarial transport accepts an already-entered call
        # after cancellation. This is a limitation witness, NOT a safety pass:
        # no host check can retroactively revoke it. Yaw is still suppressed.
        self.assertEqual(self.first_writes(), [bridge.pkt(0x0F, [1]), bridge.pkt(0x11, [0, 100])])
        self.assertFalse(bridge.mission_snapshot()["semantic_execution_enabled"])


if __name__ == "__main__":
    unittest.main()
