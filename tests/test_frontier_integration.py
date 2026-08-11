import importlib.util
import json
from pathlib import Path
import threading
import time
import unittest
import urllib.request
from unittest.mock import MagicMock, call, patch

from beep_bridge.frontier_planner import OccupancyGrid


BRIDGE_PATH = Path(__file__).resolve().parents[1] / "beep_bridge" / "beep_bridge.py"
SPEC = importlib.util.spec_from_file_location("beep_bridge_frontier_integration", BRIDGE_PATH)
assert SPEC is not None and SPEC.loader is not None
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


class FrontierIntegrationTests(unittest.TestCase):
    def setUp(self):
        bridge._reset_motion_state_for_tests()
        bridge._reset_mission_state_for_tests()

    def tearDown(self):
        bridge._reset_motion_state_for_tests()
        bridge._reset_mission_state_for_tests()

    def test_independent_watchdog_stops_exact_expired_lease(self):
        lease = bridge.MotionLease("watchdog-1", "unit_test", time.monotonic() - 0.01)
        with bridge.motion_owner_lock:
            setattr(bridge, "_active_motion_lease", lease)
        with patch.object(bridge, "stop_burst", return_value=None) as stop:
            bridge.motion_lease_watchdog(lease)

        self.assertTrue(lease.cancel_event.is_set())
        self.assertTrue(bridge.state["motion_cancelled"])
        stop.assert_called_once_with(3)

    def test_watchdog_never_stops_an_ended_lease(self):
        lease = bridge.MotionLease("watchdog-old", "unit_test", time.monotonic() - 0.01)
        lease.ended_event.set()
        with patch.object(bridge, "stop_burst", return_value=None) as stop:
            bridge.motion_lease_watchdog(lease)
        stop.assert_not_called()

    def test_camera_capture_during_motion_never_sends_app_stop(self):
        jpeg = b"\xff\xd8moving-frame\xff\xd9"
        control_socket = MagicMock()

        class Response:
            def __enter__(self): return self
            def __exit__(self, *_): return False
            def read(self, _size): return jpeg

        with patch.object(bridge, "active_motion_lease", return_value=object()), \
             patch.object(bridge.socket, "create_connection", return_value=control_socket), \
             patch.object(bridge.urllib.request, "urlopen", return_value=Response()):
            frame = bridge.capture_frame(timeout=1)
        self.assertEqual(frame, jpeg)
        sent = [entry.args[0] for entry in control_socket.sendall.call_args_list]
        self.assertEqual(sent, [bridge.pkt(0x0F, [0x01])])
        self.assertNotIn(bridge.pkt(0x12, [bridge.CMD_PAYLOAD["stop"]]), sent)

    def test_app_backward_uses_neutralized_negative_x_analog_packet(self):
        sock = MagicMock()
        sock.recv.side_effect = TimeoutError()
        connection = MagicMock()
        connection.__enter__.return_value = sock
        with patch.object(bridge.socket, "create_connection", return_value=connection):
            bridge.app_send("backward")
        sent = [entry.args[0] for entry in sock.sendall.call_args_list]
        self.assertEqual(sent, [
            bridge.pkt(0x0F, [0x01]),
            bridge.pkt(0x13, [0x64]),
            bridge.pkt(0x14, [bridge.APP_REVERSE_PACE]),
            bridge.pkt(0x11, [0x00, 0x9C]),
        ])

    def test_app_stop_neutralizes_both_axes_before_button_stop(self):
        sock = MagicMock()
        sock.recv.side_effect = TimeoutError()
        connection = MagicMock()
        connection.__enter__.return_value = sock
        with patch.object(bridge.socket, "create_connection", return_value=connection):
            bridge.app_send("stop")
        sent = [entry.args[0] for entry in sock.sendall.call_args_list]
        self.assertEqual(sent, [
            bridge.pkt(0x0F, [0x01]),
            bridge.pkt(0x11, [0x00, 0x00]),
            bridge.pkt(0x12, [0x00]),
            bridge.pkt(0x13, [0x32]),
            bridge.pkt(0x14, [0x02]),
        ])

    def test_app_reverse_selects_high_walk_before_analog_command(self):
        dog = MagicMock()
        with patch.object(bridge, "MOTOR_BACKEND", "app"), \
             patch.object(bridge, "APP_REVERSE_GAIT", "high_walk"), \
             patch.object(bridge, "sdk_init", return_value=dog), \
             patch.object(bridge, "app_send") as app:
            bridge.motor_send("backward")
        dog.gait_type.assert_called_once_with("high_walk")
        app.assert_called_once_with("backward")

    def test_perception_and_transport_failures_do_not_cancel_motion(self):
        self.assertFalse(bridge.handler_failure_requires_stop("/frame.jpg", TimeoutError("camera timeout")))
        self.assertFalse(bridge.handler_failure_requires_stop("/camera.jpg", RuntimeError("bad frame")))
        self.assertFalse(bridge.handler_failure_requires_stop("/status", BrokenPipeError("client left")))
        self.assertFalse(bridge.handler_failure_requires_stop("/mission", ConnectionResetError("client left")))
        self.assertTrue(bridge.handler_failure_requires_stop("/move", RuntimeError("motor handler failed")))
        self.assertTrue(bridge.handler_failure_requires_stop("/coverage_explore", TimeoutError("controller failed")))

    def test_async_mission_returns_while_worker_moves_and_can_be_cancelled(self):
        entered = threading.Event()
        release = threading.Event()

        def fake_walk(**_kwargs):
            entered.set()
            release.wait(2.0)
            cancelled = bridge.motion_is_cancelled()
            return {"ok": not cancelled, "reason": "motion_cancelled" if cancelled else "max_duration", "elapsed_s": 0.1}

        with patch.object(bridge, "snapshot", return_value={
                 "scan_seen": True, "scan_age_s": 0.01, "slam": {"usable": True},
                 "sectors": {"front": 1.2, "front_left": 0.8, "front_right": 0.8,
                             "left": 0.8, "right": 0.8, "rear": 0.8}}), \
             patch.object(bridge, "scan_ok", return_value=True), \
             patch.object(bridge, "slam_ok", return_value=True), \
             patch.object(bridge, "lidar_walk", side_effect=fake_walk), \
             patch.object(bridge, "stop_burst", return_value=None):
            accepted = bridge.start_autonomous_mission("coverage", 90, save=False)
            self.assertTrue(accepted["accepted"])
            self.assertTrue(entered.wait(0.5))
            running = bridge.mission_snapshot()
            self.assertEqual(running["state"], "running")
            self.assertEqual(running["duration_s"], 90.0)
            self.assertIsNotNone(bridge.active_motion_lease())
            with bridge.mission_lock:
                thread = bridge._active_mission["thread"]

            stale = bridge.cancel_autonomous_mission("mission-stale")
            self.assertFalse(stale["cancelled"])
            self.assertEqual(stale["reason"], "mission_id_mismatch")
            self.assertFalse(bridge.active_motion_lease().cancel_event.is_set())
            cancelled = bridge.cancel_autonomous_mission(running["id"])
            self.assertTrue(cancelled["cancelled"])
            release.set()
            thread.join(1.0)

        self.assertFalse(thread.is_alive())
        self.assertEqual(bridge.mission_snapshot()["state"], "cancelled")
        self.assertIsNone(bridge.active_motion_lease())

    def test_friday_demo_endpoint_remains_available(self):
        self.assertIn('p.path in ("/lidar_walk", "/demo_walk")', BRIDGE_PATH.read_text())

    def test_http_mission_control_remains_responsive_while_worker_runs(self):
        entered = threading.Event()
        release = threading.Event()

        def fake_walk(**_kwargs):
            entered.set()
            release.wait(2.0)
            cancelled = bridge.motion_is_cancelled()
            return {"ok": not cancelled, "reason": "motion_cancelled" if cancelled else "max_duration", "elapsed_s": 0.1}

        server = bridge.ThreadingHTTPServer(("127.0.0.1", 0), bridge.Handler)
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        base = f"http://127.0.0.1:{server.server_address[1]}"

        def request(path, body=None):
            data = None if body is None else json.dumps(body).encode()
            req = urllib.request.Request(base + path, data=data, headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=1.0) as response:
                return response.status, json.load(response)

        try:
            with patch.object(bridge, "snapshot", return_value={
                 "scan_seen": True, "scan_age_s": 0.01, "slam": {"usable": True},
                 "sectors": {"front": 1.2, "front_left": 0.8, "front_right": 0.8,
                             "left": 0.8, "right": 0.8, "rear": 0.8}}), \
                 patch.object(bridge, "scan_ok", return_value=True), \
                 patch.object(bridge, "slam_ok", return_value=True), \
                 patch.object(bridge, "lidar_walk", side_effect=fake_walk), \
                 patch.object(bridge, "stop_burst", return_value=None):
                started_at = time.monotonic()
                code, accepted = request("/mission/start", {"mode": "coverage", "duration_s": 90})
                self.assertLess(time.monotonic() - started_at, 0.5)
                self.assertEqual(code, 202)
                self.assertTrue(accepted["accepted"])
                self.assertTrue(entered.wait(0.5))

                code, status = request("/mission")
                self.assertEqual(code, 200)
                self.assertEqual(status["mission"]["state"], "running")
                code, cancelled = request("/mission/cancel", {"mission_id": status["mission"]["id"]})
                self.assertEqual(code, 200)
                self.assertTrue(cancelled["cancelled"])
                release.set()
                with bridge.mission_lock:
                    worker = bridge._active_mission["thread"]
                worker.join(1.0)
                self.assertFalse(worker.is_alive())
        finally:
            release.set()
            server.shutdown()
            server.server_close()
            server_thread.join(1.0)

    def test_request_stop_cancels_owned_lease_before_motor_stop(self):
        observations = []
        lease = bridge.begin_motion("unit_test")

        def fake_stop(n=3):
            observations.append((lease.cancel_event.is_set(), n))
            return None

        with patch.object(bridge, "stop_burst", side_effect=fake_stop):
            error = bridge.request_stop("unit_test")

        self.assertIsNone(error)
        self.assertTrue(lease.cancel_event.is_set())
        self.assertTrue(bridge.state["motion_cancelled"])
        self.assertEqual(observations, [(True, 3)])

    def test_cancelled_lease_cannot_be_revived_by_new_lease(self):
        first = bridge.begin_motion("first")
        with patch.object(bridge, "stop_burst", return_value=None):
            bridge.request_stop("unit_test")
        bridge.end_motion(first)

        second = bridge.begin_motion("second")

        self.assertTrue(first.cancel_event.is_set())
        self.assertFalse(second.cancel_event.is_set())
        self.assertFalse(bridge.motion_is_cancelled())

    def test_overlapping_motion_is_rejected_instead_of_queued(self):
        first = bridge.begin_motion("first")

        with self.assertRaises(bridge.MotionBusy):
            bridge.begin_motion("second")

        self.assertIs(bridge.active_motion_lease(), first)

    def test_fresh_pose_without_fresh_map_is_not_usable_slam(self):
        original = dict(bridge.state["slam"])
        try:
            now = bridge.time.time()
            bridge.state["slam"] = {
                "pose_at": now,
                "pose_valid": True,
                "map_at": None,
                "map_width": None,
                "map_height": None,
                "resolution_m": None,
            }
            missing_map = bridge.snapshot()["slam"]
            self.assertTrue(missing_map["active"])
            self.assertFalse(missing_map["usable"])
            self.assertEqual(missing_map["usable_reason"], "map_missing_stale_or_empty")

            bridge.state["slam"].update({"map_at": now, "map_width": 20, "map_height": 20, "resolution_m": 0.05})
            self.assertTrue(bridge.snapshot()["slam"]["usable"])
        finally:
            bridge.state["slam"] = original

    def test_lidar_forward_supervisor_honors_persistent_cancellation(self):
        clear = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "sectors": {"front": 1.2, "front_left": 0.8, "front_right": 0.8},
        }
        lease = bridge.begin_motion("unit_test")
        lease.cancel_event.set()
        with patch.object(bridge, "snapshot", return_value=clear):
            result = bridge.supervise_lidar_forward(1.0)

        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "motion_cancelled")

    def test_manual_non_stop_move_rejects_zero_duration(self):
        with self.assertRaisesRegex(ValueError, "positive bounded duration"):
            bridge.run_action("forward", 0.0)

    def test_run_action_stop_latches_cancellation(self):
        lease = bridge.begin_motion("unit_test")
        with patch.object(bridge, "stop_burst", return_value=None) as stop:
            result = bridge.run_action("stop", 0.2)

        self.assertTrue(result["ok"])
        self.assertEqual(result["reason"], "motion_cancelled")
        self.assertTrue(lease.cancel_event.is_set())
        stop.assert_called_once_with(3)

    def test_coverage_plateau_requires_full_window_and_low_map_growth(self):
        self.assertFalse(bridge.coverage_has_plateaued([(0, 100), (20, 110)], 20, 45, 150))
        self.assertTrue(bridge.coverage_has_plateaued([(1, 1000), (23, 1040), (46, 1070)], 46, 45, 150))
        self.assertFalse(bridge.coverage_has_plateaued([(1, 1000), (23, 1120), (46, 1300)], 46, 45, 150))

    def test_sdk_curve_combines_forward_and_yaw_then_straightens_without_stop(self):
        dog = MagicMock()
        original = dict(bridge.state)
        try:
            with patch.object(bridge, "sdk_init", return_value=dog):
                bridge.sdk_curve("right", forward_step=20, yaw_step=30)
                bridge.sdk_straighten()
            dog.gait_type.assert_called_once_with(bridge.SDK_GAIT)
            dog.pace.assert_called_once_with(bridge.SDK_PACE)
            dog.move_x.assert_called_once_with(20)
            self.assertEqual(dog.turn.call_args_list, [call(-30), call(0)])
            self.assertTrue(bridge.state["moving"])
            self.assertEqual(bridge.state["last_command"], "sdk:forward")
        finally:
            bridge.state.clear()
            bridge.state.update(original)

    def test_lidar_forward_supervisor_stops_on_fresh_close_obstacle(self):
        close = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "sectors": {"front": 0.30, "front_left": 0.8, "front_right": 0.8,
                        "left": 0.8, "right": 0.8, "rear": 0.8},
        }
        with patch.object(bridge, "snapshot", return_value=close):
            result = bridge.supervise_lidar_forward(1.0)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "obstacle_during_lidar_walk")

    def test_pose_guard_rejects_stationary_drift_but_keeps_raw_diagnosable(self):
        accepted, anchor, valid, reason = bridge.guard_slam_pose(None, None, (0.0, 0.0, 0.0), False)
        self.assertTrue(valid)
        accepted, anchor, valid, reason = bridge.guard_slam_pose(accepted, anchor, (0.03, -0.02, 0.03), False)
        self.assertTrue(valid)
        guarded, anchor, valid, reason = bridge.guard_slam_pose(accepted, anchor, (1.4, -0.8, 0.5), False)
        self.assertFalse(valid)
        self.assertEqual(reason, "stationary_drift")
        self.assertEqual(guarded, accepted)

    def test_pose_guard_accepts_plausible_motion_and_rejects_teleport(self):
        accepted = (0.0, 0.0, 0.0)
        moved, anchor, valid, reason = bridge.guard_slam_pose(accepted, accepted, (0.12, 0.02, 0.20), True)
        self.assertTrue(valid)
        self.assertEqual(reason, "moving_update")
        guarded, anchor, valid, reason = bridge.guard_slam_pose(moved, anchor, (2.0, 0.0, 2.0), True)
        self.assertFalse(valid)
        self.assertEqual(reason, "impossible_moving_jump")
        self.assertEqual(guarded, moved)

    def test_app_socket_timeout_does_not_invalidate_successful_sdk_stop(self):
        with patch.object(bridge, "sdk_send") as sdk_send, \
             patch.object(bridge, "app_send", side_effect=TimeoutError("camera socket")), \
             patch.object(bridge.time, "sleep"):
            error = bridge.stop_burst(3)

        self.assertIsNone(error)
        self.assertEqual(sdk_send.call_count, 3)
        self.assertFalse(bridge.snapshot()["moving"])

    def test_sdk_stop_failure_remains_a_motor_error(self):
        with patch.object(bridge, "sdk_send", side_effect=RuntimeError("sdk unavailable")), \
             patch.object(bridge, "app_send"), \
             patch.object(bridge.time, "sleep"):
            error = bridge.stop_burst(2)

        self.assertIn("sdk unavailable", error)

    def test_consecutive_forward_windows_start_gait_only_once(self):
        commands = []
        with patch.object(bridge, "motor_send", side_effect=lambda action, step=None: commands.append((action, step))):
            action, first_started = bridge.start_or_continue_fluent_forward(None, 20)
            action, second_started = bridge.start_or_continue_fluent_forward(action, 20)

        self.assertTrue(first_started)
        self.assertFalse(second_started)
        self.assertEqual(action, "forward")
        self.assertEqual(commands, [("forward", 20)])

    def test_bounded_frontier_loop_plans_motion_and_finishes_stopped(self):
        width = height = 15
        data = [-1] * (width * height)
        for y in range(4, 11):
            for x in range(4, 11):
                data[y * width + x] = 0
        setattr(bridge, "slam_grid", OccupancyGrid(width, height, 0.10, 0.0, 0.0, data))
        commands = []
        stops = []
        clock = {"value": 0.0}

        def now():
            clock["value"] += 0.11
            return clock["value"]

        status = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "slam": {"active": True, "pose_valid": True, "usable": True, "pose_age_s": 0.02, "map_age_s": 0.05},
            "sectors": {"front": 1.4, "front_left": 1.0, "front_right": 1.0,
                        "left": 1.2, "right": 1.1, "rear": 1.0},
        }
        pose = {"x": 0.7, "y": 0.7, "yaw": 0.0, "source": "cartographer_slam", "confidence": 0.95, "scan_match_score": None}

        with patch.object(bridge.time, "time", side_effect=now), \
             patch.object(bridge.time, "time_ns", return_value=123456), \
             patch.object(bridge.time, "sleep", return_value=None), \
             patch.object(bridge, "snapshot", side_effect=lambda full=False: dict(status)), \
             patch.object(bridge, "pose_copy", side_effect=lambda: dict(pose)), \
             patch.object(bridge, "motor_send", side_effect=lambda action, step=None: commands.append((action, step))), \
             patch.object(bridge, "stop_burst", side_effect=lambda n=3: stops.append(n)):
            result = bridge.frontier_explore(max_duration=1.0, chaos=0.45, seed=7, save=False)

        self.assertEqual(result["mode"], "frontier_explore")
        self.assertIn(result["reason"], ("max_duration", "turn_no_progress"))
        self.assertEqual(result["ok"], result["reason"] == "max_duration")
        self.assertTrue(any(action != "stop" for action, _ in commands))
        self.assertGreaterEqual(len(commands), 1)
        self.assertEqual(stops[-1], 3)
        self.assertTrue(any(item.get("event") == "frontier_selected" for item in result["trace_tail"]))
        self.assertTrue(any(item.get("event") == "motion" for item in result["trace_tail"]))


if __name__ == "__main__":
    unittest.main()
