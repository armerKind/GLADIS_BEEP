import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from beep_bridge.frontier_planner import OccupancyGrid


BRIDGE_PATH = Path(__file__).resolve().parents[1] / "beep_bridge" / "beep_bridge.py"
SPEC = importlib.util.spec_from_file_location("beep_bridge_frontier_integration", BRIDGE_PATH)
assert SPEC is not None and SPEC.loader is not None
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


class FrontierIntegrationTests(unittest.TestCase):
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
            "slam": {"active": True, "pose_age_s": 0.02, "map_age_s": 0.05},
            "sectors": {"front": 1.4, "front_left": 1.0, "front_right": 1.0, "left": 1.2, "right": 1.1},
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
        self.assertIn(result["reason"], ("max_duration", "turn_progress_stalled"))
        self.assertEqual(result["ok"], result["reason"] != "turn_progress_stalled")
        self.assertTrue(any(action != "stop" for action, _ in commands))
        self.assertGreaterEqual(len(commands), 1)
        self.assertEqual(stops[-1], 3)
        self.assertTrue(any(item.get("event") == "frontier_selected" for item in result["trace_tail"]))
        self.assertTrue(any(item.get("event") == "motion" for item in result["trace_tail"]))


if __name__ == "__main__":
    unittest.main()
