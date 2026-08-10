import math
import unittest
from unittest.mock import patch

from beep_bridge import beep_bridge as bridge


class MarkObjectPlanTests(unittest.TestCase):
    def setUp(self):
        bridge._reset_motion_state_for_tests()

    def tearDown(self):
        bridge._reset_motion_state_for_tests()

    def test_dead_reckoning_does_not_overwrite_fresh_guarded_slam_pose(self):
        original_pose = dict(bridge.state["pose"])
        original_slam = dict(bridge.state["slam"])
        try:
            bridge.state["pose"] = {"x": 1.0, "y": 2.0, "yaw": 0.3, "source": "guarded_cartographer_slam", "confidence": 0.9, "scan_match_score": None}
            bridge.state["slam"] = {"pose_at": bridge.time.time()}
            bridge.update_pose_for_action("turnright", 1.3)
            self.assertEqual(bridge.state["pose"]["source"], "guarded_cartographer_slam")
            self.assertEqual(bridge.state["pose"]["yaw"], 0.3)
        finally:
            bridge.state["pose"] = original_pose
            bridge.state["slam"] = original_slam

    def test_default_plan_turns_left_90_degrees_before_right_leg_action(self):
        result = bridge.mark_object(dry_run=True)

        plan = result["plan"]
        self.assertEqual(plan["target_front_m"], 0.55)
        self.assertEqual(plan["approach_mode"], "continuous")
        self.assertEqual(plan["turn"], "left")
        self.assertEqual(plan["turn_degrees"], 90.0)
        self.assertEqual(plan["turn_control"], "guarded_slam_yaw")
        self.assertEqual(plan["marking_side"], "right")
        self.assertEqual(plan["trick"]["name"], "pee")
        self.assertEqual(plan["trick"]["id"], 11)
        self.assertEqual(plan["trick"]["duration_s"], 8.0)

    def test_guarded_slam_turn_stops_at_measured_90_degrees(self):
        yaws = iter([0.0, 0.0, 0.45, 0.95, 1.50, 1.50])
        commands = []
        stops = []

        def fake_snapshot(*args, **kwargs):
            try:
                yaw = next(yaws)
            except StopIteration:
                yaw = 1.50
            return {
                "scan_seen": True,
                "scan_age_s": 0.01,
                "sectors": {name: 0.8 for name in ("front", "front_left", "front_right", "left", "right", "rear")},
                "slam": {"active": True, "pose_valid": True, "usable": True},
                "pose": {"yaw": yaw},
            }

        original_snapshot = bridge.snapshot
        original_motor_send = bridge.motor_send
        original_stop_burst = bridge.stop_burst
        original_sleep = bridge.time.sleep
        try:
            setattr(bridge, "snapshot", fake_snapshot)
            setattr(bridge, "motor_send", lambda action, step=None: commands.append((action, step)))
            setattr(bridge, "stop_burst", lambda n=3: stops.append(n))
            bridge.time.sleep = lambda seconds: None
            result = bridge.guarded_slam_turn(turn="left", degrees=90, max_duration=2)
        finally:
            setattr(bridge, "snapshot", original_snapshot)
            setattr(bridge, "motor_send", original_motor_send)
            setattr(bridge, "stop_burst", original_stop_burst)
            bridge.time.sleep = original_sleep

        self.assertTrue(result["ok"])
        self.assertTrue(result["reason"].startswith("target_reached"))
        self.assertEqual(commands, [("turnleft", None)])
        self.assertEqual(stops, [3])
        self.assertGreaterEqual(result["trace_tail"][-1]["progress_degrees"], 85.0)

    def test_guarded_slam_turn_rejects_wrong_direction(self):
        yaws = iter([0.0, -0.25, -0.25])
        commands = []
        stops = []

        def fake_snapshot(*args, **kwargs):
            try:
                yaw = next(yaws)
            except StopIteration:
                yaw = -0.25
            return {
                "scan_seen": True,
                "scan_age_s": 0.01,
                "sectors": {name: 0.8 for name in ("front", "front_left", "front_right", "left", "right", "rear")},
                "slam": {"active": True, "pose_valid": True, "usable": True},
                "pose": {"yaw": yaw},
            }

        original_snapshot = bridge.snapshot
        original_motor_send = bridge.motor_send
        original_stop_burst = bridge.stop_burst
        original_sleep = bridge.time.sleep
        try:
            setattr(bridge, "snapshot", fake_snapshot)
            setattr(bridge, "motor_send", lambda action, step=None: commands.append((action, step)))
            setattr(bridge, "stop_burst", lambda n=3: stops.append(n))
            bridge.time.sleep = lambda seconds: None
            result = bridge.guarded_slam_turn(turn="left", degrees=90, max_duration=2)
        finally:
            setattr(bridge, "snapshot", original_snapshot)
            setattr(bridge, "motor_send", original_motor_send)
            setattr(bridge, "stop_burst", original_stop_burst)
            bridge.time.sleep = original_sleep

        self.assertFalse(result["ok"])
        self.assertTrue(result["reason"].startswith("wrong_direction"))
        self.assertEqual(commands, [("turnleft", None)])
        self.assertEqual(stops, [3])

    def test_guarded_slam_turn_rejects_missing_sector_before_motor_command(self):
        status = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "sectors": {name: 0.8 for name in ("front", "front_left", "front_right", "left", "right")},
            "slam": {"active": True, "pose_valid": True, "usable": True},
            "pose": {"yaw": 0.0},
        }
        commands = []
        with patch.object(bridge, "snapshot", return_value=status), \
             patch.object(bridge, "motor_send", side_effect=lambda action, step=None: commands.append((action, step))):
            result = bridge.guarded_slam_turn(turn="left", degrees=90, max_duration=2)

        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "lidar_sector_missing_before_start")
        self.assertEqual(commands, [])

    def test_gesture_exception_attempts_stop_and_clears_moving_state(self):
        class BrokenDog:
            def action(self, action_id):
                raise RuntimeError("vendor action failure")

        with patch.object(bridge, "sdk_init", return_value=BrokenDog()), \
             patch.object(bridge, "stop_burst", return_value=None) as stop:
            result = bridge.sdk_trick(name="prey")

        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "gesture_exception")
        self.assertIn("vendor action failure", result["error"])
        self.assertFalse(bridge.state["moving"])
        stop.assert_called_once_with(3)

    def test_continuous_approach_aborts_on_heading_drift(self):
        snapshots = iter([
            {"scan_seen": True, "scan_age_s": 0.01,
             "sectors": {name: 0.8 for name in ("front", "front_left", "front_right", "left", "right", "rear")},
             "slam": {"usable": True}, "pose": {"yaw": 0.0}},
            {"scan_seen": True, "scan_age_s": 0.01,
             "sectors": {name: 0.8 for name in ("front", "front_left", "front_right", "left", "right", "rear")},
             "slam": {"usable": True}, "pose": {"yaw": math.radians(18.0)}},
        ])
        commands = []
        with patch.object(bridge, "snapshot", side_effect=lambda: next(snapshots)), \
             patch.object(bridge, "motor_send", side_effect=lambda action, step=None: commands.append((action, step))), \
             patch.object(bridge, "stop_burst", return_value=None), \
             patch.object(bridge.time, "sleep", return_value=None):
            result = bridge.forward_continuous_until(target_front=0.2, max_duration=1.0)

        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "heading_drift:18.0deg")
        self.assertEqual(commands, [("forward", None)])

    def test_sdk_forward_neutralizes_lateral_and_yaw_axes(self):
        calls = []

        class FakeDog:
            def gait_type(self, value): calls.append(("gait_type", value))
            def pace(self, value): calls.append(("pace", value))
            def move_x(self, value): calls.append(("move_x", value))
            def move_y(self, value): calls.append(("move_y", value))
            def turn(self, value): calls.append(("turn", value))
            def forward(self, value): calls.append(("forward", value))
            def turnleft(self, value): calls.append(("turnleft", value))

        with patch.object(bridge, "sdk_init", return_value=FakeDog()):
            bridge.sdk_send("forward", step=10)
            bridge.sdk_send("turnleft", step=10)

        self.assertEqual(calls, [
            ("gait_type", bridge.SDK_GAIT), ("pace", bridge.SDK_PACE),
            ("move_y", 0), ("turn", 0), ("forward", 10),
            ("gait_type", bridge.SDK_GAIT), ("pace", bridge.SDK_PACE),
            ("move_x", 0), ("move_y", 0), ("turnleft", 10),
        ])

    def test_continuous_approach_starts_forward_once_and_stops_at_target(self):
        readings = iter([1.20, 0.80, 0.55])
        commands = []
        stops = []

        def fake_snapshot(*args, **kwargs):
            return {
                "scan_seen": True,
                "scan_age_s": 0.01,
                "sectors": {"front": next(readings), "front_left": 0.8, "front_right": 0.8,
                            "left": 0.8, "right": 0.8, "rear": 0.8},
                "slam": {"active": True, "pose_valid": True, "usable": True},
                "pose": {"yaw": 0.0},
            }

        original_snapshot = getattr(bridge, "snapshot")
        original_motor_send = getattr(bridge, "motor_send")
        original_stop_burst = getattr(bridge, "stop_burst")
        original_sleep = bridge.time.sleep
        try:
            setattr(bridge, "snapshot", fake_snapshot)
            setattr(bridge, "motor_send", lambda action, step=None: commands.append((action, step)))
            setattr(bridge, "stop_burst", lambda n=3: stops.append(n))
            bridge.time.sleep = lambda seconds: None

            result = bridge.forward_continuous_until(target_front=0.55, max_duration=2.0)
        finally:
            setattr(bridge, "snapshot", original_snapshot)
            setattr(bridge, "motor_send", original_motor_send)
            setattr(bridge, "stop_burst", original_stop_burst)
            bridge.time.sleep = original_sleep

        self.assertTrue(result["ok"])
        self.assertTrue(result["reason"].startswith("target_reached"))
        self.assertEqual(commands, [("forward", None)])
        self.assertEqual(stops, [3])

    def test_explorer_uses_visible_forward_stride_when_path_is_clear(self):
        action, duration, reason = bridge.choose_explore_action({
            "front": 1.2,
            "front_left": 0.8,
            "front_right": 0.8,
            "left": 0.7,
            "right": 0.7,
            "rear": 0.7,
        })

        self.assertEqual(action, "forward")
        self.assertEqual(duration, 0.5)
        self.assertEqual(reason, "full_body_corridor_clear")

    def test_quaternion_to_yaw_extracts_planar_slam_heading(self):
        yaw = math.pi / 2
        actual = bridge.quaternion_to_yaw(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))

        self.assertAlmostEqual(actual, yaw, places=6)


if __name__ == "__main__":
    unittest.main()