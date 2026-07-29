import importlib.util
from pathlib import Path
import math
import unittest


MODULE_PATH = Path(__file__).parents[1] / "beep_bridge" / "beep_bridge.py"
SPEC = importlib.util.spec_from_file_location("beep_bridge", MODULE_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"could not load bridge module from {MODULE_PATH}")
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


class MarkObjectPlanTests(unittest.TestCase):
    def setUp(self):
        bridge.motion_cancel.clear()

    def tearDown(self):
        bridge.motion_cancel.clear()

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
        self.assertEqual(plan["target_front_m"], 0.25)
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
                "slam": {"active": True, "pose_valid": True},
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
                "slam": {"active": True, "pose_valid": True},
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

    def test_continuous_approach_starts_forward_once_and_stops_at_target(self):
        readings = iter([1.20, 0.80, 0.24])
        commands = []
        stops = []

        def fake_snapshot(*args, **kwargs):
            return {
                "scan_seen": True,
                "scan_age_s": 0.01,
                "sectors": {"front": next(readings)},
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

            result = bridge.forward_continuous_until(target_front=0.25, max_duration=2.0)
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
        })

        self.assertEqual(action, "forward")
        self.assertEqual(duration, 0.8)
        self.assertEqual(reason, "front_clear_stride")

    def test_quaternion_to_yaw_extracts_planar_slam_heading(self):
        yaw = math.pi / 2
        actual = bridge.quaternion_to_yaw(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))

        self.assertAlmostEqual(actual, yaw, places=6)


if __name__ == "__main__":
    unittest.main()