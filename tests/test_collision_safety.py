import math
import unittest
from unittest.mock import patch

from beep_bridge import beep_bridge as bridge


WALL_CONTACT = {
    "front": 0.127,
    "front_left": 0.128,
    "front_right": 0.157,
    "left": 0.192,
    "right": 1.125,
    "rear": 1.038,
}
FRONT_BLOCKED_REAR_CLEAR = dict(WALL_CONTACT, left=0.55, right=0.70, rear=1.038)


class CollisionSafetyTests(unittest.TestCase):
    def setUp(self):
        bridge._reset_motion_state_for_tests()

    def tearDown(self):
        bridge._reset_motion_state_for_tests()

    def test_recorded_wall_contact_stops_instead_of_strafing_or_scraping(self):
        action, duration, reason = bridge.choose_explore_action(WALL_CONTACT)
        self.assertEqual((action, duration), ("stop", 0.0))
        self.assertEqual(reason, "footprint_boxed_in_stop")

    def test_front_obstacle_backs_away_only_with_full_side_and_rear_clearance(self):
        action, duration, reason = bridge.choose_explore_action(FRONT_BLOCKED_REAR_CLEAR)
        self.assertEqual(action, "back")
        self.assertEqual(duration, 0.30)
        self.assertEqual(reason, "front_footprint_breach_backoff")

    def test_every_required_sector_fails_closed_when_missing_or_invalid(self):
        clear = {
            "front": 1.2, "front_left": 0.8, "front_right": 0.8,
            "left": 0.7, "right": 0.7, "rear": 0.7,
        }
        for name in clear:
            with self.subTest(sector=name):
                incomplete = dict(clear)
                incomplete.pop(name)
                self.assertEqual(
                    bridge.choose_explore_action(incomplete),
                    ("stop", 0.0, "lidar_sector_missing_or_invalid"),
                )
        for invalid in (None, 0.0, -1.0, float("nan"), "not-a-range"):
            with self.subTest(invalid=invalid):
                self.assertEqual(bridge.choose_explore_action(dict(clear, front=invalid))[0], "stop")

    def test_boxed_in_state_stops_instead_of_guessing(self):
        sectors = dict(WALL_CONTACT, rear=0.30, right=0.31)
        action, duration, reason = bridge.choose_explore_action(sectors)
        self.assertEqual((action, duration), ("stop", 0.0))
        self.assertEqual(reason, "footprint_boxed_in_stop")

    def test_side_footprint_breach_stops_even_when_lidar_centerline_is_open(self):
        sectors = {
            "front": 1.2, "front_left": 0.8, "front_right": 0.8,
            "left": 0.24, "right": 1.0, "rear": 1.0,
        }
        action, duration, reason = bridge.choose_explore_action(sectors)
        self.assertEqual((action, duration), ("stop", 0.0))
        self.assertEqual(reason, "side_footprint_breach_stop")

    def test_forward_requires_conservative_full_body_corridor(self):
        clear = {
            "front": 1.2, "front_left": 0.8, "front_right": 0.8,
            "left": 0.7, "right": 0.7, "rear": 0.7,
        }
        self.assertEqual(bridge.choose_explore_action(clear)[0], "forward")
        self.assertEqual(bridge.choose_explore_action(dict(clear, front_left=0.39))[0], "back")

    def test_backoff_progress_requires_real_front_clearance_gain(self):
        stalled = dict(FRONT_BLOCKED_REAR_CLEAR, front=0.13, front_left=0.13)
        improved = dict(FRONT_BLOCKED_REAR_CLEAR, front=0.24, front_left=0.24, front_right=0.24)
        self.assertFalse(bridge.escape_made_progress("back", FRONT_BLOCKED_REAR_CLEAR, stalled))
        self.assertTrue(bridge.escape_made_progress("back", FRONT_BLOCKED_REAR_CLEAR, improved))
        self.assertGreaterEqual(bridge.ESCAPE_CLEARANCE_GAIN_M, 0.08)

    def test_turn_progress_requires_heading_change_and_safe_sweep(self):
        sectors = {
            "front": 0.56, "front_left": 0.48, "front_right": 0.52,
            "left": 0.48, "right": 0.72, "rear": 0.60,
        }
        before_pose = {"yaw": 0.0}
        self.assertFalse(bridge.escape_made_progress(
            "turnright", sectors, sectors, before_pose, {"yaw": -0.05}
        ))
        self.assertTrue(bridge.escape_made_progress(
            "turnright", sectors, sectors, before_pose, {"yaw": -math.radians(18)}
        ))
        self.assertFalse(bridge.escape_made_progress(
            "turnright", sectors, dict(sectors, front_left=0.20),
            before_pose, {"yaw": -math.radians(18)}
        ))

    def test_guarded_turn_progress_window_fails_stalled_motion(self):
        self.assertFalse(bridge.turn_window_stalled(0.50, math.radians(0.0)))
        self.assertTrue(bridge.turn_window_stalled(0.75, math.radians(4.0)))
        self.assertFalse(bridge.turn_window_stalled(0.75, math.radians(12.0)))

    def test_legacy_forward_endpoint_delegates_to_six_sector_guard(self):
        guarded = {"ok": False, "reason": "footprint_clearance_rejected_before_start"}
        with patch.object(bridge, "forward_continuous_until", return_value=guarded) as continuous:
            result = bridge.forward_until(target_front=0.10, max_duration=2.0)

        continuous.assert_called_once_with(target_front=0.55, max_duration=2.0, min_target=0.55)
        self.assertEqual(result["controller"], "forward_continuous_until")
        self.assertEqual(result["mode"], "forward_until")
        self.assertFalse(result["ok"])
        self.assertFalse(hasattr(bridge, "_forward_until_legacy_disabled"))

    def test_forward_supervisor_enforces_corridor_threshold_while_moving(self):
        state = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "sectors": {
                "front": 0.60, "front_left": 0.80, "front_right": 0.80,
                "left": 0.70, "right": 0.70, "rear": 0.70,
            },
        }
        with patch.object(bridge, "snapshot", return_value=state):
            result = bridge.supervise_lidar_motion("forward", 1.0)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "forward_footprint_breach")

    def test_continuous_approach_rejects_front_below_autonomous_floor(self):
        state = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "slam": {"active": True, "pose_valid": True, "usable": True,
                     "pose_age_s": 0.01, "map_age_s": 0.01},
            "pose": {"yaw": 0.0},
            "sectors": {
                "front": 0.54, "front_left": 0.80, "front_right": 0.80,
                "left": 0.70, "right": 0.70, "rear": 0.70,
            },
        }
        with patch.object(bridge, "snapshot", return_value=state), \
                patch.object(bridge, "motor_send") as motor:
            result = bridge.forward_continuous_until(target_front=0.25, max_duration=1.0)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "footprint_clearance_rejected_before_start")
        motor.assert_not_called()

    def test_continuous_approach_aborts_if_front_floor_is_crossed(self):
        safe = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "slam": {"active": True, "pose_valid": True, "usable": True,
                     "pose_age_s": 0.01, "map_age_s": 0.01},
            "pose": {"yaw": 0.0},
            "sectors": {
                "front": 0.80, "front_left": 0.80, "front_right": 0.80,
                "left": 0.70, "right": 0.70, "rear": 0.70,
            },
        }
        breached = dict(safe, sectors=dict(safe["sectors"], front=0.54))
        with patch.object(bridge, "snapshot", side_effect=[safe, breached]), \
                patch.object(bridge, "motor_send") as motor, \
                patch.object(bridge, "stop_burst"):
            result = bridge.forward_continuous_until(target_front=0.55, max_duration=1.0)
        self.assertFalse(result["ok"])
        self.assertTrue(result["reason"].startswith("footprint_clearance_breach:front=0.540"))
        motor.assert_called_once_with("forward", step=None)

    def test_backward_supervisor_stops_when_rear_clearance_closes(self):
        state = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "sectors": {
                "front": 0.20, "front_left": 0.22, "front_right": 0.24,
                "left": 0.5, "right": 0.5, "rear": 0.31,
            },
        }
        with patch.object(bridge, "snapshot", return_value=state):
            result = bridge.supervise_lidar_motion("back", 1.0)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "rear_clearance_breach")

    def test_backward_supervisor_stops_before_side_envelope_is_lost(self):
        state = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "sectors": dict(FRONT_BLOCKED_REAR_CLEAR, left=0.34),
        }
        with patch.object(bridge, "snapshot", return_value=state):
            result = bridge.supervise_lidar_motion("back", 1.0)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "side_clearance_breach_during_backoff")

    def test_autonomous_mission_refuses_boxed_in_preflight(self):
        status = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "sectors": dict(WALL_CONTACT, rear=0.30, right=0.31),
            "slam": {"active": True, "pose_valid": True, "usable": True,
                     "pose_age_s": 0.01, "map_age_s": 0.01},
        }
        with patch.object(bridge, "snapshot", return_value=status):
            result = bridge.start_autonomous_mission(mode="local", duration_s=30)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "footprint_clearance_rejected_before_start")
        self.assertEqual(result["safety_reason"], "footprint_boxed_in_stop")

    def test_autonomous_mission_refuses_missing_sector_without_reverse_command(self):
        status = {
            "scan_seen": True,
            "scan_age_s": 0.01,
            "sectors": {
                "front_left": 0.8, "front_right": 0.8,
                "left": 0.7, "right": 0.7, "rear": 0.7,
            },
            "slam": {"active": True, "pose_valid": True, "usable": True,
                     "pose_age_s": 0.01, "map_age_s": 0.01},
        }
        with patch.object(bridge, "snapshot", return_value=status), \
                patch.object(bridge, "motor_send") as motor:
            result = bridge.start_autonomous_mission(mode="local", duration_s=30)
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "footprint_clearance_rejected_before_start")
        self.assertEqual(result["safety_reason"], "lidar_sector_missing_or_invalid")
        motor.assert_not_called()


if __name__ == "__main__":
    unittest.main()
