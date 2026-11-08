"""Synthetic discrepancy contracts; not fall detection or a motor policy."""
import unittest
from typing import Any

from beep_bridge.progress_monitor import MotionDiscrepancy, MotionWindow


def window(**change: Any) -> MotionWindow:
    data: dict[str, Any] = dict(boot_id="boot-1", map_epoch=2, map_revision=3,
                map_revision_verified=True, gait="straight",
                gait_cycles=4, started_at_robot=1.0, ended_at_robot=4.0,
                start_x_m=0.0, start_y_m=0.0, end_x_m=.4, end_y_m=0.0,
                start_yaw_rad=0.0, pose_healthy=True, forward_range_reduction_m=.3,
                visual_motion="moving", attitude_alert=None)
    data.update(change)
    return MotionWindow(**data)


class DiscrepancyTests(unittest.TestCase):
    def test_forward_range_reduction_is_start_minus_end(self):
        # A front range shrinking from 1.2 m to .9 m is +.3 m progress.
        monitor = MotionDiscrepancy(min_gait_cycles=3)
        self.assertEqual(monitor.observe(window(
            forward_range_reduction_m=.3, visual_motion="still")).state, "progress_ok")
        self.assertEqual(monitor.observe(window(
            forward_range_reduction_m=-.3, visual_motion="still")).state,
            "insufficient_evidence")

    def test_correlated_progress_and_two_confirmed_stall_windows(self):
        monitor = MotionDiscrepancy(min_gait_cycles=3)
        self.assertEqual(monitor.observe(window()).state, "progress_ok")
        stalled = window(end_x_m=.01, forward_range_reduction_m=.0, visual_motion="still")
        self.assertEqual(monitor.observe(stalled).state, "insufficient_evidence")
        second = window(started_at_robot=4, ended_at_robot=7)
        second = MotionWindow(**{**vars(second), "end_x_m": .01,
                                 "forward_range_reduction_m": 0.0, "visual_motion": "still"})
        result = monitor.observe(second)
        self.assertEqual(result.state, "stalled")
        self.assertFalse(result.motor_authority)
        self.assertIn("same_epoch", result.signals)

    def test_short_gait_invalid_pose_or_turn_cannot_be_called_stalled(self):
        for sample in (window(gait_cycles=1, end_x_m=0, visual_motion="still", forward_range_reduction_m=0),
                       window(pose_healthy=False), window(gait="turn", end_x_m=0,
                              visual_motion="still", forward_range_reduction_m=0),
                       window(forward_range_reduction_m=None, end_x_m=0, visual_motion="unknown")):
            with self.subTest(sample=sample):
                self.assertEqual(MotionDiscrepancy(min_gait_cycles=3).observe(sample).state,
                                 "insufficient_evidence")

    def test_same_epoch_revision_change_resets_stall_confirmation(self):
        monitor = MotionDiscrepancy(min_gait_cycles=3)
        stalled = window(end_x_m=.01, forward_range_reduction_m=0, visual_motion="still")
        self.assertEqual(monitor.observe(stalled).state, "insufficient_evidence")
        self.assertEqual(monitor.observe(window(map_revision=4, end_x_m=.01,
            forward_range_reduction_m=0, visual_motion="still")).state, "insufficient_evidence")
        self.assertEqual(monitor.observe(window(boot_id="new-boot", end_x_m=.01,
            forward_range_reduction_m=0, visual_motion="still")).state, "insufficient_evidence")

    def test_unverified_map_revision_never_reports_progress_or_stall(self):
        monitor = MotionDiscrepancy(min_gait_cycles=3)
        result = monitor.observe(window(map_revision_verified=False))
        self.assertEqual(result.state, "insufficient_evidence")
        self.assertIn("unverified_map_revision", result.signals)

    def test_pose_vision_conflict_and_external_attitude_are_not_battery_claims(self):
        monitor = MotionDiscrepancy(min_gait_cycles=3)
        conflict = monitor.observe(window(end_x_m=1.2, forward_range_reduction_m=0,
                                          visual_motion="still"))
        self.assertEqual(conflict.state, "localization_conflict")
        self.assertFalse(hasattr(conflict, "battery_state"))
        suspicion = monitor.observe(window(attitude_alert=True))
        self.assertEqual(suspicion.state, "possible_fall")
        self.assertFalse(suspicion.motor_authority)


if __name__ == "__main__":
    unittest.main()
