import json
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from beep_bridge import beep_bridge as bridge
from scripts import replay_motion_policy


class MotionProfileTests(unittest.TestCase):
    def test_vendor_equivalent_profile_preserves_firmware_gait(self):
        dog = MagicMock()
        with patch.object(bridge, "sdk_init", return_value=dog), \
             patch.object(bridge, "sdk_profile_key", None):
            bridge.sdk_apply_motion_profile(force=True)
        dog.gait_type.assert_not_called()
        dog.pace.assert_called_once_with("normal")
        dog.translation.assert_called_once_with("z", 108)
        dog.attitude.assert_called_once_with("y", 0)
        dog.imu.assert_called_once_with(0)

    def test_stationary_posture_can_be_commissioned_without_motion(self):
        original_moving = bridge.state["moving"]
        try:
            bridge.state["moving"] = False
            with patch.object(bridge, "active_motion_lease", return_value=None), \
                 patch.object(bridge, "SDK_BODY_HEIGHT", 108), \
                 patch.object(bridge, "SDK_SHOULDER_YAW", 0), \
                 patch.object(bridge, "SDK_IMU", 0), \
                 patch.object(bridge, "sdk_apply_motion_profile") as apply:
                result = bridge.configure_stationary_posture(body_height=105, apply=True)
                self.assertEqual(result["profile"]["body_height"], 105)
                apply.assert_called_once_with(force=True)
        finally:
            bridge.state["moving"] = original_moving
            bridge.sdk_mark_profile_dirty()

    def test_posture_change_is_rejected_during_motion(self):
        with patch.object(bridge, "active_motion_lease", return_value=object()):
            with self.assertRaises(bridge.MotionBusy):
                bridge.configure_stationary_posture(body_height=105, apply=False)

    def test_posture_rejects_values_outside_vendor_app_range(self):
        with patch.object(bridge, "active_motion_lease", return_value=None), \
             patch.dict(bridge.state, {"moving": False}):
            with self.assertRaises(ValueError):
                bridge.configure_stationary_posture(body_height=75, apply=False)


class MotionPolicyReplayTests(unittest.TestCase):
    def test_saved_shape_replay_increases_forward_bias_without_changing_hazards(self):
        fixture = Path(__file__).parent / "fixtures" / "motion_policy_trace.json"
        sectors = json.loads(fixture.read_text())
        rows = [{"path": f"fixture:{index}", "sectors": row, "recorded": None}
                for index, row in enumerate(sectors)]
        result = replay_motion_policy.replay(rows)
        self.assertGreater(result["new_forward_fraction"], result["old_forward_fraction"])
        self.assertLess(result["new_turning_fraction"], result["old_turning_fraction"])
        self.assertEqual(result["old_policy"].get("back"), result["new_policy"].get("back"))
        self.assertEqual(result["old_policy"].get("stop"), result["new_policy"].get("stop"))


if __name__ == "__main__":
    unittest.main()
