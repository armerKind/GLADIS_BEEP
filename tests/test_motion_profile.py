import json
import sys
import threading
import types
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

    def test_posture_boolean_parsing_is_strict(self):
        with patch.object(bridge, "active_motion_lease", return_value=None), \
             patch.dict(bridge.state, {"moving": False}), \
             patch.object(bridge, "sdk_apply_motion_profile") as apply:
            result = bridge.configure_stationary_posture(imu="false", apply="false")
            self.assertEqual(result["profile"]["imu"], 0)
            apply.assert_not_called()
            with self.assertRaisesRegex(ValueError, "imu must be a boolean"):
                bridge.configure_stationary_posture(imu="perhaps", apply=False)
            with self.assertRaisesRegex(ValueError, "apply must be a boolean"):
                bridge.configure_stationary_posture(apply="sometimes")

    def test_failed_posture_apply_restores_software_profile_and_reapplies_previous_values(self):
        attempts = []

        def apply(force=False):
            attempts.append((bridge.SDK_BODY_HEIGHT, bridge.SDK_SHOULDER_YAW, bridge.SDK_IMU, force))
            if len(attempts) == 1:
                raise OSError("serial write failed")

        with patch.object(bridge, "active_motion_lease", return_value=None), \
             patch.dict(bridge.state, {"moving": False}), \
             patch.object(bridge, "SDK_BODY_HEIGHT", 108), \
             patch.object(bridge, "SDK_SHOULDER_YAW", 0), \
             patch.object(bridge, "SDK_IMU", 0), \
             patch.object(bridge, "sdk_apply_motion_profile", side_effect=apply):
            with self.assertRaisesRegex(RuntimeError, "posture apply failed"):
                bridge.configure_stationary_posture(body_height=105, shoulder_yaw=2, imu=True)
            self.assertEqual((bridge.SDK_BODY_HEIGHT, bridge.SDK_SHOULDER_YAW, bridge.SDK_IMU), (108, 0, 0))
        self.assertEqual(attempts, [(105, 2, 1, True), (108, 0, 0, True)])

    def test_sdk_initialization_is_singleton_under_concurrent_first_use(self):
        created = []
        dog = object()
        module = types.SimpleNamespace(DOGZILLA=lambda: created.append(dog) or dog)
        barrier = threading.Barrier(3)
        results = []

        def initialize():
            barrier.wait()
            results.append(bridge.sdk_init())

        with patch.dict(sys.modules, {"DOGZILLALib": module}), \
             patch.object(bridge, "sdk_dog", None), \
             patch.object(bridge, "sdk_apply_motion_profile"):
            threads = [threading.Thread(target=initialize) for _ in range(2)]
            for thread in threads:
                thread.start()
            barrier.wait()
            for thread in threads:
                thread.join(1.0)
        self.assertEqual(created, [dog])
        self.assertEqual(results, [dog, dog])

    def test_systemd_and_fallback_deployment_use_same_vendor_profile(self):
        root = Path(__file__).resolve().parents[1]
        service = (root / "systemd" / "beep-bridge.service").read_text()
        deploy = (root / "scripts" / "jupyter_deploy_bridge.py").read_text()
        for expected in ("BEEP_MOTOR_BACKEND", "BEEP_SDK_GAIT", "BEEP_SDK_PACE",
                         "BEEP_SDK_BODY_HEIGHT", "BEEP_SDK_SHOULDER_YAW", "BEEP_SDK_IMU"):
            self.assertIn(expected, service)
            self.assertIn(expected, deploy)
        self.assertIn("Environment=BEEP_MOTOR_BACKEND=sdk", service)
        self.assertIn("'BEEP_MOTOR_BACKEND': 'sdk'", deploy)
        self.assertIn("Environment=BEEP_SDK_GAIT=firmware", service)
        self.assertIn("'BEEP_SDK_GAIT': 'firmware'", deploy)
        self.assertIn("beep-bridge.service", deploy)
        self.assertIn("systemctl','daemon-reload", deploy)
        self.assertIn("systemctl','restart','beep-bridge", deploy)
        self.assertIn("127.0.0.1:8766/config", deploy)


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

    def test_replay_resets_fluent_state_between_capture_runs(self):
        curve = {"front": 1.0, "front_left": 1.2, "front_right": 0.6,
                 "left": 1.2, "right": 0.6, "rear": 1.0}
        straight = {"front": 1.0, "front_left": 1.0, "front_right": 1.0,
                    "left": 1.0, "right": 1.0, "rear": 1.0}
        rows = [
            {"path": "run-a/1", "run_id": "run-a", "sectors": curve, "recorded": None},
            {"path": "run-b/1", "run_id": "run-b", "sectors": straight, "recorded": None},
        ]
        result = replay_motion_policy.replay(rows)
        self.assertEqual(result["runs"], 2)
        self.assertEqual(result["old_policy"].get("forward"), 1)

    def test_fluent_steering_is_not_consulted_when_primary_policy_rejects_forward(self):
        fixture = Path(__file__).parent / "fixtures" / "motion_policy_trace.json"
        rows = [{"path": f"hazard:{index}", "sectors": sectors, "recorded": None}
                for index, sectors in enumerate(json.loads(fixture.read_text()))
                if bridge.choose_explore_action(sectors)[0] != "forward"]
        with patch.object(bridge, "choose_fluent_steering", side_effect=AssertionError("unsafe steering call")):
            result = replay_motion_policy.replay(rows)
        self.assertEqual(result["observations"], len(rows))


if __name__ == "__main__":
    unittest.main()
