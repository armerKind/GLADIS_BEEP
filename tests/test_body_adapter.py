import unittest
from typing import Any
from unittest.mock import patch

from beep_agent import BodyDispatchError, BridgeBodyAdapter, skill


class BodyAdapterTests(unittest.TestCase):
    def test_bias_mission_sends_only_bounded_exact_owner_advice(self):
        adapter = BridgeBodyAdapter("http://beep.invalid")
        accepted = {"ok": True, "accepted": True, "motor_authority": False,
                    "semantic_execution_enabled": False, "mission_id": "mission-7", "lease_id": "lease-7"}
        with patch.object(adapter, "_json", return_value=accepted) as request:
            response = adapter.bias_mission(boot_id="boot-7", mission_id="mission-7",
                                            lease_id="lease-7", preferred_direction="left")
        self.assertTrue(response["accepted"])
        request.assert_called_once_with("/mission/bias", {
            "boot_id": "boot-7", "mission_id": "mission-7", "lease_id": "lease-7",
            "ttl_s": 4.0, "preferred_direction": "left", "preferred_bearing_rad": None})
        for field, invalid in (("boot_id", ""), ("lease_id", None),
                               ("preferred_direction", "back"), ("ttl_s", 8.01),
                               ("ttl_s", float("nan"))):
            values: dict[str, Any] = dict(boot_id="boot-7", mission_id="mission-7", lease_id="lease-7",
                                          preferred_direction="right", ttl_s=4.0)
            values[field] = invalid
            with patch.object(adapter, "_json") as request:
                with self.assertRaises(BodyDispatchError):
                    adapter.bias_mission(**values)
                request.assert_not_called()

    def test_bias_mission_rejects_replaced_or_motor_authorizing_response(self):
        adapter = BridgeBodyAdapter("http://beep.invalid")
        values = dict(boot_id="boot-7", mission_id="mission-7", lease_id="lease-7",
                      preferred_direction="right")
        for response in ({"ok": False, "http_status": 409},
                         {"ok": True, "accepted": True, "motor_authority": True,
                          "semantic_execution_enabled": False, "mission_id": "mission-7", "lease_id": "lease-7"},
                         {"ok": True, "accepted": True, "motor_authority": False,
                          "semantic_execution_enabled": False, "mission_id": "other", "lease_id": "lease-7"}):
            with patch.object(adapter, "_json", return_value=response):
                with self.assertRaises(BodyDispatchError):
                    adapter.bias_mission(**values)

    def test_shadow_explore_never_contacts_bridge(self):
        adapter = BridgeBodyAdapter("http://beep.invalid")
        with patch.object(adapter, "_json") as request:
            result = adapter.dispatch(skill("explore", duration_s=90, reason="Keep moving while planning."), rollout_mode="shadow")
        self.assertTrue(result.ok)
        self.assertFalse(result.dispatched)
        request.assert_not_called()

    def test_supervised_explore_starts_async_coverage_mission(self):
        adapter = BridgeBodyAdapter("http://beep.invalid")
        accepted = {"ok": True, "accepted": True, "mission": {"id": "mission-7", "state": "running"}}
        with patch.object(adapter, "_json", return_value=accepted) as request:
            result = adapter.dispatch(skill("explore", duration_s=190, reason="Navigate while reasoning."), rollout_mode="supervised")

        self.assertTrue(result.ok)
        self.assertTrue(result.dispatched)
        self.assertTrue(result.asynchronous)
        request.assert_called_once_with("/mission/start", {
            "mode": "coverage", "duration_s": 190.0, "min_duration": 190.0, "save": False,
        })

    def test_stationary_observe_uses_non_motion_bridge_endpoint(self):
        adapter = BridgeBodyAdapter("http://beep.invalid")
        with patch.object(adapter, "_json", return_value={"status": {"moving": False}}) as request:
            result = adapter.dispatch(skill("observe", reason="Refresh world evidence."), rollout_mode="stationary")
        self.assertTrue(result.ok)
        self.assertTrue(result.dispatched)
        request.assert_called_once_with("/observe")

    def test_stop_cancels_async_mission(self):
        adapter = BridgeBodyAdapter("http://beep.invalid")
        with patch.object(adapter, "_json", return_value={"ok": True, "cancelled": True}) as request:
            result = adapter.dispatch(skill("stop", reason="Human stop."), rollout_mode="supervised",
                                      mission_id="mission-7", lease_id="lease-7")
        self.assertTrue(result.ok)
        self.assertFalse(result.asynchronous)
        request.assert_called_once_with("/mission/cancel", {"mission_id": "mission-7", "lease_id": "lease-7"})

    def test_unimplemented_physical_skill_is_rejected(self):
        adapter = BridgeBodyAdapter("http://beep.invalid")
        call = skill("orient", target_id="object-1", direction="left", degrees=10, reason="Inspect.")
        with self.assertRaisesRegex(BodyDispatchError, "no deterministic body adapter"):
            adapter.dispatch(call, rollout_mode="supervised")

    def test_explore_remains_bounded(self):
        self.assertEqual(skill("explore", duration_s=600, reason="Maximum bounded mission.").arguments["duration_s"], 600)
        with self.assertRaisesRegex(ValueError, "duration_s"):
            skill("explore", duration_s=601, reason="Too long.")


if __name__ == "__main__":
    unittest.main()
