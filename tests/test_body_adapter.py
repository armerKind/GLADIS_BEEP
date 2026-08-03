import unittest
from unittest.mock import patch

from beep_agent import BodyDispatchError, BridgeBodyAdapter, skill


class BodyAdapterTests(unittest.TestCase):
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
            result = adapter.dispatch(skill("stop", reason="Human stop."), rollout_mode="supervised")
        self.assertTrue(result.ok)
        self.assertFalse(result.asynchronous)
        request.assert_called_once_with("/mission/cancel", {})

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
