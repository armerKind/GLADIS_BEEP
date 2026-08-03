from pathlib import Path
import re
import unittest


CONFIG = Path(__file__).resolve().parents[1] / "config" / "beep_2d.lua"
RESET_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "jupyter_reset_slam.py"


def lua_string(text: str, key: str) -> str | None:
    match = re.search(rf"^\s*{re.escape(key)}\s*=\s*\"([^\"]+)\"\s*,", text, re.MULTILINE)
    return match.group(1) if match else None


class SlamConfigTests(unittest.TestCase):
    def test_cartographer_publishes_robot_model_root(self):
        text = CONFIG.read_text()
        self.assertEqual(lua_string(text, "tracking_frame"), "base_link")
        self.assertEqual(lua_string(text, "published_frame"), "base_footprint")
        self.assertEqual(lua_string(text, "odom_frame"), "odom")
        self.assertIn("provide_odom_frame = true", text)

    def test_cartographer_does_not_claim_base_link_as_published_child(self):
        text = CONFIG.read_text()
        self.assertNotEqual(lua_string(text, "published_frame"), "base_link")

    def test_reset_restarts_single_lidar_owner_before_slam(self):
        text = RESET_SCRIPT.read_text()
        stop_duplicate = text.index("'stop', 'XGO_Start'")
        restart_lidar = text.index("'restart', 'YahboomStart'")
        restart_slam = text.index("'restart', 'beep-cartographer', 'beep-occupancy-grid'")
        restart_bridge = text.index("'restart', 'beep-bridge'")
        self.assertLess(stop_duplicate, restart_lidar)
        self.assertLess(restart_lidar, restart_slam)
        self.assertLess(restart_slam, restart_bridge)
        self.assertIn("'XGO_Start': 'inactive'", text)


if __name__ == "__main__":
    unittest.main()
