import ast
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

    def test_reset_preserves_healthy_lidar_owner_and_restarts_downstream_slam(self):
        text = RESET_SCRIPT.read_text()
        stop_duplicate = text.index("'stop', 'XGO_Start'")
        restart_slam = text.index("'restart', 'beep-cartographer', 'beep-occupancy-grid'")
        restart_bridge = text.index("'restart', 'beep-bridge'")
        self.assertLess(stop_duplicate, restart_slam)
        self.assertLess(restart_slam, restart_bridge)
        self.assertNotIn("'restart', 'YahboomStart'", text)
        self.assertIn("'XGO_Start': 'inactive'", text)
        self.assertIn("'YahboomStart': 'active'", text)

    def test_reset_is_stopped_fenced_and_checks_remote_execution(self):
        text = RESET_SCRIPT.read_text()
        tree = ast.parse(text)
        method = next(node for node in tree.body if isinstance(node, ast.AsyncFunctionDef)
                      and node.name == 'reset_slam')
        remote = next(node.value.value for node in ast.walk(method)
                      if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name)
                      and target.id == 'code' for target in node.targets)
                      and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str))
        compile(remote, '<remote-slam-reset>', 'exec')
        self.assertLess(remote.index('require_stopped()'), remote.index("'stop', 'XGO_Start'"))
        self.assertIn('if index > 0:\n        require_stopped()', remote)
        self.assertIn("'is-active','YahboomStart'", remote)
        self.assertIn('msg.get("parent_header", {}).get("msg_id")', text)
        self.assertIn('msg.get("content", {}).get("status") != "ok"', text)
        self.assertIn('session.delete(', text)


if __name__ == "__main__":
    unittest.main()
