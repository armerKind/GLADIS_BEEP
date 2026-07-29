from pathlib import Path
import re
import unittest


CONFIG = Path(__file__).resolve().parents[1] / "config" / "beep_2d.lua"


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


if __name__ == "__main__":
    unittest.main()
