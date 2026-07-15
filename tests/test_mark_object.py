import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).parents[1] / "beep_bridge" / "beep_bridge.py"
SPEC = importlib.util.spec_from_file_location("beep_bridge", MODULE_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"could not load bridge module from {MODULE_PATH}")
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


class MarkObjectPlanTests(unittest.TestCase):
    def test_default_plan_turns_left_90_degrees_before_right_leg_action(self):
        result = bridge.mark_object(dry_run=True)

        plan = result["plan"]
        self.assertEqual(plan["turn"], "left")
        self.assertEqual(plan["turn_degrees"], 90.0)
        self.assertEqual(plan["marking_side"], "right")
        self.assertEqual(plan["trick"]["name"], "pee")
        self.assertEqual(plan["trick"]["id"], 11)


if __name__ == "__main__":
    unittest.main()