import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "patch_vendor_app.py"
SPEC = importlib.util.spec_from_file_location("patch_vendor_app", MODULE_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"cannot load {MODULE_PATH}")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

NEW_ACTION = MODULE.NEW_ACTION
NEW_IP_LOOKUP = MODULE.NEW_IP_LOOKUP
NEW_TCP = MODULE.NEW_TCP
OLD_ACTION = MODULE.OLD_ACTION
OLD_IP_LOOKUP = MODULE.OLD_IP_LOOKUP
OLD_TCP = MODULE.OLD_TCP
apply_patch = MODULE.apply_patch


class VendorPatchTests(unittest.TestCase):
    def sample(self) -> str:
        return "\n".join((OLD_IP_LOOKUP, OLD_TCP, OLD_ACTION, ""))

    def test_applies_all_compatibility_changes(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "app_dogzilla.py"
            path.write_text(self.sample())

            self.assertTrue(apply_patch(path))

            patched = path.read_text()
            self.assertIn(NEW_IP_LOOKUP, patched)
            self.assertIn(NEW_TCP, patched)
            self.assertIn(NEW_ACTION, patched)
            self.assertTrue(path.with_suffix(".py.gladis-original").exists())

    def test_is_idempotent(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "app_dogzilla.py"
            path.write_text(self.sample())

            self.assertTrue(apply_patch(path))
            first = path.read_text()
            self.assertFalse(apply_patch(path))
            self.assertEqual(first, path.read_text())

    def test_rejects_unknown_source_shape(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "app_dogzilla.py"
            path.write_text("print('different vendor release')\n")

            with self.assertRaisesRegex(RuntimeError, "IP-discovery"):
                apply_patch(path)


if __name__ == "__main__":
    unittest.main()
