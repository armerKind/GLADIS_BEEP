"""Keep the optional vendor camera deployment path safe and syntactically valid."""
import ast
from pathlib import Path
import unittest
from unittest.mock import MagicMock, patch

from scripts import jupyter_deploy_bridge as deploy

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'jupyter_deploy_bridge.py'


def remote_code() -> str:
    tree = ast.parse(SCRIPT.read_text())
    method = next(node for node in tree.body if isinstance(node, ast.AsyncFunctionDef)
                  and node.name == 'restart_bridge')
    return next(node.value.value for node in ast.walk(method)
                if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name)
                and target.id == 'code' for target in node.targets)
                and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str))


class CameraDeployTests(unittest.TestCase):
    def test_uploads_vendor_patcher_only_when_explicitly_enabled(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled), patch.object(deploy, 'PATCH_VENDOR_CAMERA', enabled), \
                 patch.object(deploy, 'VENDOR_PATCH_SRC', SCRIPT.parent / 'patch_vendor_app.py'):
                session = MagicMock()
                session.get.return_value.status_code = 404
                deploy.put_bridge(session, 'xsrf', 'bridge source')
                uploads = [call.args[0] for call in session.put.call_args_list]
                patch_path = deploy.BASE + '/api/contents/beep_bridge/patch_vendor_app.py'
                self.assertEqual(patch_path in uploads, enabled)
                if enabled:
                    uploaded = next(call.kwargs['json']['content'] for call in
                        session.put.call_args_list if call.args[0] == patch_path)
                    self.assertIn('NEW_CAMERA_DISCONNECT', uploaded)

    def test_remote_code_compiles_in_both_opt_in_modes(self):
        code = remote_code()
        for flag in ('True', 'False'):
            with self.subTest(flag=flag):
                rendered = code.replace('__EXPECTED_VERSION__', '"0.22.2-stack2-greeting-optional"')
                rendered = rendered.replace('__APPLY_VENDOR_CAMERA_PATCH__', flag)
                compile(rendered, '<remote-jupyter-deploy>', 'exec')

    def test_camera_patch_path_is_opt_in_and_motion_fenced(self):
        source = SCRIPT.read_text()
        remote = remote_code()
        self.assertIn('os.environ.get("BEEP_PATCH_VENDOR_CAMERA", "0") == "1"', source)
        self.assertIn('beep_bridge/patch_vendor_app.py', source)
        gate = remote.index('refusing vendor camera patch/restart during motion or active lease')
        patch = remote.index("base/'patch_vendor_app.py'")
        restart = remote.index("'restart','beep-vendor-app'")
        self.assertLess(gate, patch)
        self.assertLess(patch, restart)
        self.assertIn('DOGZILLA_SKIP_STARTUP_ACTION=1', remote)


if __name__ == '__main__':
    unittest.main()
