"""Stopped-only SLAM reset must verify live readiness and bound publisher recovery."""
import ast
import io
import json
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/jupyter_reset_slam.py"


def remote_reset_code():
    tree = ast.parse(SCRIPT.read_text())
    reset = next(node for node in tree.body if isinstance(node, ast.AsyncFunctionDef) and node.name == "reset_slam")
    assignment = next(node for node in reset.body if isinstance(node, ast.Assign)
                      and any(isinstance(target, ast.Name) and target.id == "code" for target in node.targets))
    assert isinstance(assignment.value, ast.Constant) and isinstance(assignment.value.value, str)
    code = assignment.value.value
    compile(code, str(SCRIPT) + ":remote", "exec")
    return code.replace("import subprocess, time\nimport json, urllib.request\n", "", 1)


class ResetSlamReadinessTests(unittest.TestCase):
    def run_remote(self, recover_after_vendor_restart, initially_usable=False):
        commands = []
        state = {"vendor_restarted": False, "scan": 0}

        def run(command, **_kwargs):
            commands.append(tuple(command))
            if command[:3] == ["sudo", "systemctl", "restart"] and command[3] == "YahboomStart":
                state["vendor_restarted"] = True
            if command[:2] == ["systemctl", "is-active"]:
                return SimpleNamespace(returncode=0, stdout="inactive\n" if command[2] == "XGO_Start" else "active\n", stderr="")
            return SimpleNamespace(returncode=0, stdout="", stderr="")

        def urlopen(url, **_kwargs):
            state["scan"] += 1
            status = {"moving": False, "motion_lease_id": None, "scan_count": state["scan"],
                      "scan_age_s": 0.02, "slam": {"usable": initially_usable or
                                                   (recover_after_vendor_restart and state["vendor_restarted"])}}
            payload = {"mission": None, "status": status} if url.endswith("/mission") else status
            return io.StringIO(json.dumps(payload))

        globals_ = {"subprocess": SimpleNamespace(run=run), "time": SimpleNamespace(sleep=lambda _: None),
                    "json": json, "urllib": SimpleNamespace(request=SimpleNamespace(urlopen=urlopen))}
        exec(remote_reset_code(), globals_)
        return commands

    def test_bounded_vendor_recovery_reaches_live_readiness(self):
        commands = self.run_remote(True)
        self.assertIn(("sudo", "systemctl", "restart", "YahboomStart"), commands)
        self.assertEqual(commands.count(("sudo", "systemctl", "restart", "beep-cartographer", "beep-occupancy-grid")), 2)

    def test_healthy_downstream_reset_leaves_vendor_publisher_alone(self):
        commands = self.run_remote(False, initially_usable=True)
        self.assertNotIn(("sudo", "systemctl", "restart", "YahboomStart"), commands)

    def test_unusable_slam_fails_after_one_vendor_recovery(self):
        with self.assertRaisesRegex(RuntimeError, "SLAM remains unusable"):
            self.run_remote(False)


if __name__ == "__main__":
    unittest.main()
