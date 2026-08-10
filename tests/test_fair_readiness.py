import json
from pathlib import Path
import subprocess
import sys
import unittest

from scripts.fair_readiness import preparation_status_ready, stopped_and_unleased


ROOT = Path(__file__).resolve().parents[1]
PREPARE_SCRIPT = ROOT / "scripts" / "prepare_fair_run.sh"


def ready_status() -> dict:
    return {
        "moving": False,
        "motion_lease_id": None,
        "scan_age_s": 0.1,
        "slam": {
            "usable": True,
            "map_age_s": 0.4,
            "pose_age_s": 0.1,
        },
    }


class FairReadinessTests(unittest.TestCase):
    def test_healthy_stopped_bridge_does_not_require_reset(self):
        self.assertTrue(preparation_status_ready(ready_status()))

    def test_motion_or_lease_requires_reset_path(self):
        moving = ready_status()
        moving["moving"] = True
        leased = ready_status()
        leased["motion_lease_id"] = "motion-7"
        self.assertFalse(preparation_status_ready(moving))
        self.assertFalse(preparation_status_ready(leased))
        self.assertFalse(stopped_and_unleased(moving))
        self.assertFalse(stopped_and_unleased(leased))

    def test_stale_or_unusable_slam_requires_reset_path(self):
        stale_scan = ready_status()
        stale_scan["scan_age_s"] = 0.6
        stale_map = ready_status()
        stale_map["slam"]["map_age_s"] = 2.6
        stale_pose = ready_status()
        stale_pose["slam"]["pose_age_s"] = 0.6
        unusable = ready_status()
        unusable["slam"]["usable"] = False
        for status in (stale_scan, stale_map, stale_pose, unusable):
            self.assertFalse(preparation_status_ready(status))

    def test_cli_accepts_healthy_status_fixture_without_network(self):
        fixture = ready_status()
        result = subprocess.run(
            [
                sys.executable,
                str(ROOT / "scripts" / "fair_readiness.py"),
                "--status-json",
                json.dumps(fixture),
            ],
            cwd=ROOT,
            text=True,
            capture_output=True,
            timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertTrue(json.loads(result.stdout)["ready"])

    def test_prepare_script_probes_before_resetting_slam(self):
        text = PREPARE_SCRIPT.read_text()
        health_probe = text.index("scripts/fair_readiness.py")
        reset = text.index("scripts/jupyter_reset_slam.py")
        self.assertLess(health_probe, reset)
        self.assertIn("SLAM already healthy; preserving the active stack", text)
        self.assertIn('BEEP_HOST="${BEEP_HOST:-${RESET_HOST}}"', text)
        self.assertIn("Preparation stop unconfirmed", text)
        self.assertIn("stopped_and_unleased", text)


if __name__ == "__main__":
    unittest.main()
