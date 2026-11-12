"""A bad visual-model reply must not interrupt BEEP's onboard navigation."""
import argparse
import io
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from PIL import Image

from beep_eyes.contact_sheet import CapturedFrame
from beep_eyes.gemini import GeminiError
from beep_eyes.moving_window import MovingFrameSample
from scripts import run_moving_eyes


class InferenceFaultTests(unittest.TestCase):
    def test_model_error_is_recorded_and_next_window_still_runs(self):
        buf = io.BytesIO()
        Image.new("RGB", (160, 120), "green").save(buf, format="JPEG")
        jpeg = buf.getvalue()
        lease = "lease-1"
        now = time.time()
        status = {"moving": True, "motion_lease_id": lease, "scan_age_s": 0.02,
                  "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                  "sectors": {"front": 2.0, "front_left": 1.0, "front_right": 1.0,
                              "left": 1.0, "right": 1.0, "rear": 1.0},
                  "slam": {"usable": True, "pose_valid": True}}
        mission = {"id": "mission-1", "lease_id": lease, "state": "running"}
        sample = MovingFrameSample(CapturedFrame("f000", now, jpeg), status)

        class Capture:
            def __init__(self):
                self.ring = self
                self.running = False
                self.windows = 0
            def start(self):
                self.running = True
            def stop(self):
                self.running = False
            def wait_for(self, count, timeout):
                return self.running
            def latest(self, count):
                return [sample]
            def temporal_window(self, count, *, slam_map=None):
                self.windows += 1
                return [sample.frame], jpeg, {
                    "frames": [sample.metadata()], "panel_count": 1,
                    "distinct_appearance_count": 1, "candidate_frame_count": 1,
                    "bridge_status": status, "navigation_context": {},
                }
            def stats(self):
                return {"captured": self.windows, "errors": 0}

        capture = Capture()
        response = json.loads((Path(__file__).parent / "fixtures" / "eyes_response.json").read_text())
        client = Mock()
        def analyze(*_):
            self.assertTrue(capture.running)
            if client.analyze.call_count == 1:
                raise GeminiError("model output violates contract")
            return response
        client.analyze.side_effect = analyze
        with tempfile.TemporaryDirectory() as directory:
            args = argparse.Namespace(base_url="http://fake", panel=4, fps=3.0,
                                      duration=5.0, timeout=1.0,
                                      inference_interval=0.0, model=None,
                                      credential_source="auto", hermes_home=None,
                                      mock_response=None, allow_no_mission=False,
                                      max_inferences=2, output_root=Path(directory),
                                      goal=None, target="green baseball cap", mark=False,
                                      resume_session=None)
            def get_json(base_url, path, timeout):
                if path == "/mission":
                    return {"mission": mission}
                if path == "/status":
                    return status
                raise AssertionError(path)
            with patch.object(run_moving_eyes, "parse_args", return_value=args), \
                 patch.object(run_moving_eyes, "ContinuousMovingCapture", return_value=capture), \
                 patch.object(run_moving_eyes, "gemini_client", return_value=client), \
                 patch.object(run_moving_eyes, "fetch_json", side_effect=get_json), \
                 patch.object(run_moving_eyes, "fetch_slam_map", return_value={"ok": False}):
                self.assertEqual(run_moving_eyes.main(), 0)
            run_dir, = Path(directory).iterdir()
            summary = json.loads((run_dir / "summary.json").read_text())
            events = [json.loads(line) for line in (run_dir / "trace.jsonl").read_text().splitlines()]
            self.assertEqual(summary["inferences"], 2)
            self.assertEqual(summary["inference_errors"], 1)
            self.assertFalse(summary["dispatch_performed"])
            self.assertIn("model output violates contract", events[0]["model_error"])
            self.assertFalse(events[0]["dispatch_performed"])
            self.assertIn("scene", events[1])
            self.assertFalse(capture.running)
            self.assertEqual(mission["state"], "running")


if __name__ == "__main__":
    unittest.main()
