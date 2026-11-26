"""A quiet camera must not replay the last durable observation."""
import argparse
import io
import json
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from PIL import Image

from beep_eyes.contact_sheet import CapturedFrame
from beep_eyes.moving_window import MovingFrameRing, MovingFrameSample
from scripts import run_moving_eyes


class RepeatedCameraWindowTests(unittest.TestCase):
    def test_ring_wait_requires_an_acquisition_with_a_new_id(self):
        ring = MovingFrameRing()
        first = MovingFrameSample(CapturedFrame('f000', 1.0, b'frame'), {})
        ring.add(first)
        self.assertFalse(ring.wait_for_newer('f000', timeout_s=0))
        ring.add(MovingFrameSample(CapturedFrame('f001', 2.0, b'frame'), {}))
        self.assertTrue(ring.wait_for_newer('f000', timeout_s=0))

    def test_same_acquisition_is_skipped_then_next_frame_is_observed(self):
        buf = io.BytesIO()
        Image.new("RGB", (160, 120), "green").save(buf, format="JPEG")
        jpeg = buf.getvalue()
        now = time.time()
        robot_now = time.monotonic()
        provenance = {"map_epoch": 0, "map_revision": 0,
                      "map_revision_status": "verified", "map_snapshot_id": "grid-1"}
        status = {"moving": True, "motion_lease_id": "lease-1", "scan_age_s": 0.02,
                  "boot_id": "boot-live", "robot_monotonic_s": robot_now,
                  "capture_before": {"boot_id": "boot-live", "robot_monotonic_s": robot_now,
                                     "motion_lease_id": "lease-1", "slam": provenance},
                  "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                  "sectors": {"front": 2.0, "front_left": 1.0, "front_right": 1.0,
                              "left": 1.0, "right": 1.0, "rear": 1.0},
                  "slam": {"usable": True, "pose_valid": True, **provenance}}
        mission = {"id": "mission-1", "lease_id": "lease-1", "state": "running"}
        first = MovingFrameSample(CapturedFrame("f000", now, jpeg), status)
        second = MovingFrameSample(CapturedFrame("f001", now + .1, jpeg), status)

        class Capture:
            def __init__(self):
                self.ring = self
                self.sample = first
                self.running = False
                self.windows = 0
                self.waits = 0
            def start(self):
                self.running = True
            def stop(self):
                self.running = False
            def wait_for(self, count, timeout):
                return self.running
            def wait_for_newer(self, frame_id, timeout):
                assert frame_id == "f000" and self.sample is first
                assert self.windows == 1  # Two iterations; only one submitted window.
                self.waits += 1
                status["robot_monotonic_s"] += 0.2
                status["capture_before"]["robot_monotonic_s"] = status["robot_monotonic_s"]
                self.sample = second
                return True
            def latest(self, count):
                return [self.sample]
            def temporal_window(self, count, *, slam_map=None):
                self.windows += 1
                return [self.sample.frame], jpeg, {
                    "frames": [self.sample.metadata()], "panel_count": 1,
                    "distinct_appearance_count": 1, "candidate_frame_count": 1,
                    "bridge_status": status, "navigation_context": {},
                }
            def stats(self):
                return {"captured": self.windows, "errors": 0}

        capture = Capture()
        fixture = Path(__file__).parent / "fixtures" / "eyes_response.json"
        args = argparse.Namespace(base_url="http://fake", panel=4, fps=3.0,
                                  duration=5.0, timeout=1.0, inference_interval=0.0,
                                  model=None, credential_source="auto", hermes_home=None,
                                  mock_response=fixture, allow_no_mission=False,
                                  max_inferences=2, goal=None, target="scene", mark=False,
                                  resume_session=None)
        def get_json(base_url, path, timeout):
            if path == "/mission":
                return {"mission": mission}
            if path == "/status":
                return status
            raise AssertionError(path)

        with tempfile.TemporaryDirectory() as directory:
            args.output_root = Path(directory)
            with (patch.object(run_moving_eyes, "parse_args", return_value=args),
                  patch.object(run_moving_eyes, "ContinuousMovingCapture", return_value=capture),
                  patch.object(run_moving_eyes, "fetch_json", side_effect=get_json),
                  patch.object(run_moving_eyes, "fetch_slam_map", return_value={
                      "ok": True, "boot_id": "boot-live", **provenance})):
                self.assertEqual(run_moving_eyes.main(), 0)
            run_dir, = Path(directory).iterdir()
            summary = json.loads((run_dir / "summary.json").read_text())
            events = [json.loads(line) for line in (run_dir / "trace.jsonl").read_text().splitlines()]
            self.assertEqual(summary["inferences"], 2)
            self.assertEqual(summary["repeated_window_skips"], 1)
            self.assertEqual(capture.waits, 1)
            self.assertEqual(capture.windows, 2)
            self.assertEqual([event["latest_frame_id"] for event in events], ["f000", "f001"])
            self.assertTrue(all("scene" in event for event in events), events)
            journal = [json.loads(line) for line in (run_dir / "event_journal.jsonl").read_text().splitlines()]
            frame_ids = [item["event_id"] for item in journal if item["event_id"].startswith("frame:")]
            self.assertEqual(len(frame_ids), 2)
            self.assertEqual(len(set(frame_ids)), 2)
            self.assertFalse(capture.running)


if __name__ == "__main__":
    unittest.main()
