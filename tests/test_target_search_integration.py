"""Offline target-stop integration: observer signals, exact-owner launcher cancels."""
import argparse
import io
import json
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from PIL import Image

from beep_eyes.contact_sheet import CapturedFrame
from scripts import run_moving_eyes
from scripts.run_beep_explore import run
from test_concurrent_explore import FakeBridge, options

FIXTURES = Path(__file__).parent / "fixtures"


class TargetSearchIntegrationTests(unittest.TestCase):
    def test_observer_confirms_recorded_cap_and_never_sends_stop(self):
        packets = [json.loads((FIXTURES / f"cap_confirmation_window{i}.json").read_text()) for i in (0, 1)]
        # Replay historical sensor content on a fresh simulated clock: a live
        # inference must reject genuinely stale frames.
        base_time = time.time() - 6
        for index, value in enumerate(packets):
            value["packet"]["frames"][-1]["captured_at"] = base_time + index * 4
        lease = packets[0]["packet"]["bridge_status"]["motion_lease_id"]
        mission = {"id": "recorded-run", "lease_id": lease, "state": "running", "intent_outcomes": []}
        status = {"moving": True, "motion_lease_id": lease, "scan_age_s": 0.03,
                  "slam": {"usable": True, "pose_valid": True},
                  "pose": packets[0]["packet"]["bridge_status"]["pose"],
                  "sectors": {"front": 2.0, "front_left": 1.0, "front_right": 1.0,
                              "left": 1.0, "right": 1.0, "rear": 1.0}}
        jpeg = io.BytesIO()
        Image.new("RGB", (160, 120), "gray").save(jpeg, format="JPEG")
        frames = [CapturedFrame(p["packet"]["frames"][-1]["frame_id"],
                                p["packet"]["frames"][-1]["captured_at"], jpeg.getvalue())
                  for p in packets]

        class Capture:
            def __init__(self):
                self.ring = self
                self.running = False
                self.count = 0
            def start(self):
                self.running = True
            def stop(self):
                self.running = False
            def wait_for(self, count, timeout):
                return self.running
            def latest(self, count):
                return [type("Sample", (), {"frame": frames[max(0, self.count-1)]})()]
            def temporal_window(self, count, *, slam_map=None):
                index = self.count
                self.count += 1
                return [frames[index]], jpeg.getvalue(), dict(packets[index]["packet"],
                    distinct_appearance_count=1, candidate_frame_count=1)
            def stats(self):
                return {"captured": self.count, "errors": 0}

        capture = Capture()
        client = Mock()
        def analyze(*_):
            self.assertTrue(capture.running)
            return packets[client.analyze.call_count-1]["response"]
        client.analyze.side_effect = analyze
        def get_json(base_url, path, timeout):
            if path == "/mission":
                return {"mission": mission}
            if path == "/status":
                return status
            raise AssertionError(path)
        with tempfile.TemporaryDirectory() as directory:
            args = argparse.Namespace(base_url="http://fake", panel=4, fps=3.0,
                duration=5.0, timeout=1.0, inference_interval=0.0, model=None,
                credential_source="auto", hermes_home=None, mock_response=None,
                allow_no_mission=False, max_inferences=0, output_root=Path(directory),
                goal="find", target="green baseball cap", mark=False, resume_session=None,
                mode="shadow", arm=False, end_on_confirmed_target=True)
            with patch.object(run_moving_eyes, "parse_args", return_value=args), \
                 patch.object(run_moving_eyes, "ContinuousMovingCapture", return_value=capture), \
                 patch.object(run_moving_eyes, "gemini_client", return_value=client), \
                 patch.object(run_moving_eyes, "fetch_json", side_effect=get_json), \
                 patch.object(run_moving_eyes, "fetch_slam_map", return_value={"ok": False}):
                self.assertEqual(run_moving_eyes.main(), 0)
            output_dir, = Path(directory).iterdir()
            summary = json.loads((output_dir / "summary.json").read_text())
            self.assertEqual(summary["inferences"], 2)
            self.assertEqual(summary["target_confirmation"]["target"], "green baseball cap")
            self.assertFalse(summary["dispatch_performed"])
            self.assertEqual(mission["state"], "running")
            self.assertFalse(capture.running)
            self.assertEqual(client.analyze.call_count, 2)
            self.assertEqual(len(list(output_dir.glob("*/target_confirmation.json"))), 1)

    def test_launcher_cancels_exact_mission_after_observer_confirmation(self):
        bridge = FakeBridge()
        args = options()
        args.target = "green baseball cap"
        args.stop_on_target = True
        def observed(command, **_):
            self.assertIn("--end-on-confirmed-target", command)
            self.assertEqual(command[command.index("--goal") + 1], "find")
            self.assertEqual(bridge.active["state"], "running")
            summary = {"run_id": "test", "mission_id": "mission-exact", "output_dir": "evidence",
                       "target_confirmation": {"target": "green baseball cap", "evidence": [
                           {"lease_id": "lease-exact"}, {"lease_id": "lease-exact"}]}}
            return subprocess.CompletedProcess(command, 0, stdout=json.dumps(summary) + "\n", stderr="")
        result = run(args, adapter=bridge, runner=observed)
        assert result["target_confirmation"] is not None
        self.assertEqual(result["target_confirmation"]["target"], "green baseball cap")
        self.assertEqual(result["eyes_output_dir"], "evidence")
        self.assertEqual(result["final_mission_state"], "cancelled")
        self.assertTrue(result["original_lease_released"])
        self.assertEqual([path for path, _ in bridge.calls], ["/mission/start", "/mission/cancel"])

    def test_no_target_confirmation_from_child_still_cleans_up(self):
        bridge = FakeBridge()
        args = options()
        args.target = "green baseball cap"
        args.stop_on_target = True
        result = run(args, adapter=bridge, runner=lambda command, **_: subprocess.CompletedProcess(
            command, 0, stdout='{"run_id":"test","mission_id":"mission-exact","target_confirmation":null}\n', stderr=""))
        self.assertIsNone(result["target_confirmation"])
        self.assertTrue(result["original_lease_released"])

    def test_mock_search_cannot_trigger_found_stop_on_hardware(self):
        bridge = FakeBridge()
        args = options()
        args.target = "green baseball cap"
        args.stop_on_target = True
        args.mock_response = FIXTURES / "eyes_response.json"
        with self.assertRaisesRegex(ValueError, "mock perception"):
            run(args, adapter=bridge)
        self.assertEqual(bridge.calls, [])

    def test_wrong_lease_evidence_is_rejected_and_original_lease_cleaned(self):
        bridge = FakeBridge()
        args = options()
        args.target = "green baseball cap"
        args.stop_on_target = True
        summary = {"run_id": "test", "mission_id": "mission-exact",
                   "target_confirmation": {"target": args.target,
                                           "evidence": [{"lease_id": "old"}, {"lease_id": "old"}]}}
        with self.assertRaisesRegex(RuntimeError, "exact goal/lease"):
            run(args, adapter=bridge, runner=lambda command, **_: subprocess.CompletedProcess(
                command, 0, stdout=json.dumps(summary) + "\n", stderr=""))
        self.assertIsNone(bridge.lease)


if __name__ == "__main__":
    unittest.main()
