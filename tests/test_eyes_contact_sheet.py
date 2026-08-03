import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from PIL import Image

from beep_eyes.contact_sheet import MAX_FRAME_BYTES, CapturedFrame, build_contact_sheet, capture_sequence, fetch_jpeg, load_replay_sequence, write_capture_artifacts


def jpeg(color, size=(160, 120)):
    output = io.BytesIO()
    Image.new("RGB", size, color).save(output, format="JPEG")
    return output.getvalue()


class ContactSheetTests(unittest.TestCase):
    def test_fetch_rejects_oversized_frame(self):
        class Response:
            def __enter__(self): return self
            def __exit__(self, *_): return False
            def read(self, _limit=None): return b"x" * (MAX_FRAME_BYTES + 1)

        with patch("urllib.request.urlopen", return_value=Response()):
            with self.assertRaisesRegex(ValueError, "too large"):
                fetch_jpeg("http://beep")

    def frames(self):
        return [
            CapturedFrame("f00", 100.0, jpeg("red")),
            CapturedFrame("f01", 101.5, jpeg("green")),
            CapturedFrame("f02", 103.0, jpeg("blue")),
            CapturedFrame("f03", 104.5, jpeg("white")),
        ]

    def test_builds_decodable_two_by_two_sheet(self):
        data = build_contact_sheet(self.frames(), columns=2, panel_size=(160, 120), label_height=20)
        image = Image.open(io.BytesIO(data))
        self.assertEqual(image.size, (320, 280))
        self.assertEqual(image.format, "JPEG")

    def test_writes_packet_and_latest_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            frames = self.frames()
            packet = write_capture_artifacts(output, frames, {"moving": False}, build_contact_sheet(frames))
            self.assertEqual(packet["latest_frame_id"], "f03")
            self.assertEqual(packet["frames"][0]["quality"]["width"], 160)
            self.assertIsNotNone(packet["frames"][1]["quality"]["difference_from_previous"])
            self.assertTrue(Path(packet["artifacts"]["contact_sheet"]).exists())
            self.assertTrue((output / "observation_packet.json").exists())

    def test_stationary_capture_refuses_active_lease(self):
        with patch("beep_eyes.contact_sheet.fetch_json", return_value={"moving": True, "motion_lease_id": "motion-1"}), \
             patch("beep_eyes.contact_sheet.fetch_jpeg") as fetch:
            with self.assertRaisesRegex(RuntimeError, "moving"):
                capture_sequence("http://beep", 2, 0)
            fetch.assert_not_called()

    def test_capture_discards_warmup_and_retries_transient_failure(self):
        images = [ValueError("startup"), jpeg("black"), jpeg("red"), jpeg("blue")]
        with patch("beep_eyes.contact_sheet.fetch_json", return_value={"moving": False, "motion_lease_id": None}), \
             patch("beep_eyes.contact_sheet.fetch_jpeg", side_effect=images) as fetch, \
             patch("beep_eyes.contact_sheet.time.sleep"):
            frames, _ = capture_sequence("http://beep", 2, 0)
        self.assertEqual(fetch.call_count, 4)
        self.assertEqual(frames[0].jpeg, jpeg("red"))
        self.assertEqual(frames[1].jpeg, jpeg("blue"))

    def test_loads_stationary_replay_without_bridge(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)
            (source / "f00.jpg").write_bytes(jpeg("red"))
            (source / "f01.jpg").write_bytes(jpeg("blue"))
            packet = {
                "frames": [{"frame_id": "f00", "captured_at": 10.0}, {"frame_id": "f01", "captured_at": 11.5}],
                "bridge_status": {"moving": False, "motion_lease_id": None},
            }
            (source / "observation_packet.json").write_text(json.dumps(packet))
            frames, status = load_replay_sequence(source)
        self.assertEqual([frame.captured_at for frame in frames], [10.0, 11.5])
        self.assertFalse(status["moving"])

    def test_replay_rejects_non_chronological_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)
            (source / "f00.jpg").write_bytes(jpeg("red"))
            (source / "f01.jpg").write_bytes(jpeg("blue"))
            packet = {
                "frames": [{"frame_id": "f00", "captured_at": 11.0}, {"frame_id": "f01", "captured_at": 10.0}],
                "bridge_status": {"moving": False, "motion_lease_id": None},
            }
            (source / "observation_packet.json").write_text(json.dumps(packet))
            with self.assertRaisesRegex(ValueError, "chronological"):
                load_replay_sequence(source)


if __name__ == "__main__":
    unittest.main()
