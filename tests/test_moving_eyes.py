import io
import time
import unittest
from typing import Any
from unittest.mock import MagicMock, patch

from PIL import Image

from beep_eyes.contact_sheet import CapturedFrame
from beep_eyes.moving_window import ContinuousMovingCapture, MovingFrameRing, MovingFrameSample


def jpeg(color: Any = "gray"):
    output = io.BytesIO()
    Image.new("RGB", (160, 120), color).save(output, format="JPEG")
    return output.getvalue()


class MovingFrameRingTests(unittest.TestCase):
    def sample(self, index):
        return MovingFrameSample(
            CapturedFrame(f"m{index:06d}", 100.0 + index, jpeg((index * 10, 20, 30))),
            {"moving": True, "motion_lease_id": "lease-1", "last_command": "forward"},
        )

    def test_selects_chronological_four_six_or_nine_frame_windows(self):
        ring = MovingFrameRing(18)
        for index in range(12):
            ring.add(self.sample(index))
        self.assertEqual([item.frame.frame_id for item in ring.latest(4)], [f"m{i:06d}" for i in range(8, 12)])
        self.assertEqual([item.frame.frame_id for item in ring.latest(6)], [f"m{i:06d}" for i in range(6, 12)])
        self.assertEqual([item.frame.frame_id for item in ring.latest(9)], [f"m{i:06d}" for i in range(3, 12)])
        with self.assertRaisesRegex(ValueError, "4, 6, or 9"):
            ring.latest(5)

    def test_rejects_non_chronological_insert(self):
        ring = MovingFrameRing()
        ring.add(self.sample(1))
        with self.assertRaisesRegex(ValueError, "chronological"):
            ring.add(self.sample(0))


class ContinuousMovingCaptureTests(unittest.TestCase):
    def test_default_shutdown_wait_exceeds_network_timeout(self):
        thread = MagicMock()
        thread.is_alive.return_value = False
        capture = ContinuousMovingCapture("http://beep", timeout_s=15)
        capture._thread = thread
        capture.stop()
        self.assertGreaterEqual(thread.join.call_args.args[0], 16)

    def test_renders_six_and_nine_panel_moving_windows(self):
        capture = ContinuousMovingCapture("http://beep", fps=3.0, max_frames=18)
        for index in range(9):
            capture.ring.add(MovingFrameSample(
                CapturedFrame(f"m{index:06d}", 100.0 + index, jpeg((index * 10, 20, 30))),
                {"moving": True, "motion_lease_id": "lease-1"},
            ))
        for count, expected_size in ((6, (1440, 780)), (9, (1440, 1170))):
            frames, panel, packet = capture.temporal_window(count)
            self.assertEqual(len(frames), count)
            self.assertEqual(packet["panel_count"], count)
            self.assertEqual(Image.open(io.BytesIO(panel)).size, expected_size)

    def test_capture_continues_while_inference_would_be_running(self):
        status = {
            "moving": True,
            "motion_lease_id": "lease-1",
            "last_command": "forward",
            "scan_age_s": 0.03,
        }
        capture = ContinuousMovingCapture("http://beep", fps=6.0, max_frames=18, timeout_s=1)
        with patch("beep_eyes.moving_window.fetch_jpeg", return_value=jpeg()), \
             patch("beep_eyes.moving_window.fetch_json", return_value=status):
            capture.start()
            try:
                self.assertTrue(capture.ring.wait_for(4, 2.0))
                before = capture.stats()["captured"]
                time.sleep(0.4)  # Stand-in for one active Gemini request.
                after = capture.stats()["captured"]
                frames, panel, packet = capture.temporal_window(4)
            finally:
                capture.stop()
        self.assertGreater(after, before)
        self.assertEqual(len(frames), 4)
        self.assertEqual(packet["capture_mode"], "moving_temporal_ring")
        self.assertTrue(packet["bridge_status"]["moving"])
        self.assertTrue(packet["evidence_policy"]["motion_blur_tolerated"])
        image = Image.open(io.BytesIO(panel))
        self.assertEqual(image.size, (960, 780))


if __name__ == "__main__":
    unittest.main()
