from __future__ import annotations

import unittest

from scripts.probe_bridge_perception import jpeg_dimensions, summarize_latencies


class ProbeBridgePerceptionTests(unittest.TestCase):
    def test_summarize_latencies_includes_median_and_tail(self) -> None:
        summary = summarize_latencies([5.0, 1.0, 3.0, 2.0, 4.0])
        self.assertEqual(5, summary["samples"])
        self.assertEqual(1.0, summary["min_ms"])
        self.assertEqual(3.0, summary["median_ms"])
        self.assertEqual(5.0, summary["p95_ms"])
        self.assertEqual(5.0, summary["max_ms"])

    def test_summarize_latencies_handles_empty_input(self) -> None:
        summary = summarize_latencies([])
        self.assertEqual(0, summary["samples"])
        self.assertIsNone(summary["median_ms"])
        self.assertIsNone(summary["p95_ms"])

    def test_jpeg_dimensions_reads_baseline_sof(self) -> None:
        # SOI + SOF0 with 480x640 dimensions and enough segment payload.
        jpeg = b"\xff\xd8\xff\xc0\x00\x11\x08\x01\xe0\x02\x80" + (b"\x00" * 10)
        self.assertEqual((640, 480), jpeg_dimensions(jpeg))

    def test_jpeg_dimensions_rejects_non_jpeg(self) -> None:
        self.assertIsNone(jpeg_dimensions(b"not a jpeg"))


if __name__ == "__main__":
    unittest.main()
