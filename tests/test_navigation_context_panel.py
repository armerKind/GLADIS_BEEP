import io
import math
import unittest

from PIL import Image

from beep_eyes.contact_sheet import CapturedFrame, build_navigation_contact_sheet, summarize_navigation_context


def jpeg(color):
    output = io.BytesIO()
    Image.new("RGB", (80, 60), color).save(output, format="JPEG")
    return output.getvalue()


class NavigationContextPanelTests(unittest.TestCase):
    def test_panel_embeds_cartographer_map_pose_heading_and_frame_track(self):
        frames = [CapturedFrame(f"m{i}", 100.0 + i, jpeg((i * 40, 30, 90))) for i in range(4)]
        statuses = [
            {"pose": {"x": i * 0.1, "y": 0.0, "yaw": 0.1 * i, "confidence": 0.9,
                      "source": "guarded_cartographer_slam"},
             "slam": {"pose_valid": True, "usable": True, "map_age_s": 0.2},
             "moving": True, "motion_lease_id": "lease-1", "scan_age_s": 0.03}
            for i in range(4)
        ]
        slam_map = {
            "ok": True, "frame_id": "map", "width": 4, "height": 4,
            "resolution_m": 0.1, "origin_x": -0.2, "origin_y": -0.2,
            "origin_yaw": 0.0, "data": [0, 0, 100, 100, -1, -1, 50, 100,
                                           0, 0, -1, -1, 0, 0, 0, 100],
            "map_age_s": 0.2, "known_cells": 12, "occupied_cells": 5,
        }
        panel = build_navigation_contact_sheet(
            frames, frame_statuses=statuses, current_status=statuses[-1], slam_map=slam_map,
            columns=2, panel_size=(160, 120), label_height=36, context_height=180,
        )
        with Image.open(io.BytesIO(panel)) as source:
            image = source.convert("RGB")
            self.assertEqual(image.size, (320, 492))
            colors = image.crop((0, 312, 320, 492)).getcolors(maxcolors=1_000_000)
        self.assertGreater(len(colors or []), 3)
        context = summarize_navigation_context(statuses[-1], slam_map)
        self.assertTrue(context["map_available"])
        self.assertTrue(context["map_fresh"])
        self.assertEqual(context["map_origin_yaw_deg"], 0.0)
        self.assertTrue(context["pose_valid"])
        self.assertAlmostEqual(context["view_heading_deg"], math.degrees(0.3), delta=0.1)
        self.assertEqual(context["map_source"], "cartographer_occupancy_grid")

    def test_invalid_pose_is_explicit_and_heading_is_not_invented(self):
        status = {
            "pose": {"x": 1.0, "y": 2.0, "yaw": 0.7, "confidence": 0.2},
            "slam": {"pose_valid": False, "usable": False, "pose_guard_reason": "stationary_drift"},
        }
        context = summarize_navigation_context(status, {"ok": False, "reason": "stale_map"})
        self.assertFalse(context["map_available"])
        self.assertFalse(context["pose_valid"])
        self.assertIsNone(context["view_heading_deg"])
        self.assertEqual(context["pose_guard_reason"], "stationary_drift")
        stale = summarize_navigation_context(status, {
            "ok": True, "width": 1, "height": 1, "resolution_m": 0.05,
            "origin_x": 0.0, "origin_y": 0.0, "origin_yaw": 0.0,
            "data": [-1], "map_age_s": 4.0,
        })
        self.assertTrue(stale["map_available"])
        self.assertFalse(stale["map_fresh"])


if __name__ == "__main__":
    unittest.main()
