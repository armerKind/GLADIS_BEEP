"""Two fresh, changed-viewpoint object sightings terminate a search, not one model guess."""
import json
import math
from pathlib import Path
import unittest

from beep_agent.target_confirmation import TargetConfirmation


FIXTURES = Path(__file__).resolve().parent / "fixtures"


def observation(name):
    index = {"w0000-m000000": 0, "w0001-m000012": 1}[name]
    value = json.loads((FIXTURES / f"cap_confirmation_window{index}.json").read_text())
    return value["response"], value["packet"]


class TargetConfirmationTests(unittest.TestCase):
    def test_real_cap_frames_confirm_after_changed_viewpoint(self):
        tracker = TargetConfirmation("green baseball cap")
        first = observation("w0000-m000000")
        second = observation("w0001-m000012")
        self.assertIsNone(tracker.observe(*first, track_id="object-0001"))
        result = tracker.observe(*second, track_id="object-0001")
        assert result is not None
        self.assertEqual(result["target"], "green baseball cap")
        self.assertEqual(result["track_id"], "object-0001")
        self.assertEqual(len(result["evidence"]), 2)
        self.assertGreater(result["viewpoint_change_m"], 0.2)

    def test_duplicate_or_stationary_frame_is_not_second_view(self):
        response, packet = observation("w0000-m000000")
        tracker = TargetConfirmation("green baseball cap")
        tracker.observe(response, packet, track_id="object-0001")
        self.assertIsNone(tracker.observe(response, packet, track_id="object-0001"))
        other = observation("w0001-m000012")
        other[1]["frames"][-1]["captured_at"] = packet["frames"][-1]["captured_at"]
        self.assertIsNone(tracker.observe(*other, track_id="object-0001"))

    def test_wrong_or_unattended_object_does_not_confirm(self):
        response, packet = observation("w0000-m000000")
        tracker = TargetConfirmation("green baseball cap")
        tracker.observe(response, packet, track_id="object-0001")
        response2, packet2 = observation("w0001-m000012")
        response2["scene"]["objects"][0]["description"] = "Green bottle on the floor"
        self.assertIsNone(tracker.observe(response2, packet2, track_id="object-0001"))
        self.assertIsNone(tracker.confirmed)
        response2["scene"]["objects"][0]["description"] = "Green baseball cap on the floor"
        response2["attention"]["target_id"] = "coffee_table"
        self.assertIsNone(tracker.observe(response2, packet2, track_id="object-0001"))

    def test_identity_confidence_uncertainty_and_lease_gate(self):
        first = observation("w0000-m000000")
        second = observation("w0001-m000012")
        for change in (lambda r,p: r["scene"]["objects"][0].update(confidence=0.6),
                       lambda r,p: r.update(uncertainty=["Maybe a green baseball cap, uncertain"]),
                       lambda r,p: p["frames"][-1]["motion"].update(motion_lease_id="other-lease")):
            tracker = TargetConfirmation("green baseball cap")
            tracker.observe(*first, track_id="object-0001")
            response, packet = observation("w0001-m000012")
            change(response, packet)
            self.assertIsNone(tracker.observe(response, packet, track_id="object-0001"))
        tracker = TargetConfirmation("green baseball cap")
        tracker.observe(*first, track_id="object-0001")
        self.assertIsNone(tracker.observe(*second, track_id="object-0099"))

    def test_yaw_only_changed_view_is_valid_and_not_two_copies_in_panel(self):
        response1, packet1 = observation("w0000-m000000")
        response2, packet2 = observation("w0001-m000012")
        pose1 = packet1["bridge_status"]["pose"]
        packet2["frames"][-1]["pose"] = dict(pose1, yaw=pose1["yaw"] + math.radians(30))
        tracker = TargetConfirmation("green baseball cap")
        tracker.observe(response1, packet1, track_id="object-0001")
        result = tracker.observe(response2, packet2, track_id="object-0001")
        assert result is not None
        self.assertGreater(result["viewpoint_change_rad"], 0.4)
        self.assertLess(result["viewpoint_change_m"], 0.01)

    def test_intentional_stop_before_second_view_does_not_erase_visual_evidence(self):
        first = observation("w0000-m000000")
        second = observation("w0001-m000012")
        second[1]["frames"][-1]["motion"]["moving"] = False
        tracker = TargetConfirmation("green baseball cap")
        self.assertIsNone(tracker.observe(*first, track_id="object-0001"))
        self.assertIsNotNone(tracker.observe(*second, track_id="object-0001"))


if __name__ == "__main__":
    unittest.main()
