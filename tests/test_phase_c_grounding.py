"""Phase C shadow visual grounding; all poses/frames are synthetic except the legacy fixture."""
import json
import math
import unittest
from pathlib import Path

from beep_agent.events import EventEnvelope
from beep_agent.pose_history import MapTransform
from beep_agent.visual_grounding import (
    CameraCalibration, Detection, GroundingError, ViewpointRequest,
    ground_viewpoint, validate_viewpoint_request,
)


def exposure(*, at=1.0, revision=3, snapshot="grid-1", posture="level", event_id="frame-1",
             evidence_id="img-1", yaw=0.0, quality="sensor_exposure", lease: str | None="lease-1"):
    return EventEnvelope.from_dict({
        "schema_version": 1, "event_id": event_id, "event_kind": "frame_acquired",
        "source": "camera", "robot_id": "beep", "session_id": "session-1", "boot_id": "boot-1",
        "mission_id": "mission-1" if lease else None, "lease_id": lease,
        "acquired_at_robot": at, "received_at_robot": at + 0.01,
        "robot_clock": "boot_monotonic", "timestamp_quality": quality,
        "acquired_at_ros_ns": int(at * 1e9), "map_epoch": 2,
        "map_revision": revision, "map_revision_status": "verified",
        "map_snapshot_id": snapshot, "evidence_id": evidence_id,
        "capture_local_transform": {
            "parent_frame": "odom", "child_frame": "base_link", "stamp_ros_ns": int(at * 1e9),
            "x_m": 1.0, "y_m": 2.0, "yaw_rad": yaw, "source": "synthetic_local_tf",
            "valid": True, "uncertainty": None},
        "payload": {"camera_posture_id": posture},
    })


def transform(*, at=1.0, revision=3, snapshot="grid-1", epoch=2):
    return MapTransform("boot-1", epoch, revision, snapshot, int(at * 1e9), 0, 0, 0)


class VisualGroundingTests(unittest.TestCase):
    def setUp(self):
        self.calibration = CameraCalibration("level", math.pi / 2, 0.1, 0.1, 0.2, 0.05)
        self.detection = Detection("cap", "object", "frame-1", "img-1", 0.8, 0.7, 0.9)

    def test_right_hand_bbox_yields_uncertain_cone_not_target_coordinate(self):
        cone = ground_viewpoint(exposure(), self.detection, transform(), self.calibration)
        self.assertIsNotNone(cone)
        assert cone is not None
        self.assertEqual(cone.evidence_id, "img-1")
        self.assertEqual((cone.map_epoch, cone.map_revision, cone.map_snapshot_id), (2, 3, "grid-1"))
        self.assertAlmostEqual(cone.origin_x_m, 1.1)
        self.assertAlmostEqual(cone.origin_y_m, 2.2)
        self.assertAlmostEqual(cone.bearing_center_rad, 0.1 - 0.3 * math.pi / 2)
        self.assertAlmostEqual(cone.bearing_half_width_rad, 0.1 * math.pi / 2 + 0.05)
        self.assertFalse(hasattr(cone, "target_x_m"))
        self.assertFalse(hasattr(cone, "range_m"))
        self.assertFalse(cone.motor_authority)

    def test_capture_yaw_and_mount_yaw_are_applied_once(self):
        cone = ground_viewpoint(exposure(yaw=math.pi / 2), self.detection, transform(), self.calibration)
        self.assertIsNotNone(cone)
        assert cone is not None
        self.assertAlmostEqual(cone.origin_x_m, 0.8)
        self.assertAlmostEqual(cone.origin_y_m, 2.1)
        self.assertAlmostEqual(cone.bearing_center_rad, math.pi / 2 + 0.1 - 0.3 * math.pi / 2)

    def test_exposure_posture_evidence_and_projection_fail_closed(self):
        base = exposure()
        for event, detection, opt, calibration in (
            (exposure(quality="ingest_approximate"), self.detection, transform(), self.calibration),
            (exposure(posture="sit"), self.detection, transform(), self.calibration),
            (exposure(evidence_id="different"), self.detection, transform(), self.calibration),
            (exposure(), Detection("cap", "object", "other-frame", "img-1", .8, .7, .9), transform(), self.calibration),
            (base, self.detection, transform(revision=4), self.calibration),
            (base, self.detection, transform(snapshot="grid-2"), self.calibration),
            (base, self.detection, transform(epoch=3), self.calibration),
            (base, self.detection, transform(at=4), self.calibration),
        ):
            with self.subTest(event=event, opt=opt):
                self.assertIsNone(ground_viewpoint(event, detection, opt, calibration))
        # Stationary observation is valid: metric perception does not require a motor lease.
        self.assertIsNone(ground_viewpoint(exposure(), Detection("cap", "object", "frame-1",
            "img-1", .64, .7, .9), transform(), self.calibration))
        self.assertIsNone(ground_viewpoint(exposure(), Detection("visitor", "person", "frame-1",
            "img-1", .69, .7, .9), transform(), self.calibration))
        self.assertIsNotNone(ground_viewpoint(exposure(lease=None), self.detection,
                                              transform(), self.calibration))

    def test_invalid_calibration_and_detection_rejected_not_clamped(self):
        for args in (("level", 0, 0, 0, 0, 0), ("level", math.pi, 0, 0, 0, 0),
                     ("level", 1, float("nan"), 0, 0, 0), ("level", 1, 0, 0, 0, -1)):
            with self.subTest(args=args), self.assertRaises(GroundingError):
                CameraCalibration(*args)
        for bbox in ((.9, .7), (-.1, .5), (.1, 1.1), (float("nan"), .4)):
            with self.subTest(bbox=bbox), self.assertRaises(GroundingError):
                Detection("cap", "object", "frame-1", "img-1", .8, *bbox)
        with self.assertRaises(GroundingError):
            Detection("cap", "object", "frame-1", "img-1", 1.1, .7, .9)

    def test_old_cap_fixture_is_not_a_metric_exposure_or_bounding_box(self):
        packet = json.loads((Path(__file__).parent / "fixtures/cap_confirmation_window0.json").read_text())
        self.assertEqual(packet["response"]["scene"]["objects"][0]["position"], "right")
        self.assertNotIn("bbox", packet["response"]["scene"]["objects"][0])
        # The saved packet's captured_at is not a proven exposure stamp.
        self.assertIsNone(ground_viewpoint(exposure(quality="ingest_approximate"),
                                           self.detection, transform(), self.calibration))

    def test_directive_is_advisory_and_fenced_to_exact_evidence_and_context(self):
        cone = ground_viewpoint(exposure(), self.detection, transform(), self.calibration)
        assert cone is not None
        req = ViewpointRequest("img-1", "cap", cone.bearing_center_rad, "boot-1", 2, 3,
                               "grid-1", "mission-1", "lease-1", 1.5)
        accepted = validate_viewpoint_request(req, cone, exposure(), now_robot=1.1,
                                               boot_id="boot-1", map_epoch=2, map_revision=3,
                                               map_snapshot_id="grid-1", mission_id="mission-1", lease_id="lease-1")
        self.assertTrue(accepted.accepted)
        self.assertFalse(accepted.motor_authority)
        for changed, reason in (
            ({"evidence_id": "other"}, "evidence_mismatch"),
            ({"map_revision": 4}, "map_context_mismatch"),
            ({"bearing_rad": cone.bearing_center_rad + 1.0}, "outside_viewpoint_cone"),
            ({"lease_id": "new-lease"}, "ownership_mismatch"),
        ):
            request = ViewpointRequest(**{**vars(req), **changed})
            self.assertEqual(validate_viewpoint_request(request, cone, exposure(),
                now_robot=1.1, boot_id="boot-1", map_epoch=2, map_revision=3,
                map_snapshot_id="grid-1", mission_id="mission-1", lease_id="lease-1").reason, reason)
        self.assertEqual(validate_viewpoint_request(req, cone, exposure(), now_robot=1.6,
            boot_id="boot-1", map_epoch=2, map_revision=3, map_snapshot_id="grid-1",
            mission_id="mission-1", lease_id="lease-1").reason, "expired")
        self.assertEqual(validate_viewpoint_request(req, cone, exposure(), now_robot=1.1,
            boot_id="boot-1", map_epoch=2, map_revision=4, map_snapshot_id="grid-1",
            mission_id="mission-1", lease_id="lease-1").reason, "map_context_mismatch")
        self.assertEqual(validate_viewpoint_request(req, cone, exposure(), now_robot=1.1,
            boot_id="boot-1", map_epoch=2, map_revision=3, map_snapshot_id="grid-2",
            mission_id="mission-1", lease_id="lease-1").reason, "map_context_mismatch")
        self.assertEqual(validate_viewpoint_request(req, cone, exposure(), now_robot=1.1,
            boot_id="boot-1", map_epoch=2, map_revision=3, map_snapshot_id="grid-1",
            mission_id="new-mission", lease_id="new-lease").reason, "ownership_mismatch")
        delayed = ViewpointRequest(**{**vars(req), "expires_at_robot": 4.0})
        self.assertEqual(validate_viewpoint_request(delayed, cone, exposure(), now_robot=1.1,
            boot_id="boot-1", map_epoch=2, map_revision=3, map_snapshot_id="grid-1",
            mission_id="mission-1", lease_id="lease-1").reason, "expired")


if __name__ == "__main__":
    unittest.main()
