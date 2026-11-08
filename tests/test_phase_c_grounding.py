"""Phase C shadow visual grounding; all poses/frames are synthetic except the legacy fixture."""
import json
import math
import unittest
from pathlib import Path
from typing import Any

from beep_agent.events import EventEnvelope
from beep_agent.pose_history import MapTransform
from beep_agent.visual_grounding import (
    CameraCalibration, Detection, GroundingError, ViewpointRequest,
    ground_viewpoint, validate_viewpoint_request,
)


def exposure(*, at=1.0, revision=3, snapshot="grid-1", posture="level", event_id="frame-1",
             evidence_id="img-1", yaw=0.0, quality="sensor_exposure", lease: str | None="lease-1",
             session="session-1", uncertainty=None):
    return EventEnvelope.from_dict({
        "schema_version": 1, "event_id": event_id, "event_kind": "frame_acquired",
        "source": "camera", "robot_id": "beep", "session_id": session, "boot_id": "boot-1",
        "mission_id": "mission-1" if lease else None, "lease_id": lease,
        "acquired_at_robot": at, "received_at_robot": at + 0.01,
        "robot_clock": "boot_monotonic", "timestamp_quality": quality,
        "acquired_at_ros_ns": int(at * 1e9), "map_epoch": 2,
        "map_revision": revision, "map_revision_status": "verified",
        "map_snapshot_id": snapshot, "evidence_id": evidence_id,
        "capture_local_transform": {
            "parent_frame": "odom", "child_frame": "base_link", "stamp_ros_ns": int(at * 1e9),
            "x_m": 1.0, "y_m": 2.0, "yaw_rad": yaw, "source": "synthetic_local_tf",
            "valid": True, "uncertainty": uncertainty},
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
        left = math.atan((1 - 2 * .7) * math.tan(math.pi / 4))
        right = math.atan((1 - 2 * .9) * math.tan(math.pi / 4))
        self.assertAlmostEqual(cone.bearing_center_rad, .1 + (left + right) / 2)
        self.assertAlmostEqual(cone.bearing_half_width_rad, (left - right) / 2 + .05)
        self.assertIsNone(cone.origin_xy_std_m)
        self.assertIsNone(cone.slam_yaw_std_rad)
        self.assertFalse(hasattr(cone, "target_x_m"))
        self.assertFalse(hasattr(cone, "range_m"))
        self.assertFalse(cone.motor_authority)

    def test_capture_yaw_and_mount_yaw_are_applied_once(self):
        cone = ground_viewpoint(exposure(yaw=math.pi / 2), self.detection, transform(), self.calibration)
        self.assertIsNotNone(cone)
        assert cone is not None
        self.assertAlmostEqual(cone.origin_x_m, 0.8)
        self.assertAlmostEqual(cone.origin_y_m, 2.1)
        left = math.atan((1 - 2 * .7) * math.tan(math.pi / 4))
        right = math.atan((1 - 2 * .9) * math.tan(math.pi / 4))
        self.assertAlmostEqual(cone.bearing_center_rad, math.pi / 2 + .1 + (left + right) / 2)

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
            (base, self.detection, transform(at=12), self.calibration),
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
                               "grid-1", "mission-1", "lease-1", 1.0, 1.5,
                               cone.origin_x_m + math.cos(cone.bearing_center_rad),
                               cone.origin_y_m + math.sin(cone.bearing_center_rad))
        accepted = validate_viewpoint_request(req, cone, exposure(), now_robot=1.1,
                                               current_session_id="session-1", boot_id="boot-1", map_epoch=2, map_revision=3,
                                               map_snapshot_id="grid-1", mission_id="mission-1", lease_id="lease-1")
        self.assertTrue(accepted.accepted)
        self.assertFalse(accepted.motor_authority)
        for changed, reason in (
            ({"evidence_id": "other"}, "evidence_mismatch"),
            ({"map_revision": 4}, "map_context_mismatch"),
            ({"target_look_bearing_rad": cone.bearing_center_rad + 1.0}, "outside_viewpoint_cone"),
            ({"viewpoint_x_m": cone.origin_x_m - 1.0,
              "viewpoint_y_m": cone.origin_y_m}, "outside_viewpoint_cone"),
            ({"viewpoint_x_m": cone.origin_x_m,
              "viewpoint_y_m": cone.origin_y_m}, "outside_viewpoint_cone"),
            ({"lease_id": "new-lease"}, "ownership_mismatch"),
        ):
            request = ViewpointRequest(**{**vars(req), **changed})
            self.assertEqual(validate_viewpoint_request(request, cone, exposure(),
                now_robot=1.1, current_session_id="session-1", boot_id="boot-1", map_epoch=2, map_revision=3,
                map_snapshot_id="grid-1", mission_id="mission-1", lease_id="lease-1").reason, reason)
        self.assertEqual(validate_viewpoint_request(req, cone, exposure(), now_robot=1.6,
            current_session_id="session-1", boot_id="boot-1", map_epoch=2, map_revision=3, map_snapshot_id="grid-1",
            mission_id="mission-1", lease_id="lease-1").reason, "expired")
        self.assertEqual(validate_viewpoint_request(req, cone, exposure(), now_robot=1.1,
            current_session_id="session-1", boot_id="boot-1", map_epoch=2, map_revision=4, map_snapshot_id="grid-1",
            mission_id="mission-1", lease_id="lease-1").reason, "map_context_mismatch")
        self.assertEqual(validate_viewpoint_request(req, cone, exposure(), now_robot=1.1,
            current_session_id="session-1", boot_id="boot-1", map_epoch=2, map_revision=3, map_snapshot_id="grid-2",
            mission_id="mission-1", lease_id="lease-1").reason, "map_context_mismatch")
        self.assertEqual(validate_viewpoint_request(req, cone, exposure(), now_robot=1.1,
            current_session_id="session-1", boot_id="boot-1", map_epoch=2, map_revision=3, map_snapshot_id="grid-1",
            mission_id="new-mission", lease_id="new-lease").reason, "ownership_mismatch")
        with self.assertRaises(GroundingError):
            ViewpointRequest(**{**vars(req), "expires_at_robot": 4.0})

    def test_lateral_observer_heading_is_not_capture_relative_request_bearing(self):
        cone = ground_viewpoint(exposure(), self.detection, transform(), self.calibration)
        assert cone is not None
        x = cone.origin_x_m + math.cos(cone.bearing_center_rad)
        y = cone.origin_y_m + math.sin(cone.bearing_center_rad)
        heading_from_current = math.atan2(y - 3.0, x - 1.0)
        self.assertGreater(abs(heading_from_current - cone.bearing_center_rad),
                           cone.bearing_half_width_rad)
        req = ViewpointRequest("img-1", "cap", cone.bearing_center_rad, "boot-1", 2, 3,
                               "grid-1", "mission-1", "lease-1", 1.0, 1.5, x, y)
        self.assertTrue(validate_viewpoint_request(req, cone, exposure(), now_robot=1.1,
            current_session_id="session-1", boot_id="boot-1", map_epoch=2, map_revision=3,
            map_snapshot_id="grid-1", mission_id="mission-1", lease_id="lease-1").accepted)

    def test_stationary_observation_can_bind_new_lease_but_stale_owner_cannot(self):
        stationary = exposure(lease=None)
        cone = ground_viewpoint(stationary, self.detection, transform(), self.calibration)
        assert cone is not None
        req = ViewpointRequest("img-1", "cap", cone.bearing_center_rad, "boot-1", 2, 3,
                               "grid-1", "mission-2", "lease-2", 1.0, 1.5,
                               cone.origin_x_m + math.cos(cone.bearing_center_rad),
                               cone.origin_y_m + math.sin(cone.bearing_center_rad))
        current: dict[str, Any] = dict(now_robot=1.1, current_session_id="session-1", boot_id="boot-1",
                       map_epoch=2, map_revision=3, map_snapshot_id="grid-1",
                       mission_id="mission-2", lease_id="lease-2")
        result = validate_viewpoint_request(req, cone, stationary, **current)
        self.assertTrue(result.accepted)
        self.assertFalse(result.motor_authority)
        self.assertEqual(validate_viewpoint_request(req, cone, stationary,
            **{**current, "current_session_id": "replacement"}).reason, "session_mismatch")
        self.assertEqual(validate_viewpoint_request(req, cone, stationary,
            **{**current, "lease_id": "lease-3"}).reason, "ownership_mismatch")
        self.assertEqual(validate_viewpoint_request(req, cone, stationary,
            **{**current, "map_revision": 4}).reason, "map_context_mismatch")
        old_owned = exposure(lease="lease-1")
        old_cone = ground_viewpoint(old_owned, self.detection, transform(), self.calibration)
        assert old_cone is not None
        self.assertEqual(validate_viewpoint_request(req, old_cone, old_owned,
            **current).reason, "ownership_mismatch")
        self.assertEqual(validate_viewpoint_request(req, cone, stationary,
            **{**current, "mission_id": None, "lease_id": None}).reason, "ownership_mismatch")

    def test_pinhole_bbox_edges_and_measured_slam_yaw_expand_cone(self):
        hfov = math.radians(110)
        calibration = CameraCalibration("level", hfov, .1, .1, .2, .05)
        detection = Detection("cap", "object", "frame-1", "img-1", .8, .6, .95)
        event = exposure(uncertainty={"xy_std_m": .03, "yaw_std_rad": .08})
        cone = ground_viewpoint(event, detection, transform(), calibration)
        assert cone is not None
        left = math.atan((1 - 2 * .6) * math.tan(hfov / 2))
        right = math.atan((1 - 2 * .95) * math.tan(hfov / 2))
        self.assertAlmostEqual(cone.bearing_center_rad, .1 + (left + right) / 2)
        self.assertAlmostEqual(cone.bearing_half_width_rad, (left - right) / 2 + .05 + .08)
        assert cone.origin_xy_std_m is not None and cone.slam_yaw_std_rad is not None
        self.assertAlmostEqual(cone.origin_xy_std_m, .03)
        self.assertAlmostEqual(cone.slam_yaw_std_rad, .08)
        self.assertNotAlmostEqual(cone.bearing_center_rad,
                                  .1 + (0.5 - (.6 + .95) / 2) * hfov, places=3)
        self.assertFalse(hasattr(cone, "target_x_m"))
        self.assertFalse(cone.motor_authority)

    def test_delayed_model_evidence_and_short_post_inference_ttl_are_separate(self):
        frame = exposure(at=1.0)
        # The map transform is acquired at inference completion, not exposure.
        cone = ground_viewpoint(frame, self.detection, transform(at=8.8), self.calibration)
        assert cone is not None
        req = ViewpointRequest("img-1", "cap", cone.bearing_center_rad, "boot-1", 2, 3,
                               "grid-1", "mission-1", "lease-1", 8.9, 10.4,
                               cone.origin_x_m + math.cos(cone.bearing_center_rad),
                               cone.origin_y_m + math.sin(cone.bearing_center_rad))
        current: dict[str, Any] = dict(current_session_id="session-1", boot_id="boot-1", map_epoch=2,
                       map_revision=3, map_snapshot_id="grid-1", mission_id="mission-1",
                       lease_id="lease-1")
        self.assertTrue(validate_viewpoint_request(req, cone, frame, now_robot=9.1,
                                                   **current).accepted)
        self.assertEqual(validate_viewpoint_request(req, cone, frame, now_robot=10.4,
            **current).reason, "expired")
        self.assertIsNone(ground_viewpoint(frame, self.detection,
            transform(at=11.1), self.calibration))
        self.assertIsNone(ground_viewpoint(frame, self.detection,
            transform(at=8.8, revision=4), self.calibration))
        late = ViewpointRequest(**{**vars(req), "issued_at_robot": 9.8,
                                   "expires_at_robot": 11.3})
        self.assertEqual(validate_viewpoint_request(late, cone, frame, now_robot=11.05,
            **current).reason, "expired")  # exposure-age ceiling, despite fresh TTL
        for change in ({"issued_at_robot": 8.9, "expires_at_robot": 10.91},
                       {"issued_at_robot": 10.0, "expires_at_robot": 10.0},
                       {"issued_at_robot": float("nan"), "expires_at_robot": 10.0}):
            with self.subTest(change=change), self.assertRaises(GroundingError):
                ViewpointRequest(**{**vars(req), **change})

    def test_moving_views_are_separate_rays_not_triangulated_points(self):
        first = exposure(event_id="frame-1", evidence_id="img-1")
        second = exposure(at=1.3, event_id="frame-2", evidence_id="img-2", yaw=.2)
        second_data = second.to_dict()
        second_data["capture_local_transform"]["x_m"] = 1.4
        second = EventEnvelope.from_dict(second_data)
        a = ground_viewpoint(first, self.detection, transform(), self.calibration)
        b = ground_viewpoint(second, Detection("cap", "object", "frame-2", "img-2",
                   .8, .7, .9), transform(at=1.3), self.calibration)
        assert a is not None and b is not None
        self.assertNotEqual(a.origin_x_m, b.origin_x_m)
        self.assertNotEqual(a.bearing_center_rad, b.bearing_center_rad)
        self.assertFalse(hasattr(a, "target_x_m"))
        self.assertFalse(hasattr(b, "target_x_m"))


if __name__ == "__main__":
    unittest.main()
