"""Shadow map-space viewpoint replay: never a motor route or target coordinate."""
import math
import unittest
from dataclasses import replace
from typing import Any
from unittest.mock import patch

from beep_agent.events import EventEnvelope
from beep_agent.place_graph import GridGeometry
from beep_agent.pose_history import MapTransform
from beep_agent.visual_grounding import (CameraCalibration, Detection, ViewpointRequest,
                                          ground_viewpoint, validate_viewpoint_request)
from beep_agent.viewpoint_scorer import OccupancySnapshot, score_viewpoint


def event_at(x=1.0, y=2.0, yaw=0.0):
    value = {
        "schema_version": 1, "event_id": "frame-1", "event_kind": "frame_acquired",
        "source": "camera", "robot_id": "beep", "session_id": "session-1", "boot_id": "boot-1",
        "mission_id": None, "lease_id": None, "acquired_at_robot": 1.0,
        "received_at_robot": 1.01, "robot_clock": "boot_monotonic",
        "timestamp_quality": "sensor_exposure", "acquired_at_ros_ns": 1_000_000_000,
        "map_epoch": 2, "map_revision": 3, "map_revision_status": "verified",
        "map_snapshot_id": "grid-1", "evidence_id": "img-1",
        "capture_local_transform": {
            "parent_frame": "odom", "child_frame": "base_link", "stamp_ros_ns": 1_000_000_000,
            "x_m": x, "y_m": y, "yaw_rad": yaw, "source": "synthetic_local_tf",
            "valid": True, "uncertainty": {"xy_std_m": .01, "yaw_std_rad": .01}},
        "payload": {"camera_posture_id": "level"},
    }
    return EventEnvelope.from_dict(value)


def cone_at(x=1.0, y=2.0, yaw=0.0):
    return ground_viewpoint(event_at(x, y, yaw),
        Detection("cap", "object", "frame-1", "img-1", .9, .49, .51),
        MapTransform("boot-1", 2, 3, "grid-1", 1_100_000_000, 0, 0, 0),
        CameraCalibration("level", math.pi/2, 0, 0, 0, .01))


def grid(*, unknown=(), occupied=(), revision=3, snapshot="grid-1"):
    geometry = GridGeometry(-5.0, -5.0, .1, "map")
    data = [0] * (100 * 100)
    for x, y in unknown:
        data[y * 100 + x] = -1
    for x, y in occupied:
        data[y * 100 + x] = 100
    return OccupancySnapshot("boot-1", 2, revision, snapshot, geometry, 100, 100,
                             tuple(data))


class ViewpointScorerTests(unittest.TestCase):
    def test_scored_candidate_round_trips_through_request_with_distinct_bearings(self):
        cone = cone_at()
        assert cone is not None
        snapshot = grid()
        candidate = score_viewpoint(cone, snapshot,
            current_map_context=snapshot.identity, current_x_m=1.0,
            current_y_m=3.0, footprint_radius_m=.15)
        assert candidate is not None
        self.assertNotAlmostEqual(candidate.heading_from_current_rad,
                                  candidate.target_look_bearing_rad)
        request = ViewpointRequest(candidate.evidence_id, "cap",
            candidate.target_look_bearing_rad, candidate.boot_id,
            candidate.map_epoch, candidate.map_revision, candidate.map_snapshot_id,
            "mission-1", "lease-1", 1.0, 1.5, candidate.x_m, candidate.y_m)
        current: dict[str, Any] = dict(now_robot=1.1, current_session_id="session-1",
            boot_id="boot-1", map_epoch=2, map_revision=3,
            map_snapshot_id="grid-1", mission_id="mission-1", lease_id="lease-1")
        self.assertTrue(validate_viewpoint_request(request, cone, event_at(),
                                                   **current).accepted)
        wrong = replace(request, target_look_bearing_rad=candidate.heading_from_current_rad)
        self.assertEqual(validate_viewpoint_request(wrong, cone, event_at(),
            **current).reason, "outside_viewpoint_cone")

    def test_heading_is_from_current_pose_to_viewpoint_not_parallel_capture_ray(self):
        cone = cone_at()
        assert cone is not None
        result = score_viewpoint(cone, grid(), current_map_context=("boot-1", 2, 3, "grid-1"),
                                 current_x_m=1.0, current_y_m=3.0,
                                 footprint_radius_m=.15, min_look_m=.7, max_look_m=1.5)
        assert result is not None
        self.assertGreater(result.x_m, cone.origin_x_m)
        self.assertAlmostEqual(result.y_m, cone.origin_y_m, delta=.06)
        self.assertLess(result.heading_from_current_rad, -.4)
        self.assertNotAlmostEqual(result.heading_from_current_rad, cone.bearing_center_rad)
        self.assertFalse(result.motor_authority)
        self.assertFalse(hasattr(result, "target_x_m"))
        self.assertEqual(result.evidence_id, "img-1")

    def test_unknown_and_occupied_wedge_blocks_candidate(self):
        cone = cone_at()
        assert cone is not None
        # Cell (62,70) is x=1.2,y=2.0 on the viewing wedge.
        for blocked in (grid(unknown=((62, 70),)), grid(occupied=((62, 70),))):
            with self.subTest(blocked=blocked.data[70 * 100 + 62]):
                self.assertIsNone(score_viewpoint(cone, blocked, current_map_context=("boot-1", 2, 3, "grid-1"), current_x_m=1,
                    current_y_m=3, footprint_radius_m=.15, min_look_m=.7, max_look_m=1.5))

    def test_revision_snapshot_and_unknown_pose_uncertainty_fail_closed(self):
        cone = cone_at()
        assert cone is not None
        for candidate in (grid(revision=4), grid(snapshot="grid-2")):
            self.assertIsNone(score_viewpoint(cone, candidate, current_map_context=("boot-1", 2, 3, "grid-1"), current_x_m=1,
                current_y_m=3, footprint_radius_m=.15))
        self.assertIsNone(score_viewpoint(replace(cone, origin_xy_std_m=None), grid(),
            current_map_context=("boot-1", 2, 3, "grid-1"), current_x_m=1,
            current_y_m=3, footprint_radius_m=.15))
        self.assertIsNone(score_viewpoint(cone, grid(),
            current_map_context=("boot-1", 2, 4, "grid-1"), current_x_m=1,
            current_y_m=3, footprint_radius_m=.15))

    def test_rotated_grid_origin_keeps_candidate_in_map_coordinates(self):
        cone = cone_at()
        assert cone is not None
        rotated = OccupancySnapshot("boot-1", 2, 3, "grid-1",
            GridGeometry(-5.0, 5.0, .1, "map", -math.pi / 2), 100, 100,
            (0,) * 10_000)
        result = score_viewpoint(cone, rotated,
            current_map_context=("boot-1", 2, 3, "grid-1"), current_x_m=1,
            current_y_m=3, footprint_radius_m=.15)
        assert result is not None
        self.assertGreater(result.x_m, cone.origin_x_m)
        self.assertAlmostEqual(result.y_m, cone.origin_y_m, delta=.06)
        self.assertEqual((result.boot_id, result.map_epoch, result.map_revision),
                         ("boot-1", 2, 3))

    def test_unknown_cell_touching_capture_origin_vetoes_even_when_center_is_behind_ray(self):
        cone = cone_at()
        assert cone is not None
        # (59,70) is diagonally behind the exposure ray but its square touches
        # the capture origin; center-angle alone must not silently skip it.
        self.assertIsNone(score_viewpoint(cone, grid(unknown=((59, 70),)),
            current_map_context=("boot-1", 2, 3, "grid-1"), current_x_m=1,
            current_y_m=3, footprint_radius_m=.15))

    def test_wedge_scanned_once_and_distant_unknown_does_not_veto_near_viewpoint(self):
        cone = cone_at()
        assert cone is not None
        snapshot = grid(unknown=((74, 70),))  # 1.4 m along capture ray, past 0.7 m candidate
        original = OccupancySnapshot.first_wedge_block_m
        with patch.object(OccupancySnapshot, "first_wedge_block_m",
                          autospec=True) as scan:
            scan.side_effect = lambda obj, ray, distance: original(obj, ray, distance)
            result = score_viewpoint(cone, snapshot,
                current_map_context=snapshot.identity, current_x_m=1,
                current_y_m=3, footprint_radius_m=.15)
            self.assertEqual(scan.call_count, 1)
        self.assertIsNotNone(result)
        assert result is not None
        self.assertLess(result.x_m, 2.0)

    def test_forward_motion_does_not_choose_a_viewpoint_behind_current_pose(self):
        cone = cone_at()
        assert cone is not None
        result = score_viewpoint(cone, grid(),
            current_map_context=("boot-1", 2, 3, "grid-1"),
            current_x_m=2.0, current_y_m=2.0, footprint_radius_m=.15,
            min_look_m=.7, max_look_m=1.5)
        assert result is not None
        self.assertGreater(result.x_m, 2.0)
        self.assertAlmostEqual(result.heading_from_current_rad, 0)

    def test_blocked_wedge_needs_full_body_clearance_and_valid_current_pose(self):
        cone = cone_at()
        assert cone is not None
        # Wide uncertainty wedge intersects off-axis unknown near 0.85 m;
        # the body at the first center-ray step can otherwise miss that cell.
        cone = replace(cone, bearing_half_width_rad=.3)
        blocked = grid(unknown=((68, 72),))
        self.assertIsNone(score_viewpoint(cone, blocked,
            current_map_context=blocked.identity, current_x_m=1,
            current_y_m=2, footprint_radius_m=.2, min_look_m=.7))
        outside = grid()
        self.assertIsNone(score_viewpoint(cone, outside,
            current_map_context=outside.identity, current_x_m=100,
            current_y_m=100, footprint_radius_m=.15))

    def test_grid_contract_rejects_bad_dimensions_and_unknown_values(self):
        geometry = GridGeometry(0, 0, .1, "map")
        for data in ((0,), (200,) * 4, (-2,) * 4):
            with self.subTest(data=data), self.assertRaises(ValueError):
                OccupancySnapshot("boot-1", 2, 3, "grid-1", geometry, 2, 2, data)


        for resolution in (1e-8, .75):
            with self.subTest(resolution=resolution), self.assertRaises(ValueError):
                OccupancySnapshot("boot-1", 2, 3, "grid-1",
                    GridGeometry(0, 0, resolution, "map"), 2, 2, (0,) * 4)

if __name__ == "__main__":
    unittest.main()
