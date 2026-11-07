"""Regression: a corrupt/expanding SLAM map must not starve safety scans."""
import copy
import math
import time
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

from beep_bridge import beep_bridge as bridge


def map_msg(width, height, data, resolution=0.05):
    return SimpleNamespace(
        info=SimpleNamespace(width=width, height=height, resolution=resolution,
            origin=SimpleNamespace(position=SimpleNamespace(x=0, y=0),
                orientation=SimpleNamespace(x=0, y=0, z=0, w=1))),
        data=data, header=SimpleNamespace(frame_id="map"))


class ForbiddenData:
    def __iter__(self):
        raise AssertionError("oversized occupancy data was materialized")


class MapLoadShedTests(unittest.TestCase):
    def setUp(self):
        self.prior_state = copy.deepcopy(bridge.state["slam"])
        self.prior_grid = bridge.slam_grid
        self.prior_count = bridge.state["scan_count"]
        self.prior_at = bridge.state["scan_at"]
        self.prior_error = bridge.state["last_error"]
        self.addCleanup(self.restore)

    def restore(self):
        bridge.state["slam"] = self.prior_state
        bridge.slam_grid = self.prior_grid
        bridge.state["scan_count"] = self.prior_count
        bridge.state["scan_at"] = self.prior_at
        bridge.state["last_error"] = self.prior_error

    def test_oversized_map_is_not_converted_and_invalidates_previous_map(self):
        bridge.state["slam"].update(map_at=123.0, map_width=10, map_height=10)
        bridge.slam_grid = object()
        self.assertFalse(bridge.occupancy_callback(map_msg(9312, 3293, ForbiddenData())))
        slam = bridge.state["slam"]
        self.assertIsNone(bridge.slam_grid)
        self.assertIsNone(slam["map_at"])
        self.assertEqual(slam["map_rejected"], "invalid_dimensions_or_data_length")
        self.assertEqual((slam["map_width"], slam["map_height"]), (9312, 3293))

    def test_small_map_accepts_with_fresh_subscription(self):
        self.assertTrue(bridge.occupancy_callback(map_msg(2, 2, [-1, 0, 50, 100])))
        slam = bridge.state["slam"]
        self.assertIsNone(slam["map_rejected"])
        self.assertEqual((slam["known_cells"], slam["occupied_cells"]), (3, 2))
        self.assertEqual(bridge.slam_grid.data, (-1, 0, 50, 100))

    def test_malformed_length_and_geometry_revoke_old_grid(self):
        valid = map_msg(2, 2, [-1, 0, 50, 100])
        self.assertTrue(bridge.occupancy_callback(valid))
        for invalid in (map_msg(2, 2, [0, 1, 2]),
                        map_msg(2, 2, [0, 1, 2, 3, 4]),
                        map_msg(2, 2, [0] * 4, resolution=float("inf")),
                        map_msg(2, 2, [0] * 4, resolution=-0.1)):
            self.assertFalse(bridge.occupancy_callback(invalid))
            self.assertIsNone(bridge.slam_grid)
            self.assertIsNone(bridge.state["slam"]["map_at"])
            self.assertFalse(bridge.slam_map_snapshot()["ok"])
            self.assertTrue(bridge.occupancy_callback(valid))

    def test_rejection_unsubscribes_once_and_scans_continue(self):
        node = MagicMock()
        node.map_feed_shed = False
        node.map_subscription = object()
        bridge.receive_ros_map(node, map_msg(9312, 3293, ForbiddenData()))
        node.destroy_subscription.assert_called_once()
        self.assertIsNone(node.map_subscription)
        self.assertIsNone(bridge.state["slam"]["map_at"])
        bridge.receive_ros_map(node, map_msg(2, 2, [0] * 4))
        node.destroy_subscription.assert_called_once()
        scan_before = bridge.state["scan_count"]
        scan = SimpleNamespace(angle_min=-math.pi, angle_increment=math.pi / 180,
                               ranges=[1.0] * 360, range_max=8.0)
        bridge.scan_callback(scan)
        self.assertEqual(bridge.state["scan_count"], scan_before + 1)
        self.assertLess(time.time() - bridge.state["scan_at"], 1.0)

    def test_destroy_failure_is_contained_and_map_invalid(self):
        node = MagicMock()
        node.map_feed_shed = False
        node.map_subscription = object()
        node.destroy_subscription.side_effect = RuntimeError("ROS teardown failed")
        bridge.receive_ros_map(node, map_msg(9312, 3293, ForbiddenData()))
        self.assertIn("map unsubscribe failed", bridge.state["last_error"])
        self.assertIsNone(bridge.state["slam"]["map_at"])
        bridge.receive_ros_map(node, map_msg(2, 2, [0] * 4))
        self.assertTrue(node.map_feed_shed)
        self.assertIsNone(bridge.state["slam"]["map_at"])
        node.destroy_subscription.assert_called_once()


if __name__ == "__main__":
    unittest.main()
