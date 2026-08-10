import math
import random
import unittest

from beep_bridge.frontier_planner import (
    OccupancyGrid,
    choose_natural_motion,
    find_frontier_plan,
    inflate_obstacles,
)


class FrontierPlannerTests(unittest.TestCase):
    def room_grid(self):
        width = height = 15
        data = [-1] * (width * height)
        for y in range(4, 11):
            for x in range(4, 11):
                data[y * width + x] = 0
        data[10 * width + 10] = 100
        return OccupancyGrid(width, height, 0.10, 0.0, 0.0, data)

    def test_frontier_plan_reaches_known_free_boundary_through_free_space(self):
        grid = self.room_grid()
        plan = find_frontier_plan(grid, (0.7, 0.7, 0.0), random.Random(4), chaos=0.35)

        self.assertIsNotNone(plan)
        self.assertGreaterEqual(plan["frontier_clusters"], 1)
        self.assertEqual(plan["path"][0], (7, 7))
        self.assertIn(plan["target_cell"][0], range(4, 11))
        self.assertIn(plan["target_cell"][1], range(4, 11))
        self.assertTrue(all(grid.value(x, y) == 0 for x, y in plan["path"]))

    def test_obstacle_inflation_reserves_space_for_legs(self):
        grid = self.room_grid()
        blocked = inflate_obstacles(grid, radius_cells=4)

        self.assertIn((10, 10), blocked)
        self.assertIn((6, 10), blocked)
        self.assertIn((10, 6), blocked)
        self.assertNotIn((4, 4), blocked)

    def test_natural_motion_turns_toward_waypoint_then_varies_stride(self):
        sectors = {"front": 1.2, "front_left": 0.8, "front_right": 0.8, "left": 0.8, "right": 0.8}
        turn = choose_natural_motion((0.0, 0.0, 0.0), (0.0, 1.0), sectors, random.Random(1), chaos=0.4)
        stride = choose_natural_motion((0.0, 0.0, 0.0), (1.0, 0.0), sectors, random.Random(2), chaos=0.4)

        self.assertEqual(turn["action"], "turnleft")
        self.assertGreaterEqual(turn["duration"], 0.50)
        self.assertLessEqual(turn["duration"], 1.75)
        self.assertEqual(stride["action"], "forward")
        self.assertGreaterEqual(stride["duration"], 0.62)
        self.assertLessEqual(stride["duration"], 1.08)
        self.assertEqual(stride["step"], 20)

    def test_natural_motion_uses_open_side_when_front_is_blocked(self):
        sectors = {"front": 0.31, "front_left": 0.30, "front_right": 0.70, "left": 0.40, "right": 1.10}
        decision = choose_natural_motion((0.0, 0.0, 0.0), (1.0, 0.0), sectors, random.Random(3), chaos=0.4)

        self.assertEqual(decision["action"], "turnright")
        self.assertEqual(decision["reason"], "front_blocked_open_side")
        self.assertGreaterEqual(decision["duration"], 0.76)
        self.assertLessEqual(decision["duration"], 1.24)


if __name__ == "__main__":
    unittest.main()
