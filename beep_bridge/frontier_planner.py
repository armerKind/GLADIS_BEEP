#!/usr/bin/env python3
"""Frontier planning and deliberately non-appliance-like motion policy for BEEP."""
from __future__ import annotations

import heapq
import math
from collections import deque

TURN_DURATION_SCALE = 2.0


class OccupancyGrid:
    def __init__(self, width, height, resolution, origin_x, origin_y, data):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.data = tuple(int(v) for v in data)
        if len(self.data) != self.width * self.height:
            raise ValueError("occupancy data size does not match width*height")

    def inside(self, x, y):
        return 0 <= int(x) < self.width and 0 <= int(y) < self.height

    def value(self, x, y):
        if not self.inside(x, y):
            return 100
        return self.data[int(y) * self.width + int(x)]

    def world_to_cell(self, x, y):
        return (int(math.floor((float(x) - self.origin_x) / self.resolution + 1e-9)),
                int(math.floor((float(y) - self.origin_y) / self.resolution + 1e-9)))

    def cell_to_world(self, cell):
        x, y = cell
        return (self.origin_x + (x + 0.5) * self.resolution,
                self.origin_y + (y + 0.5) * self.resolution)


_NEIGHBORS_4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
_NEIGHBORS_8 = _NEIGHBORS_4 + ((1, 1), (1, -1), (-1, 1), (-1, -1))


def norm_angle(angle):
    while angle <= -math.pi:
        angle += 2 * math.pi
    while angle > math.pi:
        angle -= 2 * math.pi
    return angle


def inflate_obstacles(grid, radius_cells=4, occupied_threshold=50):
    radius_cells = max(0, int(radius_cells))
    occupied = []
    for y in range(grid.height):
        for x in range(grid.width):
            if grid.value(x, y) >= occupied_threshold:
                occupied.append((x, y))
    blocked = set(occupied)
    if not radius_cells:
        return blocked
    r2 = radius_cells * radius_cells
    offsets = [(dx, dy) for dy in range(-radius_cells, radius_cells + 1)
               for dx in range(-radius_cells, radius_cells + 1)
               if dx * dx + dy * dy <= r2]
    for ox, oy in occupied:
        for dx, dy in offsets:
            cell = (ox + dx, oy + dy)
            if grid.inside(*cell):
                blocked.add(cell)
    return blocked


def frontier_cells(grid, blocked=None):
    blocked = blocked or set()
    result = set()
    for y in range(grid.height):
        for x in range(grid.width):
            if (x, y) in blocked or not (0 <= grid.value(x, y) < 50):
                continue
            if any(grid.inside(x + dx, y + dy) and grid.value(x + dx, y + dy) < 0
                   for dx, dy in _NEIGHBORS_4):
                result.add((x, y))
    return result


def cluster_frontiers(cells, minimum_size=3):
    remaining = set(cells)
    clusters = []
    while remaining:
        start = remaining.pop()
        cluster = [start]
        queue = deque([start])
        while queue:
            x, y = queue.popleft()
            for dx, dy in _NEIGHBORS_8:
                nxt = (x + dx, y + dy)
                if nxt in remaining:
                    remaining.remove(nxt)
                    cluster.append(nxt)
                    queue.append(nxt)
        if len(cluster) >= int(minimum_size):
            clusters.append(cluster)
    return clusters


def nearest_free_cell(grid, start, blocked, max_radius=8):
    if grid.inside(*start) and 0 <= grid.value(*start) < 50 and start not in blocked:
        return start
    sx, sy = start
    for radius in range(1, max_radius + 1):
        candidates = []
        for y in range(sy - radius, sy + radius + 1):
            for x in range(sx - radius, sx + radius + 1):
                if not grid.inside(x, y) or (x, y) in blocked or not (0 <= grid.value(x, y) < 50):
                    continue
                candidates.append((math.hypot(x - sx, y - sy), (x, y)))
        if candidates:
            return min(candidates)[1]
    return None


def astar_path(grid, start, goal, blocked=None):
    blocked = set(blocked or ())
    blocked.discard(start)
    blocked.discard(goal)
    if not grid.inside(*start) or not grid.inside(*goal):
        return None
    if not (0 <= grid.value(*goal) < 50):
        return None
    frontier = [(0.0, start)]
    came_from = {start: None}
    cost = {start: 0.0}
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            return list(reversed(path))
        x, y = current
        for dx, dy in _NEIGHBORS_8:
            nxt = (x + dx, y + dy)
            if not grid.inside(*nxt) or nxt in blocked or not (0 <= grid.value(*nxt) < 50):
                continue
            if dx and dy:
                # Do not cut diagonally through obstacle corners.
                if (x + dx, y) in blocked or (x, y + dy) in blocked:
                    continue
            step = 1.41421356237 if dx and dy else 1.0
            new_cost = cost[current] + step
            if new_cost >= cost.get(nxt, float("inf")):
                continue
            cost[nxt] = new_cost
            came_from[nxt] = current
            heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
            heapq.heappush(frontier, (new_cost + heuristic, nxt))
    return None


def _cluster_representative(cluster):
    cx = sum(x for x, _ in cluster) / len(cluster)
    cy = sum(y for _, y in cluster) / len(cluster)
    return min(cluster, key=lambda cell: (cell[0] - cx) ** 2 + (cell[1] - cy) ** 2)


def find_frontier_plan(grid, pose, rng, chaos=0.35, robot_radius_m=0.16, excluded=None):
    chaos = min(max(float(chaos), 0.0), 1.0)
    excluded = list(excluded or ())
    radius_cells = max(1, int(math.ceil(float(robot_radius_m) / grid.resolution)))
    blocked = inflate_obstacles(grid, radius_cells)
    raw_frontiers = frontier_cells(grid, blocked)
    clusters = cluster_frontiers(raw_frontiers, minimum_size=3)
    requested_start = grid.world_to_cell(pose[0], pose[1])
    start = nearest_free_cell(grid, requested_start, blocked)
    if start is None:
        return None

    candidates = []
    for cluster in clusters:
        target = _cluster_representative(cluster)
        if any(math.hypot(target[0] - ex[0], target[1] - ex[1]) < 4 for ex in excluded):
            continue
        path = astar_path(grid, start, target, blocked)
        if not path:
            continue
        wx, wy = grid.cell_to_world(target)
        heading = math.atan2(wy - pose[1], wx - pose[0])
        heading_bonus = math.cos(norm_angle(heading - pose[2]))
        information_gain = math.log1p(len(cluster))
        distance_cost = len(path) * grid.resolution
        noise = rng.uniform(-1.0, 1.0) * chaos * 1.2
        score = 2.2 * information_gain - 0.65 * distance_cost + 0.45 * heading_bonus + noise
        candidates.append({
            "target_cell": target,
            "target_world": (round(wx, 4), round(wy, 4)),
            "path": path,
            "cluster_size": len(cluster),
            "score": score,
        })
    if not candidates:
        return None

    candidates.sort(key=lambda item: item["score"], reverse=True)
    shortlist = candidates[:min(3, len(candidates))]
    # Weighted selection among good frontiers prevents deterministic vacuum-cleaner geometry
    # while still ensuring every choice leads toward actual unknown space.
    top = shortlist[0]["score"]
    weights = [math.exp((item["score"] - top) * (1.2 - 0.7 * chaos)) for item in shortlist]
    selected = rng.choices(shortlist, weights=weights, k=1)[0]
    selected = dict(selected)
    selected["frontier_clusters"] = len(clusters)
    selected["reachable_frontiers"] = len(candidates)
    selected["start_cell"] = start
    return selected


def choose_natural_motion(pose, waypoint_world, sectors, rng, chaos=0.35):
    chaos = min(max(float(chaos), 0.0), 1.0)
    x, y, yaw = map(float, pose)
    wx, wy = map(float, waypoint_world)
    desired = math.atan2(wy - y, wx - x)
    heading_error = norm_angle(desired - yaw)
    front = float(sectors.get("front") or 0.0)
    front_left = float(sectors.get("front_left") or 0.0)
    front_right = float(sectors.get("front_right") or 0.0)
    left = float(sectors.get("left") or 0.0)
    right = float(sectors.get("right") or 0.0)

    if front < 0.40 or min(front_left, front_right) < 0.27:
        action = "turnright" if max(right, front_right) > max(left, front_left) else "turnleft"
        return {"action": action, "step": 30, "duration": round(rng.uniform(0.38, 0.62) * TURN_DURATION_SCALE, 2),
                "reason": "front_blocked_open_side", "heading_error": round(heading_error, 4)}

    turn_threshold = 0.23 + chaos * 0.08
    if abs(heading_error) > turn_threshold:
        action = "turnleft" if heading_error > 0 else "turnright"
        duration = min(0.78, max(0.25, abs(heading_error) / 1.5)) * TURN_DURATION_SCALE
        duration *= rng.uniform(0.88, 1.12)
        return {"action": action, "step": 30, "duration": round(duration, 2),
                "reason": "align_to_frontier_path", "heading_error": round(heading_error, 4)}

    # Occasionally inspect an inviting side opening. It is still bounded and the next cycle
    # replans toward a frontier, so this resembles curiosity rather than navigational collapse.
    if chaos > 0.15 and rng.random() < 0.08 + chaos * 0.10 and max(left, right) > 0.95:
        action = "turnleft" if left > right else "turnright"
        return {"action": action, "step": 30, "duration": round(rng.uniform(0.24, 0.38) * TURN_DURATION_SCALE, 2),
                "reason": "curiosity_glance", "heading_error": round(heading_error, 4)}

    duration = rng.uniform(0.68, 0.92 + chaos * 0.28)
    return {"action": "forward", "step": 20, "duration": round(duration, 2),
            "reason": "follow_frontier_path", "heading_error": round(heading_error, 4)}
