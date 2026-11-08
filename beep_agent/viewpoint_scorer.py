"""Shadow-only known-free inspection viewpoints from capture-anchored ray wedges.

This scores a possible *viewpoint*, not the object position, a path, or a motor
command. The caller supplies the current, independently verified map context.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, cos, floor, hypot, isfinite, pi, sin

from .place_graph import GridGeometry
from .visual_grounding import ViewpointCone


@dataclass(frozen=True)
class OccupancySnapshot:
    boot_id: str
    map_epoch: int
    map_revision: int
    map_snapshot_id: str
    geometry: GridGeometry
    width: int
    height: int
    data: tuple[int, ...]

    def __post_init__(self) -> None:
        if (not isinstance(self.boot_id, str) or not self.boot_id
                or not isinstance(self.map_snapshot_id, str) or not self.map_snapshot_id
                or type(self.map_epoch) is not int or self.map_epoch < 0
                or type(self.map_revision) is not int or self.map_revision < 0
                or not isinstance(self.geometry, GridGeometry)
                or not .02 <= self.geometry.resolution_m <= .5
                or type(self.width) is not int or type(self.height) is not int
                or self.width <= 0 or self.height <= 0
                or not 0 < self.width * self.height <= 1_000_000
                or type(self.data) is not tuple or len(self.data) != self.width * self.height
                or any(type(v) is not int or not -1 <= v <= 100 for v in self.data)):
            raise ValueError("invalid bounded occupancy snapshot")

    @property
    def identity(self) -> tuple[str, int, int, str]:
        return (self.boot_id, self.map_epoch, self.map_revision, self.map_snapshot_id)

    def metric_to_cell(self, x: float, y: float) -> tuple[int, int]:
        dx, dy = x - self.geometry.origin_x_m, y - self.geometry.origin_y_m
        c, s = cos(self.geometry.origin_yaw_rad), sin(self.geometry.origin_yaw_rad)
        return (floor((c * dx + s * dy) / self.geometry.resolution_m),
                floor((-s * dx + c * dy) / self.geometry.resolution_m))

    def cell(self, gx: int, gy: int) -> int:
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return self.data[gy * self.width + gx]
        return -1

    def free_at(self, x: float, y: float) -> bool:
        return self.cell(*self.metric_to_cell(x, y)) == 0

    def footprint_free(self, x: float, y: float, radius: float) -> bool:
        gx, gy = self.metric_to_cell(x, y)
        reach = int(radius / self.geometry.resolution_m) + 2
        c, s = cos(self.geometry.origin_yaw_rad), sin(self.geometry.origin_yaw_rad)
        dx, dy = x - self.geometry.origin_x_m, y - self.geometry.origin_y_m
        lx = (c * dx + s * dy) / self.geometry.resolution_m
        ly = (-s * dx + c * dy) / self.geometry.resolution_m
        for iy in range(gy - reach, gy + reach + 1):
            for ix in range(gx - reach, gx + reach + 1):
                # Nearest point on this rotated grid cell to the proposed center.
                if hypot(max(ix - lx, 0, lx - ix - 1),
                         max(iy - ly, 0, ly - iy - 1)) * self.geometry.resolution_m <= radius:
                    if self.cell(ix, iy) != 0:
                        return False
        return True

    def first_wedge_block_m(self, cone: ViewpointCone, distance: float) -> float:
        """Nearest conservative wedge blockage, scanning the bounded grid once.

        Each cell is a square area, not a point on the center ray. Its
        circumscribed disk conservatively includes edge-intersecting cells.
        Return infinity if no unknown/occupied cell intersects the wedge.
        """
        res = self.geometry.resolution_m
        half_diag = res * 2**0.5 / 2
        gx, gy = self.metric_to_cell(cone.origin_x_m, cone.origin_y_m)
        reach = int((distance + half_diag) / res) + 2
        c, s = cos(self.geometry.origin_yaw_rad), sin(self.geometry.origin_yaw_rad)
        first_block = float("inf")
        for iy in range(gy - reach, gy + reach + 1):
            for ix in range(gx - reach, gx + reach + 1):
                if self.cell(ix, iy) == 0:
                    continue
                dx = (ix + .5) * res
                dy = (iy + .5) * res
                x = self.geometry.origin_x_m + c * dx - s * dy
                y = self.geometry.origin_y_m + s * dx + c * dy
                vx, vy = x - cone.origin_x_m, y - cone.origin_y_m
                r = hypot(vx, vy)
                if r > distance + half_diag:
                    continue
                delta = atan2(sin(atan2(vy, vx) - cone.bearing_center_rad),
                              cos(atan2(vy, vx) - cone.bearing_center_rad))
                # A cell whose circumscribed disk reaches the capture origin
                # intersects every possible ray direction; do not drop a
                # nearby unknown just because its center lies behind us.
                angular_margin = (pi if r <= half_diag + 1e-9
                                  else atan2(half_diag, r - half_diag))
                if abs(delta) <= cone.bearing_half_width_rad + angular_margin:
                    first_block = min(first_block, max(0.0, r - half_diag))
        return first_block


@dataclass(frozen=True)
class ViewpointCandidate:
    x_m: float
    y_m: float
    heading_from_current_rad: float
    target_look_bearing_rad: float  # map-frame bearing from exposure origin, NOT navigation heading
    evidence_id: str
    boot_id: str
    map_epoch: int
    map_revision: int
    map_snapshot_id: str
    motor_authority: bool = field(default=False, init=False)


def score_viewpoint(cone: ViewpointCone, grid: OccupancySnapshot, *,
                    current_map_context: tuple[str, int, int, str],
                    current_x_m: float, current_y_m: float, footprint_radius_m: float,
                    min_look_m: float = .7, max_look_m: float = 1.5) -> ViewpointCandidate | None:
    """Score a bounded known-free viewpoint, then compute heading from NOW.

    A candidate is not a validated route. Any later route requires fresh
    current pose, local sensors and a footprint-inflated connectivity check.
    """
    geometry_values = (current_x_m, current_y_m, footprint_radius_m, min_look_m, max_look_m)
    if (any(isinstance(v, bool) or not isinstance(v, (int, float))
            or not isfinite(v) for v in geometry_values)
            or not 0 < footprint_radius_m <= 1
            or not .1 <= min_look_m <= max_look_m <= 3):
        raise ValueError("invalid viewpoint geometry")
    if ((cone.boot_id, cone.map_epoch, cone.map_revision, cone.map_snapshot_id) != grid.identity
            or current_map_context != grid.identity
            or cone.origin_xy_std_m is None or cone.slam_yaw_std_rad is None
            or not all(isfinite(v) for v in (cone.origin_x_m, cone.origin_y_m,
                 cone.bearing_center_rad, cone.bearing_half_width_rad, cone.origin_xy_std_m,
                 cone.slam_yaw_std_rad))
            or not 0 <= cone.origin_xy_std_m <= .2
            or not 0 <= cone.slam_yaw_std_rad <= .3
            or not 0 <= cone.bearing_half_width_rad < pi / 2):
        return None
    if not grid.footprint_free(current_x_m, current_y_m, footprint_radius_m):
        return None
    first_block_m = grid.first_wedge_block_m(cone, max_look_m)
    radius = footprint_radius_m + 2 * cone.origin_xy_std_m
    ray_x, ray_y = cos(cone.bearing_center_rad), sin(cone.bearing_center_rad)
    current_projection = ((current_x_m - cone.origin_x_m) * ray_x
                          + (current_y_m - cone.origin_y_m) * ray_y)
    step = grid.geometry.resolution_m / 2
    n = int((max_look_m - min_look_m) / step) + 1
    for index in range(n):
        distance = min_look_m + index * step
        if distance + radius >= first_block_m:
            return None  # Full uncertainty-inflated body stays before unknown.
        if distance <= current_projection + .05:
            continue  # Do not send a translated observer back down its old ray.
        x = cone.origin_x_m + distance * ray_x
        y = cone.origin_y_m + distance * ray_y
        if grid.footprint_free(x, y, radius):
            if hypot(x - current_x_m, y - current_y_m) < .05:
                continue
            return ViewpointCandidate(x, y, atan2(y - current_y_m, x - current_x_m),
                                      cone.bearing_center_rad, cone.evidence_id, cone.boot_id,
                                      cone.map_epoch,
                                      cone.map_revision, cone.map_snapshot_id)
    return None
