"""Bounded capture-time local SLAM transform ring; never stores global map poses."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import atan2, cos, sin, isfinite

from .events import EventEnvelope, EventValidationError


@dataclass(frozen=True)
class MapTransform:
    boot_id: str
    map_epoch: int
    map_revision: int
    map_snapshot_id: str
    stamp_ros_ns: int
    x_m: float
    y_m: float
    yaw_rad: float
    valid: bool = True


@dataclass(frozen=True)
class ProjectedPose:
    event_id: str
    boot_id: str
    map_epoch: int
    map_revision: int
    map_snapshot_id: str
    x_m: float
    y_m: float
    yaw_rad: float


def project_capture(event: EventEnvelope, optimized: MapTransform, *,
                    max_age_ns: int = 2_000_000_000) -> ProjectedPose | None:
    """Project a captured *local* transform with validated current map->odom.

    It cannot repair non-rigid corrections or replace the bridge's revision
    provenance. Phase C must separately calibrate/map-check any visual route.
    """
    if (not event.spatial_ready or not optimized.valid or max_age_ns < 0
            or (event.boot_id, event.map_epoch, event.map_revision, event.map_snapshot_id)
            != (optimized.boot_id, optimized.map_epoch, optimized.map_revision,
                optimized.map_snapshot_id)
            or event.acquired_at_ros_ns is None
            or optimized.stamp_ros_ns < event.acquired_at_ros_ns
            or optimized.stamp_ros_ns - event.acquired_at_ros_ns > max_age_ns
            or not all(isfinite(v) for v in (optimized.x_m, optimized.y_m, optimized.yaw_rad))):
        return None
    local = event.capture_local_transform
    assert local is not None
    c, s = cos(optimized.yaw_rad), sin(optimized.yaw_rad)
    return ProjectedPose(event.event_id, event.boot_id, event.map_epoch,
                         optimized.map_revision, optimized.map_snapshot_id,
                         optimized.x_m + c * local["x_m"] - s * local["y_m"],
                         optimized.y_m + s * local["x_m"] + c * local["y_m"],
                         atan2(sin(optimized.yaw_rad + local["yaw_rad"]),
                               cos(optimized.yaw_rad + local["yaw_rad"])))


class PoseRing:
    def __init__(self, *, capacity: int = 64, max_gap_s: float = 0.05):
        if capacity < 2 or max_gap_s <= 0:
            raise ValueError("pose ring requires capacity >= 2 and positive gap")
        self.capacity = capacity
        self.max_gap_s = max_gap_s
        self._samples: deque[EventEnvelope] = deque(maxlen=capacity)

    def clear(self) -> None:
        self._samples.clear()

    def append(self, event: EventEnvelope) -> None:
        if not event.spatial_ready:
            # Unknown revision or TF invalidates interpolations across this sample.
            self._samples.clear()
            return
        if self._samples:
            last = self._samples[-1]
            if (event.boot_id, event.map_epoch) != (last.boot_id, last.map_epoch):
                self._samples.clear()
            elif event.acquired_at_robot <= last.acquired_at_robot:
                raise EventValidationError("pose sample time must increase")
        self._samples.append(event)

    def at(self, at_robot: float, *, boot_id: str, map_epoch: int,
           map_revision: int) -> EventEnvelope | None:
        samples = [e for e in self._samples if e.boot_id == boot_id and
                   e.map_epoch == map_epoch and e.map_revision == map_revision]
        for e in samples:
            if abs(e.acquired_at_robot - at_robot) < 1e-9:
                return e
        for left, right in zip(samples, samples[1:]):
            if not (left.acquired_at_robot < at_robot < right.acquired_at_robot):
                continue
            dt = right.acquired_at_robot - left.acquired_at_robot
            if dt > self.max_gap_s or left.map_revision != right.map_revision:
                return None
            ratio = (at_robot - left.acquired_at_robot) / dt
            a, b = left.capture_local_transform, right.capture_local_transform
            assert a is not None and b is not None
            assert left.acquired_at_ros_ns is not None and right.acquired_at_ros_ns is not None
            delta = atan2(sin(b["yaw_rad"] - a["yaw_rad"]), cos(b["yaw_rad"] - a["yaw_rad"]))
            data = left.to_dict()
            data["event_id"] = f"interpolated:{left.event_id}:{right.event_id}:{at_robot:.9f}"
            data["event_kind"] = "interpolated_transform"
            data["source"] = "system"
            data["evidence_id"] = None
            data["acquired_at_robot"] = at_robot
            data["received_at_robot"] = max(left.received_at_robot, right.received_at_robot, at_robot)
            data["acquired_at_ros_ns"] = round((1 - ratio) * left.acquired_at_ros_ns + ratio * right.acquired_at_ros_ns)
            pose = data["capture_local_transform"]
            for axis in ("x_m", "y_m"):
                pose[axis] = a[axis] * (1 - ratio) + b[axis] * ratio
            pose["yaw_rad"] = a["yaw_rad"] + ratio * delta
            pose["stamp_ros_ns"] = round((1 - ratio) * a["stamp_ros_ns"] + ratio * b["stamp_ros_ns"])
            pose["uncertainty"] = None  # Interpolation does not manufacture calibrated uncertainty.
            return EventEnvelope.from_dict(data)
        return None  # No extrapolation past a recorded transform.
