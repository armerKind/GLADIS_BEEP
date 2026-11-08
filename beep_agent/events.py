"""Phase A immutable acquisition envelope; separate from legacy session snapshots."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from copy import deepcopy
import json
import math
from typing import Any


class EventValidationError(ValueError):
    pass


def _number(value: Any, name: str, *, nonnegative: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise EventValidationError(f"{name} must be finite")
    if nonnegative and value < 0:
        raise EventValidationError(f"{name} must be nonnegative")
    return float(value)


def _integer(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise EventValidationError(f"{name} must be a nonnegative integer")
    return value


def _identity(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > 256:
        raise EventValidationError(f"{name} must be a nonempty identifier")
    return value


@dataclass(frozen=True)
class EventEnvelope:
    schema_version: int
    event_id: str
    event_kind: str
    source: str
    robot_id: str
    session_id: str
    boot_id: str
    mission_id: str | None
    lease_id: str | None
    acquired_at_robot: float
    received_at_robot: float
    robot_clock: str
    acquired_at_ros_ns: int | None
    map_epoch: int
    map_revision: int | None
    map_revision_status: str
    map_snapshot_id: str | None
    capture_local_transform: dict[str, Any] | None
    evidence_id: str | None
    payload: dict[str, Any]

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "EventEnvelope":
        if not isinstance(value, dict) or set(value) != set(cls.__dataclass_fields__):
            raise EventValidationError("event envelope fields differ from v1 contract")
        if value["schema_version"] != 1 or isinstance(value["schema_version"], bool):
            raise EventValidationError("unsupported event envelope schema")
        for field in ("event_id", "event_kind", "robot_id", "session_id", "boot_id"):
            _identity(value[field], field)
        if value["source"] not in {"camera", "audio", "lidar", "map", "nav_outcome", "system"}:
            raise EventValidationError("invalid event source")
        for field in ("mission_id", "lease_id", "evidence_id", "map_snapshot_id"):
            if value[field] is not None:
                _identity(value[field], field)
        _number(value["acquired_at_robot"], "acquired_at_robot", nonnegative=True)
        _number(value["received_at_robot"], "received_at_robot", nonnegative=True)
        if value["received_at_robot"] < value["acquired_at_robot"]:
            raise EventValidationError("received before acquired on robot clock")
        if value["robot_clock"] != "boot_monotonic":
            raise EventValidationError("robot clock must be boot_monotonic")
        if value["acquired_at_ros_ns"] is not None:
            _integer(value["acquired_at_ros_ns"], "acquired_at_ros_ns")
        _integer(value["map_epoch"], "map_epoch")
        if value["map_revision_status"] not in {"verified", "unknown"}:
            raise EventValidationError("invalid map revision status")
        if value["map_revision_status"] == "verified":
            _integer(value["map_revision"], "map_revision")
        elif value["map_revision"] is not None:
            raise EventValidationError("unknown map revision must be null")
        transform = value["capture_local_transform"]
        if transform is not None:
            expected = {"parent_frame", "child_frame", "stamp_ros_ns", "x_m", "y_m",
                        "yaw_rad", "source", "valid", "uncertainty"}
            if not isinstance(transform, dict) or set(transform) != expected:
                raise EventValidationError("invalid capture transform fields")
            if transform["parent_frame"] != "odom" or transform["child_frame"] != "base_link":
                raise EventValidationError("capture transform must be odom to base_link")
            _identity(transform["source"], "transform source")
            _integer(transform["stamp_ros_ns"], "transform stamp")
            if not isinstance(transform["valid"], bool):
                raise EventValidationError("transform valid must be bool")
            for field in ("x_m", "y_m", "yaw_rad"):
                _number(transform[field], field)
            uncertainty = transform["uncertainty"]
            if uncertainty is not None:
                if not isinstance(uncertainty, dict) or set(uncertainty) != {"xy_std_m", "yaw_std_rad"}:
                    raise EventValidationError("invalid transform uncertainty")
                for field in ("xy_std_m", "yaw_std_rad"):
                    _number(uncertainty[field], field, nonnegative=True)
            if value["acquired_at_ros_ns"] is None:
                raise EventValidationError("transform requires ROS acquisition stamp")
            if abs(transform["stamp_ros_ns"] - value["acquired_at_ros_ns"]) > 50_000_000:
                raise EventValidationError("transform is too far from acquisition time")
        if not isinstance(value["payload"], dict):
            raise EventValidationError("payload must be an object")
        try:
            encoded = json.dumps(value, allow_nan=False)
        except (TypeError, ValueError, OverflowError) as exc:
            raise EventValidationError("event is not finite JSON") from exc
        if len(encoded.encode("utf-8")) > 131_072:
            raise EventValidationError("event exceeds 128 KiB; store media by evidence ID")
        return cls(**deepcopy(value))

    @property
    def spatial_ready(self) -> bool:
        return (self.map_revision_status == "verified" and self.map_snapshot_id is not None
                and self.capture_local_transform is not None
                and self.capture_local_transform["valid"] is True
                and self.acquired_at_ros_ns is not None)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
