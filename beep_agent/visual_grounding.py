"""Phase C shadow-only visual viewpoint cones, not target points or motor commands.

A calibrated camera, hardware exposure clock, capture-time local TF and a
verified, exactly matching optimized map transform are all required. Saved
legacy scene labels are semantic hints, not pixel rays or metric positions.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import atan, atan2, cos, isfinite, pi, sin, tan

from .events import EventEnvelope
from .pose_history import MapTransform, project_capture


class GroundingError(ValueError):
    pass


def _finite(value: float, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value):
        raise GroundingError(f"{label} must be finite")
    return float(value)


def _id(value: str, label: str) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > 256:
        raise GroundingError(f"{label} must be a nonempty identifier")
    return value


def _angle(value: float) -> float:
    return atan2(sin(value), cos(value))


@dataclass(frozen=True)
class CameraCalibration:
    posture_id: str
    horizontal_fov_rad: float
    yaw_offset_rad: float
    offset_forward_m: float
    offset_left_m: float
    angular_error_rad: float

    def __post_init__(self) -> None:
        _id(self.posture_id, "posture_id")
        fov = _finite(self.horizontal_fov_rad, "horizontal_fov_rad")
        if not 0 < fov < pi:
            raise GroundingError("horizontal FOV must be between 0 and pi")
        for field in ("yaw_offset_rad", "offset_forward_m", "offset_left_m"):
            _finite(getattr(self, field), field)
        error = _finite(self.angular_error_rad, "angular_error_rad")
        if not 0 <= error < pi / 2:
            raise GroundingError("angular error must be bounded")
        if fov / 2 + error >= pi:
            raise GroundingError("camera uncertainty exceeds a direction")


@dataclass(frozen=True)
class Detection:
    target_id: str
    target_kind: str
    frame_event_id: str
    evidence_id: str
    confidence: float
    bbox_left: float
    bbox_right: float

    def __post_init__(self) -> None:
        for field in ("target_id", "frame_event_id", "evidence_id"):
            _id(getattr(self, field), field)
        if self.target_kind not in {"person", "object"}:
            raise GroundingError("unsupported target kind")
        confidence = _finite(self.confidence, "confidence")
        left = _finite(self.bbox_left, "bbox_left")
        right = _finite(self.bbox_right, "bbox_right")
        if not 0 <= confidence <= 1 or not 0 <= left < right <= 1:
            raise GroundingError("confidence or normalized bounding box out of range")


@dataclass(frozen=True)
class ViewpointCone:
    target_id: str
    target_kind: str
    event_id: str
    evidence_id: str
    session_id: str
    boot_id: str
    map_epoch: int
    map_revision: int
    map_snapshot_id: str
    acquired_at_robot: float
    origin_x_m: float
    origin_y_m: float
    bearing_center_rad: float
    bearing_half_width_rad: float
    confidence: float
    origin_xy_std_m: float | None
    slam_yaw_std_rad: float | None
    motor_authority: bool = field(default=False, init=False)


def ground_viewpoint(event: EventEnvelope, detection: Detection, optimized: MapTransform,
                     calibration: CameraCalibration) -> ViewpointCone | None:
    """Return a map-frame *viewing ray interval*, never a target point/distance.

    Pixel coordinates are left-to-right; map/base yaw is counter-clockwise.
    This pinhole horizontal-angle approximation needs a physically verified
    horizontal field of view, camera mount and current posture. It cannot
    account for unmeasured roll, distortion or target depth.
    """
    if (event.source != "camera" or event.event_kind != "frame_acquired"
            or event.timestamp_quality != "sensor_exposure" or not event.spatial_ready
            or event.payload.get("camera_posture_id") != calibration.posture_id
            or event.evidence_id != detection.evidence_id
            or event.event_id != detection.frame_event_id
            or detection.confidence < (0.7 if detection.target_kind == "person" else 0.65)):
        return None
    projected = project_capture(event, optimized, max_age_ns=10_000_000_000)
    if projected is None:
        return None
    assert event.evidence_id is not None
    c, s = cos(projected.yaw_rad), sin(projected.yaw_rad)
    # Compute each pixel edge through the calibrated pinhole model. The
    # angular midpoint is not the angle of the pixel midpoint off-axis.
    half_fov_tangent = tan(calibration.horizontal_fov_rad / 2)
    left = atan((1 - 2 * detection.bbox_left) * half_fov_tangent)
    right = atan((1 - 2 * detection.bbox_right) * half_fov_tangent)
    local = event.capture_local_transform
    assert local is not None
    measured = local["uncertainty"]
    slam_yaw_std = 0.0 if measured is None else measured["yaw_std_rad"]
    half_width = (left - right) / 2 + calibration.angular_error_rad + slam_yaw_std
    if half_width >= pi / 2:
        # A hemisphere-sized interval cannot guide even an advisory viewpoint.
        return None
    return ViewpointCone(
        detection.target_id, detection.target_kind, event.event_id, event.evidence_id,
        event.session_id, event.boot_id, projected.map_epoch, projected.map_revision,
        projected.map_snapshot_id, event.acquired_at_robot,
        projected.x_m + c * calibration.offset_forward_m - s * calibration.offset_left_m,
        projected.y_m + s * calibration.offset_forward_m + c * calibration.offset_left_m,
        _angle(projected.yaw_rad + calibration.yaw_offset_rad + (left + right) / 2),
        half_width, detection.confidence,
        None if measured is None else measured["xy_std_m"],
        None if measured is None else measured["yaw_std_rad"],
    )


@dataclass(frozen=True)
class ViewpointRequest:
    evidence_id: str
    target_id: str
    target_look_bearing_rad: float  # capture-origin map bearing, never current navigation heading
    boot_id: str
    map_epoch: int
    map_revision: int
    map_snapshot_id: str
    mission_id: str
    lease_id: str
    issued_at_robot: float
    expires_at_robot: float
    viewpoint_x_m: float
    viewpoint_y_m: float

    def __post_init__(self) -> None:
        for field in ("evidence_id", "target_id", "boot_id", "map_snapshot_id",
                      "mission_id", "lease_id"):
            _id(getattr(self, field), field)
        _finite(self.target_look_bearing_rad, "target_look_bearing_rad")
        _finite(self.viewpoint_x_m, "viewpoint_x_m")
        _finite(self.viewpoint_y_m, "viewpoint_y_m")
        issued = _finite(self.issued_at_robot, "issued_at_robot")
        expires = _finite(self.expires_at_robot, "expires_at_robot")
        if not 0 <= issued < expires <= issued + 2.0:
            raise GroundingError("directive lifetime must be positive and at most two seconds")
        for field in ("map_epoch", "map_revision"):
            number = getattr(self, field)
            if isinstance(number, bool) or not isinstance(number, int) or number < 0:
                raise GroundingError(f"{field} must be a nonnegative integer")


@dataclass(frozen=True)
class DirectiveValidation:
    accepted: bool
    reason: str
    motor_authority: bool = field(default=False, init=False)


def validate_viewpoint_request(request: ViewpointRequest, cone: ViewpointCone,
                               event: EventEnvelope, *, now_robot: float,
                               current_session_id: str, boot_id: str, map_epoch: int, map_revision: int,
                               map_snapshot_id: str, mission_id: str | None,
                               lease_id: str | None) -> DirectiveValidation:
    """Advisory admission only; local collision/route policy remains separate.

    The caller MUST pass *current* boot/map/owner identities, not echo the
    request. No object position, direction of motion, or clearance is granted.
    """
    now = _finite(now_robot, "now_robot")
    if (request.evidence_id, request.target_id) != (cone.evidence_id, cone.target_id) or (
            event.event_id, event.evidence_id) != (cone.event_id, cone.evidence_id) or (
            event.acquired_at_robot != cone.acquired_at_robot or not event.spatial_ready
            or event.source != "camera" or event.event_kind != "frame_acquired"):
        return DirectiveValidation(False, "evidence_mismatch")
    context = (boot_id, map_epoch, map_revision, map_snapshot_id)
    if (request.boot_id, request.map_epoch, request.map_revision, request.map_snapshot_id) != context or (
            cone.boot_id, cone.map_epoch, cone.map_revision, cone.map_snapshot_id) != context or (
            event.boot_id, event.map_epoch, event.map_revision, event.map_snapshot_id) != context:
        return DirectiveValidation(False, "map_context_mismatch")
    if event.session_id != current_session_id or cone.session_id != current_session_id:
        return DirectiveValidation(False, "session_mismatch")
    if (not mission_id or not lease_id
            or (request.mission_id, request.lease_id) != (mission_id, lease_id)
            or (event.mission_id, event.lease_id) not in ((mission_id, lease_id), (None, None))):
        return DirectiveValidation(False, "ownership_mismatch")
    if (now < event.acquired_at_robot or now - event.acquired_at_robot > 10.0
            or request.issued_at_robot < event.acquired_at_robot
            or now < request.issued_at_robot or now >= request.expires_at_robot):
        return DirectiveValidation(False, "expired")
    # The request bearing is capture-origin LOOK bearing; current-pose heading
    # toward the metric viewpoint is a separate value from the scorer.
    dx = request.viewpoint_x_m - cone.origin_x_m
    dy = request.viewpoint_y_m - cone.origin_y_m
    if (dx * dx + dy * dy < .01
            or abs(_angle(atan2(dy, dx) - request.target_look_bearing_rad)) > 1e-6
            or abs(_angle(request.target_look_bearing_rad - cone.bearing_center_rad)) > cone.bearing_half_width_rad):
        return DirectiveValidation(False, "outside_viewpoint_cone")
    return DirectiveValidation(True, "advisory_only")
