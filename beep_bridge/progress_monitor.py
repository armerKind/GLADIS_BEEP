"""Shadow locomotion discrepancy classification from corroborated windows.

No stop, fall certification, battery inference or motor dispatch lives here.
Thresholds are synthetic replay defaults; actual gait-cycle timing must be
measured on BEEP before any physical policy uses these observations.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, hypot, isfinite, sin


@dataclass(frozen=True)
class MotionWindow:
    boot_id: str
    map_epoch: int
    map_revision: int
    map_revision_verified: bool
    gait: str
    gait_cycles: int
    started_at_robot: float
    ended_at_robot: float
    start_x_m: float
    start_y_m: float
    end_x_m: float
    end_y_m: float
    start_yaw_rad: float
    pose_healthy: bool
    range_delta_m: float | None
    visual_motion: str
    attitude_alert: bool | None

    def __post_init__(self) -> None:
        if (not isinstance(self.boot_id, str) or not self.boot_id
                or type(self.map_epoch) is not int or self.map_epoch < 0
                or type(self.map_revision) is not int or self.map_revision < 0
                or type(self.map_revision_verified) is not bool
                or self.gait not in {"straight", "turn", "transition", "unknown"}
                or type(self.gait_cycles) is not int or self.gait_cycles < 0
                or type(self.pose_healthy) is not bool
                or self.visual_motion not in {"moving", "still", "unknown"}
                or (self.attitude_alert is not None and type(self.attitude_alert) is not bool)):
            raise ValueError("invalid motion window identity or classification")
        values = (self.started_at_robot, self.ended_at_robot, self.start_x_m,
                  self.start_y_m, self.end_x_m, self.end_y_m, self.start_yaw_rad)
        if (any(isinstance(v, bool) or not isinstance(v, (float, int))
                or not isfinite(v) for v in values)
                or self.started_at_robot < 0 or self.ended_at_robot <= self.started_at_robot
                or (self.range_delta_m is not None and (isinstance(self.range_delta_m, bool)
                or not isinstance(self.range_delta_m, (float, int))
                or not isfinite(self.range_delta_m)))):
            raise ValueError("invalid motion window measurements")


@dataclass(frozen=True)
class MotionAssessment:
    state: str
    signals: tuple[str, ...]
    motor_authority: bool = field(default=False, init=False)


class MotionDiscrepancy:
    """Classify two consecutive eligible stalls; no direct control side effect."""

    def __init__(self, *, min_gait_cycles: int):
        if type(min_gait_cycles) is not int or min_gait_cycles < 2:
            raise ValueError("gait-cycle threshold requires physical calibration")
        self.min_gait_cycles = min_gait_cycles
        self._identity: tuple[str, int, int] | None = None
        self._last_end: float | None = None
        self._consecutive_stalls = 0

    def observe(self, window: MotionWindow) -> MotionAssessment:
        if not isinstance(window, MotionWindow):
            raise TypeError("expected validated MotionWindow")
        identity = (window.boot_id, window.map_epoch, window.map_revision)
        contiguous = (self._identity == identity and self._last_end is not None
                      and window.started_at_robot >= self._last_end
                      and window.started_at_robot - self._last_end <= 1.0)
        if not contiguous:
            self._consecutive_stalls = 0
        self._identity, self._last_end = identity, window.ended_at_robot
        signals = (("same_epoch",) if contiguous
                   else ("identity_or_window_reset",))
        if window.attitude_alert is True:
            self._consecutive_stalls = 0
            return MotionAssessment("possible_fall", signals + ("external_attitude_alert",))
        if not window.map_revision_verified:
            self._consecutive_stalls = 0
            self._identity, self._last_end = None, None
            return MotionAssessment("insufficient_evidence", signals + ("unverified_map_revision",))
        signals += ("revision_verified",)
        if (window.gait != "straight" or window.gait_cycles < self.min_gait_cycles
                or not window.pose_healthy or window.range_delta_m is None
                or window.visual_motion == "unknown"):
            self._consecutive_stalls = 0
            return MotionAssessment("insufficient_evidence", signals + ("unqualified_window",))
        dx, dy = window.end_x_m - window.start_x_m, window.end_y_m - window.start_y_m
        forward = dx * cos(window.start_yaw_rad) + dy * sin(window.start_yaw_rad)
        displacement = hypot(dx, dy)
        if displacement >= .5 and abs(window.range_delta_m) < .05 and window.visual_motion == "still":
            self._consecutive_stalls = 0
            return MotionAssessment("localization_conflict", signals + ("pose_range_vision_disagree",))
        if forward >= .15 and (window.range_delta_m >= .1 or window.visual_motion == "moving"):
            self._consecutive_stalls = 0
            return MotionAssessment("progress_ok", signals + ("correlated_forward_progress",))
        if displacement < .05 and abs(window.range_delta_m) < .05 and window.visual_motion == "still":
            self._consecutive_stalls += 1
            if self._consecutive_stalls >= 2:
                return MotionAssessment("stalled", signals + ("two_correlated_stall_windows",))
            return MotionAssessment("insufficient_evidence", signals + ("one_stall_window",))
        self._consecutive_stalls = 0
        return MotionAssessment("insufficient_evidence", signals + ("conflicting_or_weak_signals",))
