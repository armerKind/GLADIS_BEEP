"""BEEP temporal perception package: moving-frame capture, contact sheets, typed scene schema."""

from .moving_window import ContinuousMovingCapture, MovingFrameRing, MovingFrameSample, PANEL_COUNTS
from .schema import PerceptionValidationError, RESPONSE_JSON_SCHEMA, validate_perception_response

__all__ = [
    "ContinuousMovingCapture", "MovingFrameRing", "MovingFrameSample", "PANEL_COUNTS",
    "PerceptionValidationError", "RESPONSE_JSON_SCHEMA", "validate_perception_response",
]
