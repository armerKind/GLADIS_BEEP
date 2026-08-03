"""BEEP temporal perception and bounded-skill proposal package."""

from .moving_window import ContinuousMovingCapture, MovingFrameRing, MovingFrameSample, PANEL_COUNTS
from .policy import evaluate_proposal
from .schema import PerceptionValidationError, RESPONSE_JSON_SCHEMA, validate_perception_response

__all__ = [
    "ContinuousMovingCapture", "MovingFrameRing", "MovingFrameSample", "PANEL_COUNTS",
    "PerceptionValidationError", "RESPONSE_JSON_SCHEMA", "evaluate_proposal", "validate_perception_response",
]
