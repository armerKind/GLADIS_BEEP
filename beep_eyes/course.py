"""Camera-informed, sensor-gated course recommendations; never dispatches motion.

Vision supplies semantic intent and hazards. Fresh local range sectors and usable
SLAM gate a recommendation. Neither source alone establishes traversability.
"""
from __future__ import annotations

import math
from typing import Any

from .schema import validate_perception_response

MIN_CONFIDENCE = 0.70
MIN_FRONT_M = 0.55
MIN_DIAGONAL_M = 0.42
MIN_SIDE_M = 0.35
MAX_SCAN_AGE_S = 0.60
BLOCKING_HAZARDS = {"glass", "stairs", "drop", "person", "animal", "vehicle", "obstacle", "unknown"}
DYNAMIC_HAZARDS = {"person", "animal", "vehicle"}


def _ranges(status: dict[str, Any]) -> tuple[dict[str, float] | None, str]:
    if status.get("moving") and status.get("motion_lease_id") is None:
        return None, "unowned_motion"
    age = status.get("scan_age_s")
    if (isinstance(age, bool) or not isinstance(age, (int, float))
            or not math.isfinite(float(age)) or not 0.0 <= float(age) <= MAX_SCAN_AGE_S):
        return None, "scan_missing_or_stale"
    slam = status.get("slam")
    if not isinstance(slam, dict) or slam.get("usable") is not True:
        return None, "slam_unusable"
    sectors = status.get("sectors")
    if not isinstance(sectors, dict):
        return None, "sectors_missing"
    values = {}
    for name in ("front", "front_left", "front_right", "left", "right", "rear"):
        raw = sectors.get(name)
        if isinstance(raw, bool) or not isinstance(raw, (int, float)) or not math.isfinite(float(raw)):
            return None, "sector_missing:" + name
        values[name] = float(raw)
    return values, "ok"


def recommend_course(response: dict[str, Any], status: dict[str, Any], *, mission_valid: bool = True) -> dict[str, Any]:
    """Produce a bounded advisory from one validated panel response and live state.

    Deliberately never returns a motor command, duration, or dispatch permission.
    """
    response = validate_perception_response(response)
    confidence = float(response["confidence"])
    hazards = [h for h in response["scene"]["hazards"]
               if float(h["confidence"]) >= 0.5 and h["type"] in BLOCKING_HAZARDS]
    sensor, reason = _ranges(status)
    proposal = next((step for step in response["plan"]["steps"]
                     if step.get("skill") in {"advance", "retreat", "orient", "explore", "stop"}), None)
    result = {"recommendation": "hold_and_reobserve", "reason": "no_bounded_visual_motion_intent",
              "visual_intent": None if proposal is None else proposal.get("skill"),
              "visual_hazards": [h.get("type") for h in hazards],
              "sensor_gate": {"ok": sensor is not None, "reason": reason},
              "mission_valid": bool(mission_valid),
              "dispatch_performed": False}
    if not mission_valid:
        result["reason"] = "mission_identity_or_state_changed"
        return result
    if response.get("escalate") or confidence < MIN_CONFIDENCE:
        result["reason"] = "visual_confidence_insufficient"
        return result
    if sensor is None:
        result["reason"] = reason
        return result
    if proposal is not None and proposal["skill"] == "stop":
        result.update(recommendation="stop_and_reassess", reason="panel_proposed_stop")
        return result
    dynamic_center = [h for h in hazards
                      if h.get("type") in DYNAMIC_HAZARDS and h.get("position") in {"center", "unknown"}]
    if dynamic_center:
        blocked_sides = {h.get("position") for h in hazards if h.get("type") not in DYNAMIC_HAZARDS}
        candidates = [side for side in ("left", "right")
                      if side not in blocked_sides and sensor[side] >= MIN_SIDE_M
                      and sensor["front_" + side] >= MIN_DIAGONAL_M]
        if candidates:
            direction = max(candidates, key=lambda side: min(sensor[side], sensor["front_" + side]))
            result.update(recommendation="replan_around_dynamic_hazard",
                          reason="yield_to_shared_space_traffic_via_clear_side",
                          direction=direction)
        else:
            result["reason"] = "dynamic_hazard_no_clear_side_route"
        return result
    if proposal is None:
        return result
    if any(h.get("type") in {"stairs", "drop", "unknown"}
           and h.get("position") in {"center", "unknown"} for h in hazards):
        result["reason"] = "central_or_unlocalized_visual_hazard"
        return result
    if proposal["skill"] == "advance":
        if any(h.get("position") in {"center", "unknown"} for h in hazards):
            result["reason"] = "central_visual_hazard"
            return result
        clear = (sensor["front"] >= MIN_FRONT_M and sensor["front_left"] >= MIN_DIAGONAL_M
                 and sensor["front_right"] >= MIN_DIAGONAL_M)
        result.update(recommendation="forward_candidate" if clear else "hold_and_reobserve",
                      reason="panel_proposed_advance_and_range_gate_passed" if clear else "forward_range_gate_failed")
        return result
    if proposal["skill"] == "retreat":
        clear = sensor["rear"] >= MIN_FRONT_M and sensor["left"] >= MIN_SIDE_M and sensor["right"] >= MIN_SIDE_M
        result.update(recommendation="reverse_candidate" if clear else "hold_and_reobserve",
                      reason="panel_proposed_retreat_and_rear_gate_passed" if clear else "reverse_range_gate_failed")
        return result
    if proposal["skill"] == "orient":
        direction = proposal["direction"]
        side, diagonal = sensor[direction], sensor["front_" + direction]
        visual_block = any(h.get("position") == direction for h in hazards)
        clear = side >= MIN_SIDE_M and diagonal >= MIN_DIAGONAL_M and not visual_block
        result.update(recommendation="look_" + direction if clear else "hold_and_reobserve",
                      reason="panel_requested_view_change_and_range_gate_passed" if clear else "view_direction_blocked_or_uncertain")
        return result
    # An open-ended explore duration is not translated into locomotion by this layer.
    result["reason"] = "explore_requires_onboard_bounded_mission_controller"
    return result
