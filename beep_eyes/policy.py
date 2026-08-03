"""Deterministic policy gate for BEEP perception proposals.

The policy evaluates eligibility only. It does not dispatch hardware actions.
"""
from __future__ import annotations

import math
from typing import Any

from .schema import validate_perception_response

MIN_MODEL_CONFIDENCE = 0.65
MAX_SCAN_AGE_S = 0.60
HARD_CLEARANCE_M = 0.15
PHYSICAL_SKILLS = {"orient", "advance", "retreat", "explore", "gesture"}
MOVEMENT_SKILLS = {"orient", "advance", "retreat", "explore"}


def _sensor_gate(status: dict[str, Any]) -> tuple[bool, str]:
    if bool(status.get("moving")) or status.get("motion_lease_id") is not None:
        return False, "bridge_busy"
    scan_age = status.get("scan_age_s")
    if isinstance(scan_age, bool) or not isinstance(scan_age, (int, float)) or not math.isfinite(float(scan_age)):
        return False, "scan_missing"
    if float(scan_age) > MAX_SCAN_AGE_S:
        return False, "scan_stale"
    sectors = status.get("sectors") or {}
    for name in ("front", "front_left", "front_right", "left", "right", "rear"):
        value = sectors.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            return False, f"sector_missing:{name}"
        if float(value) < HARD_CLEARANCE_M:
            return False, f"clearance:{name}"
    return True, "ok"


def evaluate_proposal(response: dict[str, Any], status: dict[str, Any], mode: str = "shadow") -> dict[str, Any]:
    """Evaluate model steps under deterministic state and rollout rules.

    Modes:
      shadow: no step is eligible for dispatch.
      stationary: observe/speak/stop and stationary gestures may be eligible;
                  movement remains prohibited.
      supervised: bounded movement may be eligible, but this module still does
                  not execute it.
    """
    response = validate_perception_response(response)
    if mode not in {"shadow", "stationary", "supervised"}:
        raise ValueError("mode must be shadow, stationary, or supervised")
    confidence = float(response["confidence"])
    hazards = {item["type"] for item in response["scene"]["hazards"] if float(item["confidence"]) >= 0.50}
    sensor_ok, sensor_reason = _sensor_gate(status)
    evaluations = []
    for index, step in enumerate(response["plan"]["steps"]):
        skill = step["skill"]
        eligible = True
        reason = "eligible"
        if mode == "shadow":
            eligible, reason = False, "shadow_mode"
        elif response["escalate"]:
            eligible, reason = False, "model_escalation"
        elif confidence < MIN_MODEL_CONFIDENCE:
            eligible, reason = False, "low_confidence"
        elif skill == "observe":
            eligible, reason = True, "no_physical_action"
        elif skill == "stop":
            eligible, reason = True, "stop_is_fail_safe"
        elif skill == "speak":
            if bool(status.get("moving")) or status.get("motion_lease_id") is not None:
                eligible, reason = False, "speech_requires_stationary"
        elif skill in PHYSICAL_SKILLS and not sensor_ok:
            eligible, reason = False, sensor_reason
        elif skill in MOVEMENT_SKILLS and mode != "supervised":
            eligible, reason = False, "movement_not_enabled"
        elif skill in MOVEMENT_SKILLS and hazards & {"glass", "stairs", "drop", "person", "animal", "vehicle"}:
            eligible, reason = False, "visual_hazard"
        elif skill in MOVEMENT_SKILLS and not bool((status.get("slam") or {}).get("usable")):
            eligible, reason = False, "slam_unusable"
        evaluations.append({"index": index, "skill": skill, "eligible": eligible, "reason": reason, "step": step})
    return {
        "mode": mode,
        "dispatch_performed": False,
        "model_confidence": confidence,
        "model_escalated": response["escalate"],
        "sensor_gate": {"ok": sensor_ok, "reason": sensor_reason},
        "visual_hazards": sorted(hazards),
        "steps": evaluations,
    }
