"""Typed semantic functions available to the embodied executive.

These are intentions, not motor commands. A separate deterministic adapter must
validate current sensors, acquire a lease, and execute one bounded skill.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


class SkillValidationError(ValueError):
    pass


FUNCTION_SCHEMAS: list[dict[str, Any]] = [
    {"name": "observe", "description": "Remain stationary and acquire a fresh temporal perception window.", "parameters": {"type": "object", "required": ["reason"], "properties": {"reason": {"type": "string"}, "target_id": {"type": "string"}}}},
    {"name": "orient", "description": "Request a bounded relative body turn toward a tracked target.", "parameters": {"type": "object", "required": ["target_id", "direction", "degrees", "reason"], "properties": {"target_id": {"type": "string"}, "direction": {"enum": ["left", "right"]}, "degrees": {"type": "number", "minimum": 1, "maximum": 20}, "reason": {"type": "string"}}}},
    {"name": "approach_target", "description": "Request one short guarded approach segment, then stop and re-perceive.", "parameters": {"type": "object", "required": ["target_id", "max_step_m", "stop_distance_m", "reason"], "properties": {"target_id": {"type": "string"}, "max_step_m": {"type": "number", "minimum": 0.05, "maximum": 0.20}, "stop_distance_m": {"type": "number", "minimum": 0.25, "maximum": 0.80}, "reason": {"type": "string"}}}},
    {"name": "inspect_target", "description": "Acquire another viewpoint to reveal object faces and outward corners.", "parameters": {"type": "object", "required": ["target_id", "strategy", "max_turn_deg", "reason"], "properties": {"target_id": {"type": "string"}, "strategy": {"enum": ["left_view", "right_view", "alternate"]}, "max_turn_deg": {"type": "number", "minimum": 5, "maximum": 20}, "reason": {"type": "string"}}}},
    {"name": "mark_target", "description": "Request deterministic approach, sideways alignment, and marking gesture at a visible outward corner.", "parameters": {"type": "object", "required": ["target_id", "feature", "dry_run", "reason"], "properties": {"target_id": {"type": "string"}, "feature": {"enum": ["left_outward_corner", "right_outward_corner"]}, "dry_run": {"type": "boolean"}, "reason": {"type": "string"}}}},
    {"name": "explore", "description": "Start one bounded fluent autonomous navigation mission while the middle layer continues reasoning.", "parameters": {"type": "object", "required": ["duration_s", "reason"], "properties": {"duration_s": {"type": "number", "minimum": 5, "maximum": 600}, "reason": {"type": "string"}}}},
    {"name": "speak", "description": "Speak briefly while stationary, optionally toward a tracked entity.", "parameters": {"type": "object", "required": ["text", "reason"], "properties": {"text": {"type": "string", "maxLength": 240}, "target_id": {"type": "string"}, "reason": {"type": "string"}}}},
    {"name": "gesture", "description": "Perform one whitelisted stationary body gesture.", "parameters": {"type": "object", "required": ["name", "reason"], "properties": {"name": {"enum": ["pray", "stretch", "swing"]}, "reason": {"type": "string"}}}},
    {"name": "stop", "description": "Cancel current bodily activity and request repeated motor stop.", "parameters": {"type": "object", "required": ["reason"], "properties": {"reason": {"type": "string"}}}},
]

_SCHEMAS = {item["name"]: item["parameters"] for item in FUNCTION_SCHEMAS}


@dataclass(frozen=True)
class SkillCall:
    name: str
    arguments: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _number(value: Any, field: str, minimum: float, maximum: float) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not minimum <= float(value) <= maximum:
        raise SkillValidationError(f"{field} must be within [{minimum}, {maximum}]")


def validate_skill_call(call: SkillCall) -> SkillCall:
    schema = _SCHEMAS.get(call.name)
    if schema is None:
        raise SkillValidationError(f"unknown embodied skill: {call.name}")
    arguments = call.arguments
    if not isinstance(arguments, dict):
        raise SkillValidationError("skill arguments must be an object")
    properties, required = schema["properties"], set(schema["required"])
    missing, unknown = required - arguments.keys(), arguments.keys() - properties.keys()
    if missing:
        raise SkillValidationError(f"missing skill arguments: {sorted(missing)}")
    if unknown:
        raise SkillValidationError(f"unknown skill arguments: {sorted(unknown)}")
    for key, value in arguments.items():
        rule = properties[key]
        if "enum" in rule and value not in rule["enum"]:
            raise SkillValidationError(f"{key} is not allowed")
        if rule.get("type") == "string":
            if not isinstance(value, str) or not value.strip():
                raise SkillValidationError(f"{key} must be a non-empty string")
            if len(value) > rule.get("maxLength", 240):
                raise SkillValidationError(f"{key} is too long")
        elif rule.get("type") == "boolean" and not isinstance(value, bool):
            raise SkillValidationError(f"{key} must be boolean")
        elif rule.get("type") == "number":
            _number(value, key, rule["minimum"], rule["maximum"])
    return call


def skill(name: str, **arguments: Any) -> SkillCall:
    return validate_skill_call(SkillCall(name=name, arguments=arguments))
