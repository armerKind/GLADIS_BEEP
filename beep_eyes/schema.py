"""Strict typed contract for BEEP temporal perception responses."""
from __future__ import annotations

from typing import Any

SCHEMA_VERSION = "1.0"
POSITIONS = {"left", "center", "right", "unknown"}
ENGAGEMENTS = {"none", "looking", "speaking", "approaching", "leaving", "unknown"}
TARGET_TYPES = {"person", "object", "direction", "none"}
HAZARD_TYPES = {"glass", "stairs", "drop", "person", "animal", "vehicle", "obstacle", "unknown"}
SKILLS = {"observe", "stop", "orient", "advance", "retreat", "explore", "gesture", "speak"}
GESTURES = {"pray", "stretch", "swing"}
DIRECTIONS = {"left", "right"}


class PerceptionValidationError(ValueError):
    """Raised when a model response violates the physical-planning contract."""


def _mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise PerceptionValidationError(f"{path} must be an object")
    return value


def _list(value: Any, path: str, maximum: int) -> list[Any]:
    if not isinstance(value, list):
        raise PerceptionValidationError(f"{path} must be an array")
    if len(value) > maximum:
        raise PerceptionValidationError(f"{path} must contain at most {maximum} items")
    return value


def _text(value: Any, path: str, maximum: int, allow_empty: bool = False) -> str:
    if not isinstance(value, str):
        raise PerceptionValidationError(f"{path} must be a string")
    if not allow_empty and not value.strip():
        raise PerceptionValidationError(f"{path} must not be empty")
    if len(value) > maximum:
        raise PerceptionValidationError(f"{path} exceeds {maximum} characters")
    return value


def _number(value: Any, path: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PerceptionValidationError(f"{path} must be numeric")
    number = float(value)
    if not minimum <= number <= maximum:
        raise PerceptionValidationError(f"{path} must be within [{minimum}, {maximum}]")
    return number


def _exact_keys(value: dict[str, Any], path: str, required: set[str], optional: set[str] | None = None) -> None:
    optional = optional or set()
    missing = required - value.keys()
    unknown = value.keys() - required - optional
    if missing:
        raise PerceptionValidationError(f"{path} missing keys: {sorted(missing)}")
    if unknown:
        raise PerceptionValidationError(f"{path} contains unknown keys: {sorted(unknown)}")


def _entity(value: Any, path: str, *, person: bool) -> None:
    item = _mapping(value, path)
    required = {"id", "description", "position", "confidence"}
    if person:
        required.add("engagement")
    _exact_keys(item, path, required)
    _text(item["id"], f"{path}.id", 48)
    _text(item["description"], f"{path}.description", 240)
    if item["position"] not in POSITIONS:
        raise PerceptionValidationError(f"{path}.position is invalid")
    _number(item["confidence"], f"{path}.confidence", 0.0, 1.0)
    if person and item["engagement"] not in ENGAGEMENTS:
        raise PerceptionValidationError(f"{path}.engagement is invalid")


def _hazard(value: Any, path: str) -> None:
    item = _mapping(value, path)
    _exact_keys(item, path, {"type", "description", "position", "confidence"})
    if item["type"] not in HAZARD_TYPES:
        raise PerceptionValidationError(f"{path}.type is invalid")
    _text(item["description"], f"{path}.description", 240)
    if item["position"] not in POSITIONS:
        raise PerceptionValidationError(f"{path}.position is invalid")
    _number(item["confidence"], f"{path}.confidence", 0.0, 1.0)


def _step(value: Any, path: str) -> None:
    item = _mapping(value, path)
    if "skill" not in item or "reason" not in item:
        raise PerceptionValidationError(f"{path} requires skill and reason")
    skill = item["skill"]
    if skill not in SKILLS:
        raise PerceptionValidationError(f"{path}.skill is not whitelisted")
    required = {"skill", "reason"}
    if skill == "orient":
        required |= {"direction", "degrees"}
    elif skill in {"advance", "retreat"}:
        required.add("distance_m")
    elif skill == "explore":
        required.add("duration_s")
    elif skill == "gesture":
        required.add("name")
    elif skill == "speak":
        required.add("text")
    _exact_keys(item, path, required)
    _text(item["reason"], f"{path}.reason", 240)
    if skill == "orient":
        if item["direction"] not in DIRECTIONS:
            raise PerceptionValidationError(f"{path}.direction is invalid")
        _number(item["degrees"], f"{path}.degrees", 1.0, 30.0)
    elif skill in {"advance", "retreat"}:
        _number(item["distance_m"], f"{path}.distance_m", 0.05, 0.30)
    elif skill == "explore":
        _number(item["duration_s"], f"{path}.duration_s", 5.0, 30.0)
    elif skill == "gesture":
        if item["name"] not in GESTURES:
            raise PerceptionValidationError(f"{path}.name is invalid")
    elif skill == "speak":
        _text(item["text"], f"{path}.text", 240)


def validate_perception_response(value: Any) -> dict[str, Any]:
    """Validate and return a model response without coercing unsafe values."""
    root = _mapping(value, "response")
    _exact_keys(
        root,
        "response",
        {"schema_version", "scene", "attention", "plan", "confidence", "uncertainty", "escalate", "escalation_reason"},
    )
    if root["schema_version"] != SCHEMA_VERSION:
        raise PerceptionValidationError("unsupported schema_version")

    scene = _mapping(root["scene"], "scene")
    _exact_keys(scene, "scene", {"summary", "changes", "people", "objects", "hazards"})
    _text(scene["summary"], "scene.summary", 600)
    for index, change in enumerate(_list(scene["changes"], "scene.changes", 8)):
        _text(change, f"scene.changes[{index}]", 240)
    for index, person in enumerate(_list(scene["people"], "scene.people", 12)):
        _entity(person, f"scene.people[{index}]", person=True)
    for index, obj in enumerate(_list(scene["objects"], "scene.objects", 20)):
        _entity(obj, f"scene.objects[{index}]", person=False)
    for index, hazard in enumerate(_list(scene["hazards"], "scene.hazards", 10)):
        _hazard(hazard, f"scene.hazards[{index}]")

    attention = _mapping(root["attention"], "attention")
    _exact_keys(attention, "attention", {"target_type", "target_id", "reason"})
    if attention["target_type"] not in TARGET_TYPES:
        raise PerceptionValidationError("attention.target_type is invalid")
    if attention["target_id"] is not None:
        _text(attention["target_id"], "attention.target_id", 48)
    if attention["target_type"] == "none" and attention["target_id"] is not None:
        raise PerceptionValidationError("attention.target_id must be null when target_type is none")
    if attention["target_type"] != "none" and attention["target_id"] is None:
        raise PerceptionValidationError("attention.target_id is required for a selected target")
    _text(attention["reason"], "attention.reason", 240)

    plan = _mapping(root["plan"], "plan")
    _exact_keys(plan, "plan", {"goal", "steps"})
    _text(plan["goal"], "plan.goal", 240)
    steps = _list(plan["steps"], "plan.steps", 3)
    for index, step in enumerate(steps):
        _step(step, f"plan.steps[{index}]")

    _number(root["confidence"], "confidence", 0.0, 1.0)
    for index, uncertainty in enumerate(_list(root["uncertainty"], "uncertainty", 8)):
        _text(uncertainty, f"uncertainty[{index}]", 240)
    if not isinstance(root["escalate"], bool):
        raise PerceptionValidationError("escalate must be boolean")
    if root["escalation_reason"] is not None:
        _text(root["escalation_reason"], "escalation_reason", 240)
    if root["escalate"] and root["escalation_reason"] is None:
        raise PerceptionValidationError("escalation_reason is required when escalate=true")
    return root


RESPONSE_JSON_SCHEMA: dict[str, Any] = {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "type": "object",
    "additionalProperties": False,
    "required": ["schema_version", "scene", "attention", "plan", "confidence", "uncertainty", "escalate", "escalation_reason"],
    "properties": {
        "schema_version": {"type": "string", "const": SCHEMA_VERSION},
        "scene": {
            "type": "object", "additionalProperties": False,
            "required": ["summary", "changes", "people", "objects", "hazards"],
            "properties": {
                "summary": {"type": "string", "maxLength": 600},
                "changes": {"type": "array", "maxItems": 8, "items": {"type": "string", "maxLength": 240}},
                "people": {"type": "array", "maxItems": 12, "items": {"$ref": "#/$defs/person"}},
                "objects": {"type": "array", "maxItems": 20, "items": {"$ref": "#/$defs/object"}},
                "hazards": {"type": "array", "maxItems": 10, "items": {"$ref": "#/$defs/hazard"}},
            },
        },
        "attention": {
            "type": "object", "additionalProperties": False,
            "required": ["target_type", "target_id", "reason"],
            "properties": {
                "target_type": {"type": "string", "enum": sorted(TARGET_TYPES)},
                "target_id": {"type": ["string", "null"], "maxLength": 48},
                "reason": {"type": "string", "maxLength": 240},
            },
        },
        "plan": {
            "type": "object", "additionalProperties": False,
            "required": ["goal", "steps"],
            "properties": {
                "goal": {"type": "string", "maxLength": 240},
                "steps": {"type": "array", "maxItems": 3, "items": {"$ref": "#/$defs/step"}},
            },
        },
        "confidence": {"type": "number", "minimum": 0, "maximum": 1},
        "uncertainty": {"type": "array", "maxItems": 8, "items": {"type": "string", "maxLength": 240}},
        "escalate": {"type": "boolean"},
        "escalation_reason": {"type": ["string", "null"], "maxLength": 240},
    },
    "$defs": {
        "person": {
            "type": "object", "additionalProperties": False,
            "required": ["id", "description", "position", "engagement", "confidence"],
            "properties": {
                "id": {"type": "string", "maxLength": 48}, "description": {"type": "string", "maxLength": 240},
                "position": {"type": "string", "enum": sorted(POSITIONS)},
                "engagement": {"type": "string", "enum": sorted(ENGAGEMENTS)},
                "confidence": {"type": "number", "minimum": 0, "maximum": 1},
            },
        },
        "object": {
            "type": "object", "additionalProperties": False,
            "required": ["id", "description", "position", "confidence"],
            "properties": {
                "id": {"type": "string", "maxLength": 48}, "description": {"type": "string", "maxLength": 240},
                "position": {"type": "string", "enum": sorted(POSITIONS)},
                "confidence": {"type": "number", "minimum": 0, "maximum": 1},
            },
        },
        "hazard": {
            "type": "object", "additionalProperties": False,
            "required": ["type", "description", "position", "confidence"],
            "properties": {
                "type": {"type": "string", "enum": sorted(HAZARD_TYPES)},
                "description": {"type": "string", "maxLength": 240},
                "position": {"type": "string", "enum": sorted(POSITIONS)},
                "confidence": {"type": "number", "minimum": 0, "maximum": 1},
            },
        },
        "step": {
            "type": "object",
            "oneOf": [
                {"$ref": "#/$defs/observe_step"}, {"$ref": "#/$defs/stop_step"}, {"$ref": "#/$defs/orient_step"},
                {"$ref": "#/$defs/distance_step"}, {"$ref": "#/$defs/explore_step"}, {"$ref": "#/$defs/gesture_step"}, {"$ref": "#/$defs/speak_step"},
            ],
        },
        "base_reason": {"type": "string", "maxLength": 240},
        "observe_step": {"type": "object", "additionalProperties": False, "required": ["skill", "reason"], "properties": {"skill": {"const": "observe"}, "reason": {"$ref": "#/$defs/base_reason"}}},
        "stop_step": {"type": "object", "additionalProperties": False, "required": ["skill", "reason"], "properties": {"skill": {"const": "stop"}, "reason": {"$ref": "#/$defs/base_reason"}}},
        "orient_step": {"type": "object", "additionalProperties": False, "required": ["skill", "direction", "degrees", "reason"], "properties": {"skill": {"const": "orient"}, "direction": {"type": "string", "enum": sorted(DIRECTIONS)}, "degrees": {"type": "number", "minimum": 1, "maximum": 30}, "reason": {"$ref": "#/$defs/base_reason"}}},
        "distance_step": {"type": "object", "additionalProperties": False, "required": ["skill", "distance_m", "reason"], "properties": {"skill": {"type": "string", "enum": ["advance", "retreat"]}, "distance_m": {"type": "number", "minimum": 0.05, "maximum": 0.30}, "reason": {"$ref": "#/$defs/base_reason"}}},
        "explore_step": {"type": "object", "additionalProperties": False, "required": ["skill", "duration_s", "reason"], "properties": {"skill": {"const": "explore"}, "duration_s": {"type": "number", "minimum": 5, "maximum": 30}, "reason": {"$ref": "#/$defs/base_reason"}}},
        "gesture_step": {"type": "object", "additionalProperties": False, "required": ["skill", "name", "reason"], "properties": {"skill": {"const": "gesture"}, "name": {"type": "string", "enum": sorted(GESTURES)}, "reason": {"$ref": "#/$defs/base_reason"}}},
        "speak_step": {"type": "object", "additionalProperties": False, "required": ["skill", "text", "reason"], "properties": {"skill": {"const": "speak"}, "text": {"type": "string", "maxLength": 240}, "reason": {"$ref": "#/$defs/base_reason"}}},
    },
}


# Gemini accepts only a complexity-limited JSON Schema subset. The compact
# wire schema guides generation; validate_perception_response remains authoritative.
GEMINI_COMPACT_JSON_SCHEMA: dict[str, Any] = {
    "type": "object",
    "required": [
        "schema_version", "scene_summary", "changes", "entities", "hazards",
        "attention_type", "attention_id", "attention_reason", "goal", "steps",
        "confidence", "uncertainty", "escalate", "escalation_reason",
    ],
    "properties": {
        "schema_version": {"type": "string", "enum": [SCHEMA_VERSION]},
        "scene_summary": {"type": "string"},
        "changes": {"type": "array", "items": {"type": "string"}},
        "entities": {
            "type": "array",
            "items": {
                "type": "object",
                "required": ["kind", "id", "description", "position", "engagement", "confidence"],
                "properties": {
                    "kind": {"type": "string", "enum": ["person", "object"]},
                    "id": {"type": "string"}, "description": {"type": "string"},
                    "position": {"type": "string", "enum": sorted(POSITIONS)},
                    "engagement": {"type": "string", "enum": sorted(ENGAGEMENTS)},
                    "confidence": {"type": "number"},
                },
            },
        },
        "hazards": {
            "type": "array",
            "items": {
                "type": "object",
                "required": ["type", "description", "position", "confidence"],
                "properties": {
                    "type": {"type": "string", "enum": sorted(HAZARD_TYPES)},
                    "description": {"type": "string"},
                    "position": {"type": "string", "enum": sorted(POSITIONS)},
                    "confidence": {"type": "number"},
                },
            },
        },
        "attention_type": {"type": "string", "enum": sorted(TARGET_TYPES)},
        "attention_id": {"type": "string"}, "attention_reason": {"type": "string"},
        "goal": {"type": "string"},
        "steps": {
            "type": "array",
            "items": {
                "type": "object", "required": ["skill", "reason", "argument", "amount"],
                "properties": {
                    "skill": {"type": "string", "enum": sorted(SKILLS)},
                    "reason": {"type": "string"}, "argument": {"type": "string"},
                    "amount": {"type": "number"},
                },
            },
        },
        "confidence": {"type": "number"},
        "uncertainty": {"type": "array", "items": {"type": "string"}},
        "escalate": {"type": "boolean"}, "escalation_reason": {"type": "string"},
    },
}


def normalize_gemini_response(value: Any) -> dict[str, Any]:
    """Convert Gemini's compact wire format into the strict local contract."""
    wire = _mapping(value, "gemini_response")
    wire_keys = {
        "schema_version", "scene_summary", "changes", "entities", "hazards",
        "attention_type", "attention_id", "attention_reason", "goal", "steps",
        "confidence", "uncertainty", "escalate", "escalation_reason",
    }
    _exact_keys(wire, "gemini_response", wire_keys)
    entities = _list(wire.get("entities"), "gemini_response.entities", 32)
    people: list[dict[str, Any]] = []
    objects: list[dict[str, Any]] = []
    for entity in entities:
        item = _mapping(entity, "gemini_response.entity")
        _exact_keys(item, "gemini_response.entity", {"kind", "id", "description", "position", "engagement", "confidence"})
        if item.get("kind") not in {"person", "object"}:
            raise PerceptionValidationError("gemini_response.entity.kind is invalid")
        base = {key: item.get(key) for key in ("id", "description", "position", "confidence")}
        if item.get("kind") == "person":
            people.append({**base, "engagement": item.get("engagement")})
        else:
            objects.append(base)

    steps = []
    for item in _list(wire.get("steps"), "gemini_response.steps", 3):
        step = _mapping(item, "gemini_response.step")
        _exact_keys(step, "gemini_response.step", {"skill", "reason", "argument", "amount"})
        skill = step.get("skill")
        canonical: dict[str, Any] = {"skill": skill, "reason": step.get("reason")}
        if skill == "orient":
            canonical.update(direction=step.get("argument"), degrees=step.get("amount"))
        elif skill in {"advance", "retreat"}:
            canonical["distance_m"] = step.get("amount")
        elif skill == "explore":
            canonical["duration_s"] = step.get("amount")
        elif skill == "gesture":
            canonical["name"] = step.get("argument")
        elif skill == "speak":
            canonical["text"] = step.get("argument")
        steps.append(canonical)

    normalized = {
        "schema_version": wire.get("schema_version"),
        "scene": {
            "summary": wire.get("scene_summary"), "changes": wire.get("changes"),
            "people": people, "objects": objects, "hazards": wire.get("hazards"),
        },
        "attention": {
            "target_type": wire.get("attention_type"),
            "target_id": wire.get("attention_id") or None,
            "reason": wire.get("attention_reason"),
        },
        "plan": {"goal": wire.get("goal"), "steps": steps},
        "confidence": wire.get("confidence"), "uncertainty": wire.get("uncertainty"),
        "escalate": wire.get("escalate"),
        "escalation_reason": wire.get("escalation_reason") or None,
    }
    return validate_perception_response(normalized)
