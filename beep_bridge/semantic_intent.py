"""Pure, bounded mission intent mailbox; no I/O and no motor authority.

The bridge serializes calls under mission_lock. IDs are retained for the whole
mission (capacity exhaustion rejects new work, never evicts deduplication keys).
"""
from __future__ import annotations

import copy
import math


MAX_EVIDENCE_AGE_S = 5.0
MAX_INTENTS = 1000
KINDS = frozenset({"continue", "course_left", "course_right", "yield", "stop"})
FIELDS = frozenset({"mission_id", "lease_id", "session_id", "decision_id",
                    "frame_id", "observed_at", "latest_frame_at", "expires_at", "kind"})


def finite(value: object) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def validate_intent(value, now):
    if not isinstance(value, dict) or set(value) != FIELDS:
        raise ValueError("invalid_intent_fields")
    for key in ("mission_id", "lease_id", "session_id", "decision_id", "frame_id"):
        if not isinstance(value[key], str) or not 1 <= len(value[key]) <= 160:
            raise ValueError("invalid_" + key)
    if not isinstance(value["kind"], str) or value["kind"] not in KINDS:
        raise ValueError("unsupported_intent")
    if not all(finite(value[key]) for key in ("observed_at", "latest_frame_at", "expires_at")):
        raise ValueError("invalid_evidence_time")
    if not (0 < value["observed_at"] <= value["latest_frame_at"] <= now
            < value["expires_at"] <= value["observed_at"] + MAX_EVIDENCE_AGE_S):
        raise ValueError("expired_or_future_evidence")
    return dict(value)


def sensor_gate(status):
    """Strict finite six-sector and localization gate, in addition to local guards."""
    if not isinstance(status, dict):
        return False
    age = status.get("scan_age_s")
    slam = status.get("slam") or {}
    if not isinstance(slam, dict):
        return False
    if not finite(age) or not 0 <= age <= 0.60 or slam.get("usable") is not True:
        return False
    if slam.get("pose_valid") is not True:
        return False
    for key, maximum in (("pose_age_s", 6.0), ("map_age_s", 2.5)):
        age = slam.get(key)
        if not finite(age) or not 0 <= age <= maximum:
            return False
    sectors = status.get("sectors") or {}
    if not isinstance(sectors, dict):
        return False
    return all(finite(sectors.get(key)) and sectors[key] > 0
               for key in ("front", "front_left", "front_right", "left", "right", "rear"))


class IntentMailbox:
    def __init__(self):
        self.records = {}
        self.pending = None
        self.session_id = None
        self.latest_frame_at = 0.0
        self.expires_at = None

    def outcome(self, decision_id, state, reason):
        record = self.records[decision_id]
        record.update(state=state, reason=reason)
        record["history"].append({"state": state, "reason": reason})
        return copy.deepcopy(record)

    def submit(self, value, now):
        # Repeated payloads return their actual outcome, even after expiry.
        if isinstance(value, dict) and isinstance(value.get("decision_id"), str):
            old = self.records.get(value["decision_id"])
            if old is not None:
                if old["intent"] != value:
                    return {"ok": False, "state": "rejected", "reason": "decision_id_conflict"}
                return copy.deepcopy(old)
        try:
            intent = validate_intent(value, now)
        except ValueError as exc:
            return {"ok": False, "state": "rejected", "reason": str(exc)}
        if self.session_id not in (None, intent["session_id"]):
            return {"ok": False, "state": "rejected", "reason": "executor_owner_mismatch"}
        if intent["latest_frame_at"] <= self.latest_frame_at:
            return {"ok": False, "state": "rejected", "reason": "superseded_evidence"}
        if len(self.records) >= MAX_INTENTS:
            return {"ok": False, "state": "rejected", "reason": "intent_capacity_exhausted"}
        if self.pending is not None:
            self.outcome(self.pending, "rejected", "superseded_intent")
        self.session_id = intent["session_id"]
        self.latest_frame_at = intent["latest_frame_at"]
        self.expires_at = intent["expires_at"]
        self.pending = intent["decision_id"]
        self.records[self.pending] = {"ok": True, "intent": intent, "history": []}
        return self.outcome(self.pending, "accepted", "queued_for_navigation_boundary")

    def take(self, now):
        key, self.pending = self.pending, None
        if key is None:
            return None
        record = self.records[key]
        try:
            validate_intent(record["intent"], now)
        except ValueError as exc:
            self.outcome(key, "rejected", str(exc))
            return None
        return dict(record["intent"])

    def snapshot(self):
        return copy.deepcopy(list(self.records.values()))

    def close(self, reason):
        for key, record in self.records.items():
            if record["state"] in {"accepted", "applied"}:
                self.outcome(key, "rejected", reason)
        self.pending = None
