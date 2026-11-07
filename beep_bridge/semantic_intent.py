"""Pure, bounded mission intent mailbox; no I/O and no motor authority.

The bridge serializes calls under mission_lock. IDs are retained for the whole
mission (capacity exhaustion rejects new work, never evicts deduplication keys).
"""
from __future__ import annotations

import copy
import math
from dataclasses import asdict, dataclass


MAX_EVIDENCE_AGE_S = 5.0
MAX_INTENTS = 1000
KINDS = frozenset({"continue", "course_left", "course_right", "yield", "stop"})
FIELDS = frozenset({"mission_id", "lease_id", "session_id", "decision_id",
                    "frame_id", "observed_at", "latest_frame_at", "expires_at", "kind"})
FENCED_FIELDS = FIELDS | {"executor_instance_id", "executor_generation"}


@dataclass(frozen=True)
class ExecutorFence:
    mission_id: str
    lease_id: str
    session_id: str
    executor_instance_id: str
    executor_generation: int

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or set(value) != set(cls.__annotations__):
            raise ValueError("invalid_executor_fence")
        for key in set(cls.__annotations__) - {"executor_generation"}:
            if not isinstance(value[key], str) or not 1 <= len(value[key]) <= 160:
                raise ValueError("invalid_executor_fence")
        if type(value["executor_generation"]) is not int or value["executor_generation"] < 1:
            raise ValueError("invalid_executor_generation")
        return cls(**value)

    def to_dict(self):
        return asdict(self)

    def matches(self, intent):
        return (isinstance(intent, dict) and type(intent.get("executor_generation")) is int
                and all(intent.get(k) == v for k, v in self.to_dict().items()))


def finite(value: object) -> bool:
    return type(value) in (int, float) and math.isfinite(value)


def validate_intent(value, now):
    if not isinstance(value, dict) or set(value) not in (FIELDS, FENCED_FIELDS):
        raise ValueError("invalid_intent_fields")
    if set(value) == FENCED_FIELDS:
        ExecutorFence.from_dict({k: value[k] for k in ExecutorFence.__annotations__})
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
    def __init__(self, *, require_fencing=False):
        self.records = {}
        self.pending = None
        self.session_id = None
        self.latest_frame_at = 0.0
        self.expires_at = None
        # Legacy admission exists only for the still-interlocked bridge. New
        # bridges MUST use require_fencing=True; claim permanently fences either.
        self.require_fencing = require_fencing
        self.fence = None
        self.closed = False
        self.in_flight = set()

    def fence_snapshot(self):
        return None if self.fence is None else self.fence.to_dict()

    def claim(self, value, *, safe_to_takeover=False):
        """CAS under bridge mission/transport locks, never trust a client safe flag.

        Bridge must verify exact live mission/lease and quiesce old packet writes
        before passing safe_to_takeover=True. Keep this mailbox for the mission's
        entire lifetime. Claim retries use the SAME instance and expected epoch.
        """
        def reject(reason):
            return {"ok": False, "reason": reason}
        fields = {"mission_id", "lease_id", "session_id", "executor_instance_id",
                  "expected_generation", "takeover"}
        if not isinstance(value, dict) or set(value) != fields:
            return reject("invalid_executor_claim")
        expected = value["expected_generation"]
        if type(expected) is not int or expected < 0 or type(value["takeover"]) is not bool:
            return reject("invalid_executor_claim")
        try:
            candidate = ExecutorFence.from_dict({
                **{k: value[k] for k in fields - {"expected_generation", "takeover"}},
                "executor_generation": expected + 1})
        except ValueError as exc:
            return reject(str(exc))
        if self.closed:
            return reject("mailbox_closed")
        if self.fence == candidate:
            return {"ok": True, "fence": self.fence_snapshot(), "intent_outcomes": self.snapshot()}
        generation = 0 if self.fence is None else self.fence.executor_generation
        if generation != expected:
            return reject("executor_generation_conflict")
        if self.session_id not in (None, candidate.session_id):
            return reject("executor_owner_mismatch")
        if self.fence is not None:
            if any(getattr(self.fence, k) != getattr(candidate, k)
                   for k in ("mission_id", "lease_id", "session_id")):
                return reject("executor_owner_mismatch")
        if self.fence is not None or self.records:
            if not value["takeover"] or not safe_to_takeover:
                return reject("explicit_safe_takeover_required")
        for key, record in self.records.items():
            if record["state"] in {"accepted", "applied"}:
                if key in self.in_flight:
                    self.outcome(key, "unknown", "takeover_delivery_unconfirmed")
                else:
                    self.outcome(key, "rejected", "executor_taken_over_before_delivery")
        self.pending = None
        self.fence = candidate
        self.session_id = candidate.session_id
        self.require_fencing = True
        return {"ok": True, "fence": self.fence_snapshot(), "intent_outcomes": self.snapshot()}

    def authorizes(self, intent, now):
        """Recheck at EACH serialized packet boundary, not just queue admission."""
        if self.closed or (self.require_fencing and (self.fence is None or not self.fence.matches(intent))):
            return False
        try:
            validate_intent(intent, now)
        except ValueError:
            return False
        record = self.records.get(intent["decision_id"])
        return (record is not None and record["intent"] == intent
                and record["state"] in {"accepted", "applied"})

    def outcome(self, decision_id, state, reason):
        record = self.records[decision_id]
        # Late old workers and reordered acknowledgments cannot revive authority.
        if record.get("state") in {"completed", "rejected", "unknown"}:
            return copy.deepcopy(record)
        if record.get("state") == "applied" and state == "accepted":
            return copy.deepcopy(record)
        record.update(state=state, reason=reason)
        record["history"].append({"state": state, "reason": reason})
        return copy.deepcopy(record)

    def submit(self, value, now):
        if self.closed:
            return {"ok": False, "state": "rejected", "reason": "mailbox_closed"}
        if self.require_fencing and (self.fence is None or not self.fence.matches(value)):
            return {"ok": False, "state": "rejected", "reason": "stale_executor_fence"}
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
        if not self.authorizes(record["intent"], now):
            self.outcome(key, "rejected", "intent_authority_expired")
            return None
        try:
            validate_intent(record["intent"], now)
        except ValueError as exc:
            self.outcome(key, "rejected", str(exc))
            return None
        self.in_flight.add(key)
        return dict(record["intent"])

    def snapshot(self):
        return copy.deepcopy(list(self.records.values()))

    def close(self, reason):
        self.closed = True
        for key, record in self.records.items():
            if record["state"] in {"accepted", "applied"}:
                self.outcome(key, "rejected", reason)
        self.pending = None
