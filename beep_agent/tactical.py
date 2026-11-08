"""Phase B local attention contracts; never a motor or goal planner."""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import asdict, dataclass
from math import isfinite, pi
from typing import Any

from .session import AgentSession


class TacticalValidationError(ValueError):
    pass


def _id(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip()) and len(value) <= 256


@dataclass(frozen=True)
class TacticalEvent:
    schema_version: int
    event_id: str
    session_id: str
    boot_id: str
    map_epoch: int
    map_revision: int | None
    mission_id: str | None
    lease_id: str | None
    source: str
    acquired_at_robot: float
    ttl_ms: int
    confidence: float
    suggestion: str
    bearing_hint_rad: float | None
    evidence_ids: tuple[str, ...]

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "TacticalEvent":
        if not isinstance(raw, dict) or set(raw) != set(cls.__dataclass_fields__):
            raise TacticalValidationError("tactical event fields differ from v1")
        if type(raw["schema_version"]) is not int or raw["schema_version"] != 1:
            raise TacticalValidationError("unsupported tactical schema")
        if not all(_id(raw[k]) for k in ("event_id", "session_id", "boot_id")):
            raise TacticalValidationError("invalid tactical identity")
        if any(raw[k] is not None and not _id(raw[k]) for k in ("mission_id", "lease_id")):
            raise TacticalValidationError("invalid motion identity")
        if (type(raw["map_epoch"]) is not int or raw["map_epoch"] < 0 or
                (raw["map_revision"] is not None and
                 (type(raw["map_revision"]) is not int or raw["map_revision"] < 0))):
            raise TacticalValidationError("invalid map identity")
        if (not isinstance(raw["source"], str) or raw["source"] not in {
                "vad_onset", "vad_end", "visual_salience", "local_hazard", "speech_keyword"}):
            raise TacticalValidationError("invalid tactical source")
        if (not isinstance(raw["suggestion"], str) or raw["suggestion"] not in {
                "notify_strategy", "attention_candidate", "ack_cue", "request_yield"}):
            raise TacticalValidationError("invalid tactical suggestion")
        for k in ("acquired_at_robot", "confidence"):
            if isinstance(raw[k], bool) or not isinstance(raw[k], (int, float)) or not isfinite(raw[k]):
                raise TacticalValidationError("invalid tactical time/confidence")
        if raw["acquired_at_robot"] < 0 or not 0 <= raw["confidence"] <= 1:
            raise TacticalValidationError("invalid tactical time/confidence range")
        if type(raw["ttl_ms"]) is not int or not 1 <= raw["ttl_ms"] <= 10_000:
            raise TacticalValidationError("invalid tactical TTL")
        bearing = raw["bearing_hint_rad"]
        if bearing is not None and (isinstance(bearing, bool) or
                not isinstance(bearing, (int, float)) or not isfinite(bearing) or
                not -pi <= bearing <= pi):
            raise TacticalValidationError("invalid bearing hint")
        evidence = raw["evidence_ids"]
        if (not isinstance(evidence, list) or len(evidence) > 8 or
                not all(_id(x) for x in evidence) or len(set(evidence)) != len(evidence)):
            raise TacticalValidationError("invalid evidence IDs")
        return cls(**{**raw, "acquired_at_robot": float(raw["acquired_at_robot"]),
                      "confidence": float(raw["confidence"]),
                      "bearing_hint_rad": None if bearing is None else float(bearing),
                      "evidence_ids": tuple(evidence)})

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["evidence_ids"] = list(self.evidence_ids)
        return result


@dataclass(frozen=True)
class TacticalDisposition:
    accepted: bool
    suggestion: str | None
    reason: str
    motor_authority: bool = False


class TacticalGate:
    """Bounded dedupe/attention gate: only advisory outputs, never dispatch."""

    def __init__(self, session: AgentSession, *, max_seen: int = 512,
                 cue_cooldown_s: float = 2.0):
        if max_seen < 1 or cue_cooldown_s < 0:
            raise ValueError("invalid attention limits")
        self.session = session
        self.max_seen = max_seen
        self.cue_cooldown_s = cue_cooldown_s
        self._seen: OrderedDict[str, TacticalEvent] = OrderedDict()
        self._last_cue_at: float | None = None

    def ingest(self, event: TacticalEvent, *, now_robot: float, map_epoch: int,
               map_revision: int | None, locomotion_active: bool,
               corroborated_visual: bool = False, corroborated_speech: bool = False,
               verified_hazard: bool = False, mixer_allowed: bool = False) -> TacticalDisposition:
        if not isinstance(event, TacticalEvent):
            raise TacticalValidationError("expected validated tactical event")
        event = TacticalEvent.from_dict(event.to_dict())
        if (isinstance(now_robot, bool) or not isinstance(now_robot, (int, float))
                or not isfinite(now_robot) or now_robot < 0):
            raise TacticalValidationError("invalid current robot clock")
        owner = self.session.execution["lease_id"]
        if ((event.session_id, event.boot_id) != (self.session.session_id, self.session.boot_id)
                or (event.mission_id, event.lease_id) !=
                   ((self.session.mission_id, owner) if owner is not None else (None, None))):
            return TacticalDisposition(False, None, "wrong_owner")
        if (event.map_epoch, event.map_revision) != (map_epoch, map_revision):
            return TacticalDisposition(False, None, "map_revision_changed")
        age = now_robot - event.acquired_at_robot
        if age < 0 or age * 1000 > event.ttl_ms:
            return TacticalDisposition(False, None, "expired_or_future")
        previous = self._seen.get(event.event_id)
        if previous is not None:
            return TacticalDisposition(False, None,
                                       "duplicate" if previous == event else "conflicting_duplicate")
        self._seen[event.event_id] = event
        if len(self._seen) > self.max_seen:
            self._seen.popitem(last=False)
        suggestion, reason = event.suggestion, "advisory_only"
        if event.source == "visual_salience" and locomotion_active and not corroborated_visual:
            suggestion, reason = "attention_candidate", "egomotion_ambiguous"
        elif event.source in {"vad_onset", "vad_end"} and locomotion_active and not corroborated_speech:
            suggestion, reason = "attention_candidate", "motor_audio_ambiguous"
        elif event.source == "speech_keyword" and event.suggestion == "request_yield":
            suggestion, reason = "notify_strategy", "separate_stop_required"
        elif event.source == "local_hazard" and not verified_hazard:
            suggestion, reason = "attention_candidate", "unverified_hazard"
        elif suggestion == "request_yield" and event.source not in {"local_hazard", "visual_salience"}:
            suggestion, reason = "notify_strategy", "local_policy_required"
        if suggestion == "ack_cue":
            if not mixer_allowed or (event.source in {"vad_onset", "vad_end"} and not corroborated_speech):
                suggestion, reason = "attention_candidate", "mixer_or_speech_unverified"
            elif self._last_cue_at is not None and now_robot - self._last_cue_at < self.cue_cooldown_s:
                suggestion, reason = "attention_candidate", "cue_rate_limited"
            else:
                self._last_cue_at = now_robot
        return TacticalDisposition(True, suggestion, reason)
