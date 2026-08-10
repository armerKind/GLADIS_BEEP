from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import hashlib
import json
import os
from pathlib import Path
import time
from typing import Any, Mapping, Protocol


class RiskLevel(str, Enum):
    CLEAR = "clear"
    WATCH = "watch"
    STOP = "stop"


@dataclass(frozen=True)
class Frame:
    source: str
    sequence: int
    captured_at: float
    payload: bytes | None = None
    suffix: str = ".jpg"
    metadata: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class Assessment:
    event_id: str
    source: str
    sequence: int
    captured_at: float
    assessed_at: float
    risk: RiskLevel
    reasons: tuple[str, ...] = ()
    metrics: Mapping[str, float | int | str | bool | None] = field(default_factory=dict)


@dataclass(frozen=True)
class StopOutcome:
    requested: bool
    delivered: bool
    reason: str
    mission_id: str | None = None
    response: Mapping[str, Any] = field(default_factory=dict)


class FrameSource(Protocol):
    def read(self) -> Frame | None: ...


class RiskEvaluator(Protocol):
    def evaluate(self, frame: Frame) -> Assessment: ...


class StopSink(Protocol):
    @property
    def authenticated(self) -> bool: ...

    def stop(self, assessment: Assessment) -> StopOutcome: ...


@dataclass(frozen=True)
class ObserverConfig:
    mode: str = "record"
    stop_cooldown_s: float = 2.0

    def __post_init__(self) -> None:
        if self.mode not in {"record", "stop"}:
            raise ValueError("observer mode must be 'record' or 'stop'")
        if self.stop_cooldown_s < 0.5:
            raise ValueError("stop cooldown must be at least 0.5 seconds")


class MetadataRiskEvaluator:
    """Fixture/plugin adapter; real vision must emit the same bounded schema."""

    def evaluate(self, frame: Frame) -> Assessment:
        raw_risk = str(frame.metadata.get("risk", "clear")).lower()
        try:
            risk = RiskLevel(raw_risk)
        except ValueError as exc:
            raise ValueError(f"unsupported observer risk: {raw_risk}") from exc
        reasons = tuple(str(item) for item in frame.metadata.get("reasons", ()))
        metrics = frame.metadata.get("metrics", {})
        if not isinstance(metrics, Mapping):
            raise ValueError("observer metrics must be a mapping")
        event_material = f"{frame.source}:{frame.sequence}:{frame.captured_at:.6f}".encode()
        event_id = hashlib.sha256(event_material).hexdigest()[:20]
        return Assessment(
            event_id=event_id,
            source=frame.source,
            sequence=frame.sequence,
            captured_at=frame.captured_at,
            assessed_at=time.time(),
            risk=risk,
            reasons=reasons,
            metrics=dict(metrics),
        )


class EvidenceStore:
    """Local JSONL evidence; image bytes are retained only for watch/stop events."""

    def __init__(self, root: str | Path, retain_frames_for=(RiskLevel.WATCH, RiskLevel.STOP)):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(self.root, 0o700)
        self.events_path = self.root / "events.jsonl"
        self.retain_frames_for = frozenset(RiskLevel(level) for level in retain_frames_for)

    def record(self, frame: Frame, assessment: Assessment, outcome: StopOutcome) -> dict[str, Any]:
        frame_sha256 = hashlib.sha256(frame.payload).hexdigest() if frame.payload is not None else None
        retained_path = None
        if frame.payload is not None and assessment.risk in self.retain_frames_for:
            suffix = frame.suffix if frame.suffix.startswith(".") else ".bin"
            target = self.root / f"{assessment.event_id}{suffix}"
            target.write_bytes(frame.payload)
            os.chmod(target, 0o600)
            retained_path = str(target)
        event = {
            **asdict(assessment),
            "risk": assessment.risk.value,
            "reasons": list(assessment.reasons),
            "metrics": dict(assessment.metrics),
            "frame_sha256": frame_sha256,
            "retained_path": retained_path,
            "stop": asdict(outcome),
        }
        with self.events_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(event, sort_keys=True, separators=(",", ":")) + "\n")
        os.chmod(self.events_path, 0o600)
        return event


class ExternalObserver:
    def __init__(self, source: FrameSource, evaluator: RiskEvaluator, evidence: EvidenceStore,
                 config: ObserverConfig | None = None, stop_sink: StopSink | None = None,
                 clock=time.monotonic):
        self.source = source
        self.evaluator = evaluator
        self.evidence = evidence
        self.config = config or ObserverConfig()
        self.stop_sink = stop_sink
        self.clock = clock
        self._last_stop_at: float | None = None

    def run_once(self) -> dict[str, Any] | None:
        frame = self.source.read()
        if frame is None:
            return None
        assessment = self.evaluator.evaluate(frame)
        outcome = self._maybe_stop(assessment)
        return self.evidence.record(frame, assessment, outcome)

    def _maybe_stop(self, assessment: Assessment) -> StopOutcome:
        if assessment.risk is not RiskLevel.STOP:
            return StopOutcome(False, False, "risk_below_stop")
        if self.config.mode != "stop":
            return StopOutcome(False, False, "record_only_mode")
        if self.stop_sink is None:
            return StopOutcome(False, False, "stop_sink_missing")
        if not self.stop_sink.authenticated:
            return StopOutcome(False, False, "stop_sink_unauthenticated")
        now = self.clock()
        if self._last_stop_at is not None and now - self._last_stop_at < self.config.stop_cooldown_s:
            return StopOutcome(False, False, "stop_rate_limited")
        self._last_stop_at = now
        return self.stop_sink.stop(assessment)
