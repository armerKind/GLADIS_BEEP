"""Offline persistent GLADIS strategic model-session adapter; no model or motor I/O.

A caller injects one long-lived inference provider and live robot-clock/map readers.
This class owns one in-flight request and replaces queued stale windows with the
latest validated EventJournal evidence. Only StrategicOwner admits a goal.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from hashlib import sha256
import json
from math import isfinite
import threading
from typing import Callable

from .journal import EventJournal, JournalError
from .session import AgentSession
from .strategy import GoalProposal, StrategicOwner, StrategyError


class AdapterError(ValueError):
    pass


@dataclass(frozen=True)
class ModelWindow:
    sequence: int
    acquired_at_robot: float
    map_epoch: int
    map_revision: int | None
    source_event_ids: tuple[str, ...]

    def __post_init__(self):
        if (type(self.sequence) is not int or self.sequence < 0
                or type(self.map_epoch) is not int or self.map_epoch < 0
                or (self.map_revision is not None and
                    (type(self.map_revision) is not int or self.map_revision < 0))
                or type(self.acquired_at_robot) not in (int, float)
                or not isfinite(self.acquired_at_robot) or self.acquired_at_robot < 0
                or not isinstance(self.source_event_ids, tuple)
                or not 1 <= len(self.source_event_ids) <= 16
                or any(not isinstance(i, str) or not i.strip() or len(i) > 256
                       for i in self.source_event_ids)
                or len(set(self.source_event_ids)) != len(self.source_event_ids)):
            raise AdapterError('invalid bounded strategic window')


@dataclass(frozen=True)
class AdapterOutcome:
    status: str
    goal_revision: int
    motor_authority: bool = field(default=False, init=False)


class StrategicModelSession:
    """Serial inference with newest-only pending window; no model execution policy.

    The provider is an injected, persistent session object/callable. A completed
    response never mutates the goal without a current exact journal/world/map
    check and StrategicOwner's typed proposal validation.
    """

    def __init__(self, session: AgentSession, journal: EventJournal,
                 provider: Callable[[dict], dict]):
        if (not isinstance(session, AgentSession) or session.boot_id is None
                or not isinstance(journal, EventJournal) or not callable(provider)):
            raise AdapterError('model adapter requires boot, journal and provider')
        self.session, self.journal, self.provider = session, journal, provider
        self.owner = StrategicOwner(session)
        self._lock = threading.RLock()
        self._in_flight = False
        self._highest_sequence = -1
        self._pending: tuple[ModelWindow, dict, int] | None = None

    def _evidence(self, window: ModelWindow) -> dict:
        try:
            records = {event.event_id: event for event in self.journal.replay()}
        except JournalError as exc:
            raise AdapterError('journal is corrupt or incomplete') from exc
        result = {}
        for event_id in window.source_event_ids:
            event = records.get(event_id)
            if (event is None or
                    (event.robot_id, event.session_id, event.boot_id) !=
                    (self.session.robot_id, self.session.session_id, self.session.boot_id)
                    or (event.mission_id, event.lease_id) !=
                    (self.session.mission_id, self.session.execution['lease_id'])
                    or (event.map_epoch, event.map_revision) !=
                    (window.map_epoch, window.map_revision)
                    or event.acquired_at_robot > window.acquired_at_robot):
                raise AdapterError('missing, future, or wrong-owner journal evidence')
            encoded = json.dumps(event.to_dict(), sort_keys=True, separators=(',', ':'),
                                 allow_nan=False).encode('utf-8')
            result[event_id] = dict(source=event.source, event_kind=event.event_kind,
                acquired_at_robot=event.acquired_at_robot,
                timestamp_quality=event.timestamp_quality,
                fingerprint=sha256(encoded).hexdigest())
        return result

    def offer(self, window: ModelWindow) -> None:
        if not isinstance(window, ModelWindow):
            raise AdapterError('expected validated strategic window')
        with self._lock, self.session._strategy_lock:
            if window.sequence <= self._highest_sequence:
                raise AdapterError('stale or duplicate strategic window')
            evidence = self._evidence(window)
            self._highest_sequence = window.sequence
            self._pending = (window, evidence, self.session.world.sequence)

    def run_one(self, *, now_robot: Callable[[], float],
                current_map: Callable[[], tuple[int, int | None]]) -> AdapterOutcome:
        with self._lock:
            if self._in_flight:
                return AdapterOutcome('busy', self.session.strategy['goal_revision'])
            if self._pending is None:
                return AdapterOutcome('idle', self.session.strategy['goal_revision'])
            window, evidence, world_sequence = self._pending
            self._pending = None
            self._in_flight = True
        try:
            with self.session._strategy_lock:
                if (self.session.world.sequence != world_sequence or
                        current_map() != (window.map_epoch, window.map_revision)):
                    return AdapterOutcome('stale', self.session.strategy['goal_revision'])
                context = self.owner.build_context()
                context['window'] = dict(sequence=window.sequence,
                    acquired_at_robot=window.acquired_at_robot,
                    map_epoch=window.map_epoch, map_revision=window.map_revision,
                    source_event_ids=list(window.source_event_ids),
                    # No full media/person payload enters the model by default.
                    events=[dict(event_id=i, source=evidence[i]['source'],
                                 event_kind=evidence[i]['event_kind'],
                                 acquired_at_robot=evidence[i]['acquired_at_robot'],
                                 timestamp_quality=evidence[i]['timestamp_quality'])
                            for i in window.source_event_ids])
            try:
                raw = self.provider(context)
            except Exception:
                return AdapterOutcome('model_error', self.session.strategy['goal_revision'])
            with self._lock:
                if self._pending is not None:
                    return AdapterOutcome('superseded', self.session.strategy['goal_revision'])
                with self.session._strategy_lock:
                    try:
                        if (self.session.world.sequence != world_sequence or
                                current_map() != (window.map_epoch, window.map_revision)
                                or self._evidence(window) != evidence):
                            return AdapterOutcome('stale', self.session.strategy['goal_revision'])
                        candidate = GoalProposal.from_dict(raw)
                        self.owner.submit(candidate, now_robot=now_robot(),
                            map_epoch=window.map_epoch, map_revision=window.map_revision,
                            known_event_ids=frozenset(evidence))
                    except (AdapterError, StrategyError, ValueError, TypeError):
                        return AdapterOutcome('rejected', self.session.strategy['goal_revision'])
                    return AdapterOutcome('admitted', self.session.strategy['goal_revision'])
        finally:
            with self._lock:
                self._in_flight = False
