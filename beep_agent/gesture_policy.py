"""Pure, host-only policy for proposing expressive gestures.

An allowed proposal is not permission to actuate. This policy has no network,
bridge, or SDK dependency; later admission must independently prove ownership.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

INITIAL_AUTO_GESTURES = frozenset({'wave_hand', 'pray', 'stretch'})
_AUTO_CONTEXTS = {
    'social_greeting': 'wave_hand',
    'begging_attention': 'pray',
    'after_rest': 'stretch',
}
_PERSON_CONTEXTS = frozenset({'social_greeting', 'begging_attention'})


@dataclass(frozen=True)
class PolicyDecision:
    allowed: bool
    name: str | None
    reason: str
    event_id: str | None = None
    entity_id: str | None = None
    rationale: str = ''


class GesturePolicy:
    """Bound one session's *proposals*, not its physical action receipts."""

    def __init__(self, *, cooldown_s: float = 20., max_per_session: int = 3) -> None:
        if (type(cooldown_s) not in (int, float) or not math.isfinite(cooldown_s)
                or not 0 < cooldown_s <= 3600 or type(max_per_session) is not int
                or not 1 <= max_per_session <= 100):
            raise ValueError('invalid gesture policy limits')
        self.cooldown_s = float(cooldown_s)
        self.max_per_session = max_per_session
        self.used = 0
        self.last_at: float | None = None
        self.seen_events: set[str] = set()
        self.seen_turns: set[tuple[str, str]] = set()

    def consider(self, context: str, *, event_id: str, entity_id: str | None,
                 observed_at: float, now: float, reason: str,
                 conversation_turn_id: str | None = None) -> PolicyDecision:
        name = _AUTO_CONTEXTS.get(context)
        if name is None or name not in INITIAL_AUTO_GESTURES:
            return PolicyDecision(False, None, 'context_not_auto_allowed')
        if (not isinstance(event_id, str) or not 0 < len(event_id.strip()) <= 128
                or not isinstance(reason, str) or not 0 < len(reason.strip()) <= 240
                or type(now) not in (int, float) or type(observed_at) not in (int, float)
                or not math.isfinite(now) or not math.isfinite(observed_at)
                or not 0 <= now-observed_at <= 5):
            return PolicyDecision(False, name, 'stale_evidence')
        if context in _PERSON_CONTEXTS and (not isinstance(entity_id, str)
                                             or not 0 < len(entity_id.strip()) <= 128):
            return PolicyDecision(False, name, 'entity_required')
        if event_id in self.seen_events:
            return PolicyDecision(False, name, 'duplicate_event')
        turn = ((entity_id or ''), conversation_turn_id) if conversation_turn_id else None
        if turn is not None and turn in self.seen_turns:
            return PolicyDecision(False, name, 'same_conversation_turn')
        if self.used >= self.max_per_session:
            return PolicyDecision(False, name, 'session_budget')
        if self.last_at is not None and now-self.last_at < self.cooldown_s:
            return PolicyDecision(False, name, 'cooldown')
        self.used += 1
        self.last_at = float(now)
        self.seen_events.add(event_id)
        if turn is not None:
            self.seen_turns.add(turn)
        return PolicyDecision(True, name, 'proposed_shadow_only', event_id, entity_id, reason.strip())

    def to_dict(self) -> dict:
        return {'schema_version': 1, 'cooldown_s': self.cooldown_s,
                'max_per_session': self.max_per_session, 'used': self.used,
                'last_at': self.last_at, 'seen_events': sorted(self.seen_events),
                'seen_turns': [list(t) for t in sorted(self.seen_turns)]}

    @classmethod
    def from_dict(cls, data: dict) -> 'GesturePolicy':
        if not isinstance(data, dict) or set(data) != {
            'schema_version', 'cooldown_s', 'max_per_session', 'used', 'last_at',
            'seen_events', 'seen_turns'} or data['schema_version'] != 1:
            raise ValueError('invalid gesture policy checkpoint')
        obj = cls(cooldown_s=data['cooldown_s'], max_per_session=data['max_per_session'])
        used, last_at = data['used'], data['last_at']
        events, turns = data['seen_events'], data['seen_turns']
        if (type(used) is not int or not 0 <= used <= obj.max_per_session
                or (last_at is not None and (type(last_at) not in (int, float) or not math.isfinite(last_at)))
                or (used == 0) != (last_at is None)
                or not isinstance(events, list) or not isinstance(turns, list)
                or len(events) != used or len(turns) > used
                or any(not isinstance(e, str) or not e or len(e) > 128 for e in events)
                or any(not isinstance(t, list) or len(t) != 2
                       or any(not isinstance(v, str) or not v or len(v) > 128 for v in t)
                       for t in turns)
                or len(set(events)) != len(events)
                or len({tuple(t) for t in turns}) != len(turns)):
            raise ValueError('invalid gesture policy checkpoint state')
        obj.used, obj.last_at = used, last_at
        obj.seen_events = set(events)
        obj.seen_turns = {tuple(t) for t in turns}
        return obj
