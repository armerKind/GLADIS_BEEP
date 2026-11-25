"""Pure, host-only policy for proposing expressive gestures.

An allowed proposal is not permission to actuate. This policy has no network,
bridge, or SDK dependency; later admission must independently prove ownership.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
from .skills import canonical_gesture

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


class GestureAdmissionError(ValueError):
    """A proposal has no authority to reach the gesture transport."""


@dataclass(frozen=True)
class GestureIntent:
    decision_id: str
    session_id: str
    boot_id: str
    name: str
    event_id: str
    observed_at_robot: float
    issued_at_robot: float
    expires_at_robot: float
    map_epoch: int
    map_revision: int
    released_mission_id: str
    released_lease_id: str
    release_id: str
    rollout_mode: str = 'shadow'
    stationary_preflight: bool = False

    def __post_init__(self):
        for field in ('decision_id', 'session_id', 'boot_id', 'event_id',
                      'released_mission_id', 'released_lease_id', 'release_id'):
            value = getattr(self, field)
            if not isinstance(value, str) or not 0 < len(value.strip()) <= 128:
                raise GestureAdmissionError(f'invalid {field}')
        if canonical_gesture(self.name) != self.name or self.name not in INITIAL_AUTO_GESTURES:
            raise GestureAdmissionError('gesture is outside the initial LifeLoop allowlist')
        if self.rollout_mode not in ('shadow', 'supervised'):
            raise GestureAdmissionError('invalid gesture rollout mode')
        if type(self.stationary_preflight) is not bool or (self.stationary_preflight and
                (self.released_mission_id, self.released_lease_id, self.release_id) !=
                ('stationary_preflight', 'none', self.decision_id)):
            raise GestureAdmissionError('invalid explicit stationary preflight identity')
        times = (self.observed_at_robot, self.issued_at_robot, self.expires_at_robot)
        if (any(type(t) not in (int, float) or not math.isfinite(t) for t in times)
                or not 0 <= times[0] <= times[1] < times[2]
                or times[2] - times[1] > 8 or times[1] - times[0] > 10
                or type(self.map_epoch) is not int or self.map_epoch < 0
                or type(self.map_revision) is not int or self.map_revision < 0):
            raise GestureAdmissionError('invalid gesture evidence time or map identity')


class GestureAdmission:
    """Host-side exact-release fence; still not a bridge or motor dispatcher.

    Physical confirmation must come from a human or independently commissioned
    posture sensor, not the bridge's generic software 'standing' field.
    """

    def __init__(self, *, session_id: str, boot_id: str,
                 released_mission_id: str, released_lease_id: str,
                 release_id: str):
        if any(not isinstance(v, str) or not v.strip() for v in (
                session_id, boot_id, released_mission_id, released_lease_id, release_id)):
            raise GestureAdmissionError('exact session and release provenance required')
        self.session_id, self.boot_id = session_id, boot_id
        self.release = (released_mission_id, released_lease_id, release_id)
        self.claimed: set[str] = set()
        self.claimed_events: set[str] = set()
        self.uncertain_reason: str | None = None
        self._claim_lock = threading.Lock()

    def check(self, intent: GestureIntent, readback: dict, *,
              physical_posture_confirmed: bool) -> None:
        if not isinstance(intent, GestureIntent) or intent.rollout_mode != 'supervised':
            raise GestureAdmissionError('gesture is not supervised')
        if (intent.session_id, intent.boot_id) != (self.session_id, self.boot_id):
            raise GestureAdmissionError('wrong session or boot')
        if (intent.released_mission_id, intent.released_lease_id,
                intent.release_id) != self.release:
            raise GestureAdmissionError('wrong exact release')
        if self.uncertain_reason is not None or physical_posture_confirmed is not True:
            raise GestureAdmissionError('physical posture unconfirmed')
        if intent.decision_id in self.claimed or intent.event_id in self.claimed_events:
            raise GestureAdmissionError('duplicate gesture intent')
        latest = readback.get('mission') if isinstance(readback, dict) else None
        if intent.stationary_preflight:
            if latest is not None:
                raise GestureAdmissionError('explicit stationary preflight has mission owner')
        elif (not isinstance(latest, dict) or latest.get('state') not in
              ('completed', 'cancelled', 'failed')
              or (latest.get('id'), latest.get('lease_id'), latest.get('release_id')) !=
                 (intent.released_mission_id, intent.released_lease_id, intent.release_id)):
            raise GestureAdmissionError('exact mission-release readback missing')
        s = readback.get('status')
        if not isinstance(s, dict):
            raise GestureAdmissionError('missing bridge status')
        slam = s.get('slam') or {}
        now, age = s.get('robot_monotonic_s'), s.get('scan_age_s')
        if (s.get('boot_id') != self.boot_id or s.get('moving') is not False
                or s.get('motion_lease_id') is not None or s.get('posture') != 'standing'
                or s.get('posture_uncertain') is not False or s.get('fall_suspected') is not False
                or type(now) not in (int, float) or not math.isfinite(now)
                or type(age) not in (int, float) or not math.isfinite(age)
                or not 0 <= age <= .6
                or not intent.issued_at_robot <= now <= intent.expires_at_robot
                or now - intent.observed_at_robot > 10
                or not isinstance(slam, dict)
                or (slam.get('map_epoch'), slam.get('map_revision'),
                    slam.get('map_revision_status')) !=
                   (intent.map_epoch, intent.map_revision, 'verified')):
            raise GestureAdmissionError('stale or unsafe stationary bridge readback')

    def claim(self, intent: GestureIntent) -> None:
        with self._claim_lock:
            if intent.decision_id in self.claimed or intent.event_id in self.claimed_events:
                raise GestureAdmissionError('gesture intent was already claimed')
            self.claimed.add(intent.decision_id)
            self.claimed_events.add(intent.event_id)

    def mark_uncertain(self, reason: str) -> None:
        self.uncertain_reason = reason or 'unknown_gesture_outcome'

    def to_dict(self) -> dict:
        return {'schema_version': 1, 'session_id': self.session_id,
                'boot_id': self.boot_id, 'release': list(self.release),
                'claimed': sorted(self.claimed),
                'claimed_events': sorted(self.claimed_events),
                'uncertain_reason': self.uncertain_reason}

    @classmethod
    def from_dict(cls, value: dict) -> 'GestureAdmission':
        if (not isinstance(value, dict) or set(value) != {
                'schema_version', 'session_id', 'boot_id', 'release',
                'claimed', 'claimed_events', 'uncertain_reason'}
                or value['schema_version'] != 1 or not isinstance(value['release'], list)
                or len(value['release']) != 3 or not isinstance(value['claimed'], list)
                or not isinstance(value['claimed_events'], list)
                or len(value['claimed']) != len(set(value['claimed']))
                or len(value['claimed_events']) != len(set(value['claimed_events']))
                or len(value['claimed']) != len(value['claimed_events'])
                or any(not isinstance(v, str) or not v for v in value['claimed']+value['claimed_events'])
                or (value['uncertain_reason'] is not None and
                    (not isinstance(value['uncertain_reason'], str) or not value['uncertain_reason']))):
            raise GestureAdmissionError('invalid admission checkpoint')
        obj = cls(session_id=value['session_id'], boot_id=value['boot_id'],
                  released_mission_id=value['release'][0],
                  released_lease_id=value['release'][1], release_id=value['release'][2])
        obj.claimed = set(value['claimed'])
        obj.claimed_events = set(value['claimed_events'])
        obj.uncertain_reason = value['uncertain_reason']
        return obj
