"""Read-only bridge-to-Stack-2 adapter for the live moving-eyes runner.

No dispatch method. Camera time is request-start ingest-approximate, never exposure.
The bridge is authoritative for submap revision; this process does not invent one.
"""
from __future__ import annotations

from pathlib import Path

from .checkpoint import CombinedCheckpoint
from .coordinator import CoordinatorContext, GoalCoordinator
from .events import EventEnvelope
from .executive import EmbodiedGoal
from .journal import EventJournal
from .map_revision import MapRevisionTracker
from .model_session import StrategicModelSession
from .observe_supervisor import ObserveOnlySupervisor
from .phase_a import PhaseARecorder
from .session import AgentSession
from .tactical import TacticalEvent, TacticalGate


class LiveEvidenceUnavailable(ValueError):
    pass


class BridgeRevisionView(MapRevisionTracker):
    """Proxy the bridge's actual SubmapList-derived status, not a fabricated submap."""
    def __init__(self, bridge_status: dict):
        self.update(bridge_status)

    def update(self, bridge_status: dict) -> None:
        slam = bridge_status.get('slam') or {}
        self._status = dict(boot_id=bridge_status.get('boot_id'),
            map_epoch=slam.get('map_epoch'), map_revision=slam.get('map_revision'),
            status=slam.get('map_revision_status'))

    def status(self) -> dict:
        return dict(self._status)


def _identity(status: dict) -> tuple:
    slam = status.get('slam') or {}
    return (status.get('boot_id'), slam.get('map_epoch'), slam.get('map_revision'),
            slam.get('map_revision_status'), slam.get('map_snapshot_id'))


def _context(status: dict, mission: dict, slam_map: dict) -> CoordinatorContext:
    slam = status.get('slam') or {}
    identity = _identity(status)
    if (not isinstance(mission, dict) or mission.get('state') not in ('starting', 'running')
            or not isinstance(mission.get('id'), str) or not isinstance(mission.get('lease_id'), str)
            or mission['lease_id'] != status.get('motion_lease_id')
            or identity[0] != slam_map.get('boot_id')
            or identity[1:4] != (slam_map.get('map_epoch'), slam_map.get('map_revision'),
                                  slam_map.get('map_revision_status'))
            or identity[4] != slam_map.get('map_snapshot_id')
            or identity[3] != 'verified' or not slam_map.get('ok')
            or slam.get('usable') is not True
            or not isinstance(status.get('scan_age_s'), (int, float))
            or status['scan_age_s'] < 0 or status['scan_age_s'] > 0.6
            or not isinstance(status.get('robot_monotonic_s'), (int, float))):
        raise LiveEvidenceUnavailable('bridge mission, lease, map or sensors unverified')
    return CoordinatorContext(identity[0], identity[1], identity[2], identity[4],
        True, True, status['robot_monotonic_s'], mission['id'], mission['lease_id'], True)


class LiveBridgeObserver:
    def __init__(self, directory: Path, status: dict, mission: dict, slam_map: dict,
                 *, resume_checkpoint: Path | None = None,
                 fixed_goal: EmbodiedGoal | None = None):
        context = _context(status, mission, slam_map)
        self.directory = Path(directory if resume_checkpoint is None else resume_checkpoint.parent)
        self.checkpoint = CombinedCheckpoint(self.directory / 'stack2_checkpoint.json'
            if resume_checkpoint is None else resume_checkpoint)
        self.journal = EventJournal(self.directory / 'event_journal.jsonl')
        self.revisions = BridgeRevisionView(status)
        if resume_checkpoint is None:
            self.session = AgentSession(robot_id='BEEP', mission_id=None,
                                        boot_id=context.boot_id, fixed_goal=fixed_goal)
            self.coordinator = GoalCoordinator(self.session)
            self.coordinator.observe_existing_owner(context)
            self.checkpoint.save(self.session, self.coordinator)
        else:
            self.session, self.coordinator = self.checkpoint.load(
                expected_robot_id='BEEP', expected_boot_id=context.boot_id)
            if (self.session.mission_id, self.session.execution['lease_id']) != (
                    context.mission_id, context.lease_id):
                raise LiveEvidenceUnavailable('checkpoint belongs to another live mission/lease')
            if fixed_goal is not None and self.session.fixed_goal != fixed_goal:
                raise LiveEvidenceUnavailable('checkpoint fixed goal differs from requested goal')
        self.recorder = PhaseARecorder(self.session, self.journal, self.revisions)
        self.tactical = TacticalGate(self.session)
        self._response: dict | None = None
        self._now = context.now_robot
        self.model = StrategicModelSession(self.session, self.journal, self._vision_proposal)
        self.supervisor = ObserveOnlySupervisor(self.session, self.recorder, self.tactical,
                                                self.model, self.coordinator, self.checkpoint)

    def _vision_proposal(self, context: dict) -> dict | None:
        """Limited vision-derived strategic suggestion, not a GLADIS model session."""
        response = self._response
        if response is None or self.session.fixed_goal is not None:
            return None
        scene = response.get('scene') or {}
        # A scene hazard is attention, not an authority to physically stop.
        goal = 'observe' if (response.get('escalate') or scene.get('hazards')
                             or scene.get('people') or response.get('confidence', 0) < .65) else 'explore'
        if (self.session.strategy['goal_revision'] and self.session.selection is not None
                and self.session.selection.goal.name == goal):
            return None
        window = context['window']
        return dict(schema_version=1, proposal_id='vision-' + window['source_event_ids'][-1],
            session_id=context['session_id'], boot_id=context['boot_id'],
            map_epoch=window['map_epoch'], map_revision=window['map_revision'],
            base_goal_revision=context['goal_revision'],
            observed_world_sequence=context['world']['sequence'], goal_name=goal,
            target_track_id=None, issued_at_robot=self._now,
            expires_at_robot=self._now + 8.0,
            source_event_ids=window['source_event_ids'],
            reason='Advisory goal derived from the live vision scene; local motion remains autonomous.')

    def observe(self, *, frame_id: str, before: dict, after: dict, current: dict,
                mission: dict, slam_map: dict, perception: dict) -> dict:
        context = _context(current, mission, slam_map)
        previous = _identity(before)
        if (previous != _identity(after) or previous[:4] != _identity(current)[:4]
                or before.get('motion_lease_id') != context.lease_id
                or after.get('motion_lease_id') != context.lease_id
                or before.get('boot_id') != context.boot_id
                or after.get('capture_ownership_unverified')
                or self.session.boot_id != context.boot_id
                or (self.session.mission_id, self.session.execution['lease_id']) !=
                   (context.mission_id, context.lease_id)):
            raise LiveEvidenceUnavailable('capture crossed a bridge owner or map transition')
        acquired = before.get('robot_monotonic_s')
        received = after.get('robot_monotonic_s')
        if (not isinstance(acquired, (int, float)) or not isinstance(received, (int, float))
                or not 0 <= acquired <= received <= context.now_robot
                or context.now_robot - acquired > 10.0):
            raise LiveEvidenceUnavailable('camera request-start robot clock missing or stale')
        self.revisions.update(current)
        event_id = f'frame:{frame_id}'
        event = EventEnvelope.from_dict(dict(schema_version=1, event_id=event_id,
            event_kind='frame_acquired', source='camera', robot_id='BEEP',
            session_id=self.session.session_id, boot_id=context.boot_id,
            mission_id=context.mission_id, lease_id=context.lease_id,
            acquired_at_robot=acquired, received_at_robot=received,
            robot_clock='boot_monotonic', timestamp_quality='ingest_approximate',
            acquired_at_ros_ns=None, map_epoch=context.map_epoch,
            map_revision=context.map_revision, map_revision_status='verified',
            map_snapshot_id=previous[4], capture_local_transform=None,
            evidence_id=frame_id, payload={'frame_id': frame_id,
                'scene_summary': perception['scene']['summary'][:512]}))
        attention = TacticalEvent.from_dict(dict(schema_version=1,
            event_id='attention:' + frame_id, session_id=self.session.session_id,
            boot_id=context.boot_id, map_epoch=context.map_epoch,
            map_revision=context.map_revision, mission_id=context.mission_id,
            lease_id=context.lease_id, source='visual_salience',
            acquired_at_robot=acquired, ttl_ms=10000,
            confidence=perception['confidence'], suggestion='attention_candidate',
            bearing_hint_rad=None, evidence_ids=[event_id]))
        self._response, self._now = perception, context.now_robot
        return self.supervisor.observe(event, attention, perception, context=context)
