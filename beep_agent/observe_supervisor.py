"""Host-only observe supervisor; deliberately has no motor or bridge adapter.

Cross-file journal/checkpoint persistence is not atomic. If an append exists
without its corresponding session checkpoint, refuse replay and reconcile
explicitly rather than inventing an exactly-once state transition.
"""
from __future__ import annotations

from .checkpoint import CombinedCheckpoint
from .coordinator import CoordinatorContext, GoalCoordinator
from .events import EventEnvelope
from .model_session import ModelWindow, StrategicModelSession
from .phase_a import PhaseARecorder
from .session import AgentSession
from .tactical import TacticalEvent, TacticalGate


class SupervisorError(ValueError):
    pass


class ObserveOnlySupervisor:
    """Join existing offline contracts without a dispatch or live I/O surface."""

    def __init__(self, session: AgentSession, recorder: PhaseARecorder,
                 tactical: TacticalGate, model: StrategicModelSession,
                 coordinator: GoalCoordinator, checkpoint: CombinedCheckpoint):
        if (not isinstance(session, AgentSession) or session.boot_id is None
                or not isinstance(recorder, PhaseARecorder) or recorder.session is not session
                or not isinstance(tactical, TacticalGate) or tactical.session is not session
                or not isinstance(model, StrategicModelSession) or model.session is not session
                or model.journal is not recorder.journal
                or not isinstance(coordinator, GoalCoordinator) or coordinator.session is not session
                or not isinstance(checkpoint, CombinedCheckpoint)):
            raise SupervisorError('observe supervisor components must share one session/journal')
        self.session = session
        self.recorder = recorder
        self.tactical = tactical
        self.model = model
        self.coordinator = coordinator
        self.checkpoint = checkpoint

    def observe(self, event: EventEnvelope, attention: TacticalEvent,
                perception: dict, *, context: CoordinatorContext) -> dict:
        if (not isinstance(event, EventEnvelope) or not isinstance(attention, TacticalEvent)
                or not isinstance(context, CoordinatorContext) or not isinstance(perception, dict)
                or self.session.execution['lease_id'] is not None
                or context.local_motion_active or context.mission_id is not None
                or context.lease_id is not None
                or context.boot_id != self.session.boot_id
                or not context.map_verified or not context.sensors_fresh
                or (event.robot_id, event.session_id, event.boot_id) !=
                   (self.session.robot_id, self.session.session_id, self.session.boot_id)
                or (event.mission_id, event.lease_id) != (None, None)
                or (event.source, event.event_kind) != ('camera', 'frame_acquired')
                or (event.map_epoch, event.map_revision, event.map_snapshot_id) !=
                   (context.map_epoch, context.map_revision, context.map_snapshot_id)
                or not event.acquired_at_robot <= context.now_robot
                or (attention.session_id, attention.boot_id, attention.map_epoch,
                    attention.map_revision, attention.mission_id, attention.lease_id) !=
                   (event.session_id, event.boot_id, event.map_epoch,
                    event.map_revision, None, None)
                or (attention.source, attention.suggestion) !=
                   ('visual_salience', 'attention_candidate')
                or attention.evidence_ids != (event.event_id,)
                or attention.acquired_at_robot != event.acquired_at_robot):
            raise SupervisorError('observe-only identity, provenance or ownership mismatch')
        if event.event_id in self.recorder._seen:
            if self.session.world.last_packet_id != event.event_id:
                raise SupervisorError('journal ahead of checkpoint; reconciliation required')
            raise SupervisorError('duplicate observation; already checkpointed')
        status = self.recorder.revisions.status()
        if ((status['boot_id'], status['map_epoch'], status['map_revision'], status['status']) !=
            (event.boot_id, event.map_epoch, event.map_revision, event.map_revision_status)):
            raise SupervisorError('unverified map revision')
        disposition = self.tactical.ingest(attention, now_robot=context.now_robot,
            map_epoch=context.map_epoch, map_revision=context.map_revision,
            locomotion_active=False, mixer_allowed=False)
        if not disposition.accepted or disposition.motor_authority:
            raise SupervisorError('tactical event was not admitted as advisory')
        # First make acquisition durable. If the later checkpoint fails, a
        # restart sees an unmatched journal event and refuses to proceed.
        if not self.recorder.record_acquisition(event):
            raise SupervisorError('duplicate acquisition')
        self.session.update(perception, observed_at=event.acquired_at_robot,
                            packet_id=event.event_id, mission_id=None, lease_id=None)
        self.model.offer(ModelWindow(self.session.world.sequence,
            event.acquired_at_robot, event.map_epoch, event.map_revision,
            (event.event_id,)))
        result = self.model.run_one(now_robot=lambda: context.now_robot,
            current_map=lambda: (context.map_epoch, context.map_revision))
        decision = self.coordinator.tick(context)
        if result.motor_authority or decision.motor_authority:
            raise SupervisorError('advisory output unexpectedly gained motor authority')
        generation = self.checkpoint.save(self.session, self.coordinator)
        return dict(event_id=event.event_id, evidence_quality=event.timestamp_quality,
            spatial_ready=event.spatial_ready, tactical_suggestion=disposition.suggestion,
            model_status=result.status, goal_revision=result.goal_revision,
            coordinator_action=decision.action, checkpoint_generation=generation,
            motor_authority=False, dispatch_performed=False, route_validated=False)
