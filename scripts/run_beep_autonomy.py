#!/usr/bin/env python3
"""Offline observe-only supervisor replay. No robot, socket, ROS, ALSA or motor I/O.

Historical camera/model packets are qualitative; boot time, map, owner and
journal envelopes below are synthetic fixtures, not measured provenance.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from beep_agent.checkpoint import CombinedCheckpoint  # noqa: E402
from beep_agent.coordinator import CoordinatorContext, GoalCoordinator  # noqa: E402
from beep_agent.events import EventEnvelope  # noqa: E402
from beep_agent.journal import EventJournal  # noqa: E402
from beep_agent.map_revision import MapRevisionTracker  # noqa: E402
from beep_agent.model_session import StrategicModelSession  # noqa: E402
from beep_agent.observe_supervisor import ObserveOnlySupervisor  # noqa: E402
from beep_agent.phase_a import PhaseARecorder  # noqa: E402
from beep_agent.session import AgentSession  # noqa: E402
from beep_agent.tactical import TacticalEvent, TacticalGate  # noqa: E402

FIXTURES = ROOT / 'tests' / 'fixtures'


def fake_strategy(context: dict) -> dict:
    """A typed fixture, not a GLADIS/model call or an enduring cap objective."""
    window = context['window']
    return dict(schema_version=1, proposal_id=f"replay-goal-{window['sequence']}",
        session_id=context['session_id'], boot_id=context['boot_id'],
        map_epoch=window['map_epoch'], map_revision=window['map_revision'],
        base_goal_revision=context['goal_revision'],
        observed_world_sequence=context['world']['sequence'],
        goal_name='explore', target_track_id=None,
        issued_at_robot=window['acquired_at_robot'],
        expires_at_robot=window['acquired_at_robot'] + 2.,
        source_event_ids=window['source_event_ids'],
        reason='Explore novel space without assuming this object is a mission.')


def components(root: Path, *, restore: bool) -> ObserveOnlySupervisor:
    checkpoint = CombinedCheckpoint(root / 'state.json')
    if restore:
        session, coordinator = checkpoint.load(expected_robot_id='beep',
                                                expected_boot_id='synthetic-boot')
    else:
        session = AgentSession(robot_id='beep', mission_id=None,
                               boot_id='synthetic-boot', session_id='offline-observe')
        coordinator = GoalCoordinator(session)
    journal = EventJournal(root / 'journal.jsonl')
    revisions = MapRevisionTracker(clock=lambda: 1.5)
    recorder = PhaseARecorder(session, journal, revisions)
    recorder.observe_submaps({'synthetic-submap': (0., 0., 0., 0)},
                             boot_id='synthetic-boot', map_epoch=1, healthy=True)
    model = StrategicModelSession(session, journal, fake_strategy,
        episodes_source=lambda: [], places_source=lambda: recorder.graph.to_dict()['nodes'])
    return ObserveOnlySupervisor(session, recorder, TacticalGate(session), model,
                                 coordinator, checkpoint)


def fake_observation(index: int) -> tuple[EventEnvelope, TacticalEvent, dict, CoordinatorContext]:
    source = FIXTURES / f'cap_confirmation_window{index}.json'
    saved = json.loads(source.read_text())
    frame = saved['packet']['frames'][0]
    # No exposure/TF or true map revision exists in these saved packets.
    stamp = 1. + .2 * index
    event = EventEnvelope.from_dict(dict(schema_version=1,
        event_id='offline-frame-' + frame['frame_id'], event_kind='frame_acquired',
        source='camera', robot_id='beep', session_id='offline-observe',
        boot_id='synthetic-boot', mission_id=None, lease_id=None,
        acquired_at_robot=stamp, received_at_robot=stamp + .01,
        robot_clock='boot_monotonic', timestamp_quality='ingest_approximate',
        acquired_at_ros_ns=None, map_epoch=1, map_revision=0,
        map_revision_status='verified', map_snapshot_id='synthetic-grid',
        capture_local_transform=None, evidence_id=frame['frame_id'],
        payload={'source_fixture': source.name}))
    attention = TacticalEvent.from_dict(dict(schema_version=1,
        event_id='attention-' + frame['frame_id'],
        session_id=event.session_id, boot_id=event.boot_id,
        map_epoch=1, map_revision=0, mission_id=None, lease_id=None,
        source='visual_salience', acquired_at_robot=stamp, ttl_ms=1000,
        confidence=.8, suggestion='attention_candidate',
        bearing_hint_rad=None, evidence_ids=[event.event_id]))
    local = CoordinatorContext('synthetic-boot', 1, 0, 'synthetic-grid', True,
        True, stamp + .1, None, None, False)
    return event, attention, saved['response'], local


def replay(root: Path) -> dict:
    observations = []
    runner = components(root, restore=False)
    for index in range(2):
        # A process restart between windows exercises the actual persisted pair.
        if index:
            runner = components(root, restore=True)
        event, attention, response, local = fake_observation(index)
        observations.append(runner.observe(event, attention, response, context=local))
    assert runner.session.selection is not None
    return dict(schema_version=1, mode='offline_observe_only',
        map_and_lease_context='synthetic_not_recorded_provenance',
        recorded_fixtures=[f'cap_confirmation_window{i}.json' for i in range(2)],
        observations=observations, goal=runner.session.selection.goal.name,
        goal_revision=runner.session.strategy['goal_revision'],
        checkpoint_generation=runner.checkpoint.generation,
        journal_events=len(runner.recorder.journal.replay()),
        dispatch_performed=False, route_validated=False,
        physical_stop_verified=False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observe-only', action='store_true', required=True,
                        help='mandatory non-actuating mode; no motion mode exists')
    parser.add_argument('--output', type=Path, help='optional JSON report')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as directory:
        result = replay(Path(directory))
    payload = json.dumps(result, sort_keys=True, indent=2) + '\n'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload)
    print(payload, end='')


if __name__ == '__main__':
    main()
