"""Full host-only observe loop: saved qualitative frames, no robot or dispatch."""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from beep_agent.checkpoint import CombinedCheckpoint
from beep_agent.coordinator import CoordinatorContext, GoalCoordinator
from beep_agent.events import EventEnvelope
from beep_agent.journal import EventJournal
from beep_agent.map_revision import MapRevisionTracker
from beep_agent.model_session import StrategicModelSession
from beep_agent.phase_a import PhaseARecorder
from beep_agent.session import AgentSession
from beep_agent.tactical import TacticalEvent, TacticalGate
from beep_agent.observe_supervisor import ObserveOnlySupervisor, SupervisorError

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / 'tests' / 'fixtures'


def observation(index):
    saved = json.loads((FIXTURES / f'cap_confirmation_window{index}.json').read_text())
    frame = saved['packet']['frames'][0]
    at = 1. + index * .2  # synthetic boot-clock time, NOT recorded exposure
    event = EventEnvelope.from_dict(dict(schema_version=1,
        event_id='replay-' + frame['frame_id'], event_kind='frame_acquired',
        source='camera', robot_id='beep', session_id='s', boot_id='b',
        mission_id=None, lease_id=None, acquired_at_robot=at,
        received_at_robot=at + .01, robot_clock='boot_monotonic',
        timestamp_quality='ingest_approximate', acquired_at_ros_ns=None,
        map_epoch=1, map_revision=0, map_revision_status='verified',
        map_snapshot_id='synthetic-grid', capture_local_transform=None,
        evidence_id=frame['frame_id'], payload={'fixture': f'cap_confirmation_window{index}.json'}))
    tactical = TacticalEvent.from_dict(dict(schema_version=1,
        event_id='attention-' + frame['frame_id'], session_id='s', boot_id='b',
        map_epoch=1, map_revision=0, mission_id=None, lease_id=None,
        source='visual_salience', acquired_at_robot=at, ttl_ms=1000,
        confidence=.8, suggestion='attention_candidate', bearing_hint_rad=None,
        evidence_ids=[event.event_id]))
    return event, tactical, saved['response']


def provider(context):
    return dict(schema_version=1, proposal_id='goal-' + str(context['window']['sequence']),
        session_id=context['session_id'], boot_id=context['boot_id'],
        map_epoch=context['window']['map_epoch'], map_revision=context['window']['map_revision'],
        base_goal_revision=context['goal_revision'],
        observed_world_sequence=context['world']['sequence'], goal_name='explore',
        target_track_id=None, issued_at_robot=context['window']['acquired_at_robot'],
        expires_at_robot=context['window']['acquired_at_robot'] + 2.,
        source_event_ids=context['window']['source_event_ids'],
        reason='Explore the next verified known-free region, advisory only.')


def build(root, *, restored=False):
    checkpoint = CombinedCheckpoint(root / 'state.json')
    if restored:
        session, coordinator = checkpoint.load(expected_robot_id='beep', expected_boot_id='b')
    else:
        session = AgentSession(robot_id='beep', mission_id=None, boot_id='b', session_id='s')
        coordinator = GoalCoordinator(session)
    journal = EventJournal(root / 'events.jsonl')
    revisions = MapRevisionTracker(clock=lambda: 1.5)
    recorder = PhaseARecorder(session, journal, revisions)
    recorder.observe_submaps({'synthetic': (0., 0., 0., 0)},
                             boot_id='b', map_epoch=1, healthy=True)
    tactical = TacticalGate(session)
    model = StrategicModelSession(session, journal, provider,
        episodes_source=lambda: [], places_source=lambda: recorder.graph.to_dict()['nodes'])
    runner = ObserveOnlySupervisor(session, recorder, tactical, model,
                                   coordinator, checkpoint)
    return runner, session, journal, checkpoint


def context(now):
    return CoordinatorContext('b', 1, 0, 'synthetic-grid', True,
                              True, now, None, None, False)


class ObserveSupervisorTests(unittest.TestCase):
    def test_recorded_qualitative_loop_checkpoint_and_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runner, session, journal, checkpoint = build(root)
            first = runner.observe(*observation(0), context=context(1.1))
            self.assertEqual(first['model_status'], 'admitted')
            self.assertEqual(first['coordinator_action'], 'plan_next_segment')
            self.assertFalse(first['motor_authority'])
            self.assertFalse(first['dispatch_performed'])
            self.assertFalse(first['spatial_ready'])
            self.assertEqual(checkpoint.generation, 1)
            restored, session2, journal2, checkpoint2 = build(root, restored=True)
            self.assertEqual(session2.strategy['goal_revision'], 1)
            second = restored.observe(*observation(1), context=context(1.3))
            self.assertEqual(second['model_status'], 'admitted')
            self.assertEqual(second['goal_revision'], 2)
            self.assertEqual(checkpoint2.generation, 2)
            self.assertEqual(len(journal2.replay()), 2)
            self.assertEqual(session2.world.sequence, 2)
            self.assertEqual(restored.recorder.graph.nodes, {})
            self.assertFalse(second['route_validated'])
            with self.assertRaises(SupervisorError):
                restored.observe(*observation(1), context=context(1.3))
            self.assertEqual(checkpoint2.generation, 2)

    def test_journal_ahead_of_checkpoint_fails_closed_on_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            runner, _, journal, _ = build(root)
            runner.observe(*observation(0), context=context(1.1))
            journal.append(observation(1)[0])  # injected crash after append, before state commit
            restored, _, _, checkpoint = build(root, restored=True)
            with self.assertRaises(SupervisorError):
                restored.observe(*observation(1), context=context(1.3))
            self.assertEqual(checkpoint.generation, 1)

    def test_wrong_owner_or_map_and_active_lease_are_not_observe_only(self):
        with tempfile.TemporaryDirectory() as directory:
            runner, _, journal, checkpoint = build(Path(directory))
            event, tactical, response = observation(0)
            for bad_context in (context(1.1).__class__('b', 1, 1, 'synthetic-grid',
                                    True, True, 1.1, None, None, False),
                                CoordinatorContext('b', 1, 0, 'synthetic-grid', True,
                                    True, 1.1, 'm', 'l', True)):
                with self.assertRaises(SupervisorError):
                    runner.observe(event, tactical, response, context=bad_context)
            self.assertEqual(journal.replay(), [])
            self.assertEqual(checkpoint.generation, 0)

    def test_cli_report_is_executed_and_contains_no_hardware_claims(self):
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / 'replay.json'
            completed = subprocess.run([sys.executable, str(ROOT / 'scripts/run_beep_autonomy.py'),
                '--observe-only', '--output', str(report)], cwd=ROOT,
                capture_output=True, text=True, check=True, timeout=15)
            result = json.loads(completed.stdout)
            self.assertEqual(result, json.loads(report.read_text()))
            self.assertEqual(len(result['observations']), 2)
            self.assertEqual(result['checkpoint_generation'], 2)
            self.assertEqual(result['map_and_lease_context'], 'synthetic_not_recorded_provenance')
            self.assertFalse(result['dispatch_performed'])
            self.assertFalse(result['route_validated'])
            self.assertFalse(result['physical_stop_verified'])
            refused = subprocess.run([sys.executable, str(ROOT / 'scripts/run_beep_autonomy.py')],
                                     cwd=ROOT, capture_output=True, text=True, timeout=10)
            self.assertNotEqual(refused.returncode, 0)
            self.assertIn('--observe-only', refused.stderr)


if __name__ == '__main__':
    unittest.main()
