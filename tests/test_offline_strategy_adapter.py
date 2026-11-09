"""Offline model-session and atomic session/coordinator checkpoint regressions."""
import json
import tempfile
import threading
import unittest
from pathlib import Path
from unittest.mock import patch

from beep_agent.coordinator import CoordinatorContext, GoalCoordinator
from beep_agent.events import EventEnvelope
from beep_agent.journal import EventJournal
from beep_agent.session import AgentSession
from beep_agent.strategy import StrategicOwner
from beep_agent.checkpoint import CombinedCheckpoint, CheckpointError
from beep_agent.model_session import StrategicModelSession, ModelWindow, AdapterError


def session_and_owner():
    session = AgentSession(robot_id='beep', mission_id=None, boot_id='b', session_id='s')
    return session, GoalCoordinator(session)


def event(event_id, at):
    return EventEnvelope.from_dict(dict(schema_version=1, event_id=event_id,
        event_kind='attention', source='system', robot_id='beep', session_id='s',
        boot_id='b', mission_id=None, lease_id=None, acquired_at_robot=at,
        received_at_robot=at, robot_clock='boot_monotonic',
        timestamp_quality='sensor_acquisition', acquired_at_ros_ns=None,
        map_epoch=1, map_revision=2, map_revision_status='verified',
        map_snapshot_id='grid', capture_local_transform=None,
        evidence_id=None, payload={'note': 'synthetic'}))


def proposal(context, *, proposal_id='self-1'):
    return dict(schema_version=1, proposal_id=proposal_id, session_id='s',
        boot_id='b', map_epoch=context['window']['map_epoch'],
        map_revision=context['window']['map_revision'],
        base_goal_revision=context['goal_revision'],
        observed_world_sequence=context['world']['sequence'], goal_name='explore',
        target_track_id=None, issued_at_robot=1., expires_at_robot=3.,
        source_event_ids=context['window']['source_event_ids'],
        reason='Explore the verified known-free region.')


def window(number, *, evidence=('e1',), acquired=1.):
    return ModelWindow(number, acquired, 1, 2, evidence)


class CheckpointTests(unittest.TestCase):
    def test_atomic_pair_restore_exact_bound_owner_and_release_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'state.json'
            session, coordinator = session_and_owner()
            # The real owner must adopt a goal; construct through the strategy contract.
            from beep_agent.strategy import GoalProposal
            StrategicOwner(session).submit(GoalProposal.from_dict(proposal(dict(
                window=dict(map_epoch=1, map_revision=2, source_event_ids=[]),
                goal_revision=0, world=dict(sequence=0)))), now_robot=2.,
                map_epoch=1, map_revision=2)
            ctx = CoordinatorContext('b', 1, 2, 'grid', True, True, 2., 'm', 'l', True)
            coordinator.bind_confirmed(ctx, goal_revision=1)
            store = CombinedCheckpoint(path)
            self.assertEqual(store.save(session, coordinator), 1)
            restored, owner = CombinedCheckpoint(path).load(expected_robot_id='beep',
                                                              expected_boot_id='b')
            self.assertEqual(owner.tick(ctx).action, 'continue_local')
            owner.complete_segment(CoordinatorContext('b', 1, 2, 'grid', True,
                True, 2.1, 'm', 'l', False), result='blocked', release_id='r1')
            self.assertEqual(CombinedCheckpoint(path).load(expected_boot_id='b')[1]
                             .tick(ctx).action, 'continue_local')  # old pair is intact
            advanced = CombinedCheckpoint(path)
            advanced.load(expected_boot_id='b')
            self.assertEqual(advanced.save(restored, owner), 2)
            again, persisted = CombinedCheckpoint(path).load(expected_boot_id='b')
            self.assertEqual(again.execution['lease_id'], None)
            self.assertEqual(persisted.to_dict()['release_ids'], ['r1'])
            with self.assertRaises(CheckpointError):
                store.save(session, coordinator)  # stale generation cannot overwrite

    def test_restore_rejects_torn_pair_wrong_boot_and_corrupt_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'state.json'
            session, owner = session_and_owner()
            CombinedCheckpoint(path).save(session, owner)
            raw = json.loads(path.read_text())
            raw['coordinator']['session_id'] = 'foreign'
            path.write_text(json.dumps(raw))
            with self.assertRaises(CheckpointError):
                CombinedCheckpoint(path).load(expected_boot_id='b')
            path.write_text('{')
            with self.assertRaises(CheckpointError):
                CombinedCheckpoint(path).load(expected_boot_id='b')
            path.unlink()
            CombinedCheckpoint(path).save(session, owner)
            with self.assertRaises(CheckpointError):
                CombinedCheckpoint(path).load(expected_boot_id='replacement')

    def test_writer_cannot_replace_same_generation_with_another_session(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'state.json'
            session, owner = session_and_owner()
            store = CombinedCheckpoint(path)
            store.save(session, owner)
            foreign = AgentSession(robot_id='beep', mission_id=None,
                                   boot_id='b', session_id='foreign')
            with self.assertRaises(CheckpointError):
                store.save(foreign, GoalCoordinator(foreign))
            restored, _ = CombinedCheckpoint(path).load(expected_boot_id='b')
            self.assertEqual(restored.session_id, 's')

    def test_failed_replace_preserves_previous_pair_and_generation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'state.json'
            session, owner = session_and_owner()
            store = CombinedCheckpoint(path)
            store.save(session, owner)
            before = path.read_bytes()
            with patch('beep_agent.checkpoint.os.replace', side_effect=OSError('disk fault')):
                with self.assertRaises(OSError):
                    store.save(session, owner)
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(store.generation, 1)
            self.assertEqual(CombinedCheckpoint(path).load(expected_boot_id='b')[0].session_id, 's')


class AdapterTests(unittest.TestCase):
    def test_newer_window_supersedes_inflight_and_one_provider_session_is_reused(self):
        with tempfile.TemporaryDirectory() as directory:
            session, coordinator = session_and_owner()
            journal = EventJournal(Path(directory) / 'events.jsonl')
            journal.append(event('e1', 1.))
            journal.append(event('e2', 1.02))
            entered, release = threading.Event(), threading.Event()
            calls = []
            def provider(context):
                calls.append(context['window']['sequence'])
                if len(calls) == 1:
                    entered.set()
                    self.assertTrue(release.wait(3))
                return proposal(context, proposal_id=f'self-{len(calls)}')
            adapter = StrategicModelSession(session, journal, provider)
            adapter.offer(window(1))
            first = []
            assert adapter._pending is not None
            self.assertNotIn('synthetic', json.dumps(adapter._pending[1]))
            thread = threading.Thread(target=lambda: first.append(adapter.run_one(
                now_robot=lambda: 2., current_map=lambda: (1, 2))))
            thread.start()
            self.assertTrue(entered.wait(3))
            self.assertEqual(adapter.run_one(now_robot=lambda: 2.,
                                             current_map=lambda: (1, 2)).status, 'busy')
            adapter.offer(window(2, evidence=('e2',), acquired=1.02))
            release.set()
            thread.join(3)
            self.assertFalse(thread.is_alive())
            self.assertEqual(first[0].status, 'superseded')
            self.assertEqual(session.strategy['goal_revision'], 0)
            accepted = adapter.run_one(now_robot=lambda: 2., current_map=lambda: (1, 2))
            self.assertEqual(accepted.status, 'admitted')
            self.assertFalse(accepted.motor_authority)
            self.assertEqual(calls, [1, 2])
            self.assertEqual(session.strategy['goal_revision'], 1)
            self.assertEqual(coordinator.tick(CoordinatorContext('b', 1, 2, 'grid',
                True, True, 2., None, None, False)).action, 'plan_next_segment')

    def test_missing_or_wrong_owner_evidence_fails_before_model_call(self):
        with tempfile.TemporaryDirectory() as directory:
            session, _ = session_and_owner()
            journal = EventJournal(Path(directory) / 'events.jsonl')
            calls = []
            adapter = StrategicModelSession(session, journal,
                                            lambda ctx: (calls.append(ctx) or {}))
            with self.assertRaises(AdapterError):
                adapter.offer(window(1))
            journal.append(event('e1', 1.))
            with self.assertRaises(AdapterError):
                adapter.offer(window(1, acquired=.9))
            adapter.offer(window(1))
            with self.assertRaises(AdapterError):
                adapter.offer(window(1))
            self.assertEqual(calls, [])

    def test_pending_windows_coalesce_and_malformed_provider_cannot_admit(self):
        with tempfile.TemporaryDirectory() as directory:
            session, _ = session_and_owner()
            journal = EventJournal(Path(directory) / 'events.jsonl')
            journal.append(event('e1', 1.))
            calls = []
            def provider(ctx):
                calls.append(ctx['window']['sequence'])
                return {'goal_name': 'raw_motor_command'}
            adapter = StrategicModelSession(session, journal, provider)
            adapter.offer(window(1))
            adapter.offer(window(2))
            self.assertEqual(adapter.run_one(now_robot=lambda: 2.,
                current_map=lambda: (1, 2)).status, 'rejected')
            self.assertEqual(calls, [2])
            self.assertEqual(session.strategy['goal_revision'], 0)
            with self.assertRaises(AdapterError):
                ModelWindow(3, 1., 1, 2, ['e1'])  # type: ignore[arg-type]  # Mutable evidence is forbidden.

    def test_journal_rewrite_world_change_and_map_change_drop_reply(self):
        for change in ('journal', 'world', 'map'):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as directory:
                session, _ = session_and_owner()
                journal = EventJournal(Path(directory) / 'events.jsonl')
                journal.append(event('e1', 1.))
                def provider(context):
                    if change == 'journal':
                        journal.path.write_text('')
                    if change == 'world':
                        session.world.sequence += 1
                    return proposal(context)
                adapter = StrategicModelSession(session, journal, provider)
                adapter.offer(window(1))
                result = adapter.run_one(now_robot=lambda: 2.,
                    current_map=lambda: (1, 3) if change == 'map' else (1, 2))
                self.assertNotEqual(result.status, 'admitted')
                self.assertEqual(session.strategy['goal_revision'], 0)


if __name__ == '__main__':
    unittest.main()
