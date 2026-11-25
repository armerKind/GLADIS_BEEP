"""Synthetic offline LifeLoop replay. All robot provenance here is mocked."""
import json
import io
from pathlib import Path
import runpy
import socket
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch
import urllib.request

from beep_agent.life_loop import ShadowLifeLoop, LifeLoopError

FIXTURES = Path(__file__).parent / 'fixtures'


class ShadowLoopTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.events = json.loads((FIXTURES/'life_loop_shadow_events.json').read_text())

    def test_session_crosses_lease_release_restart_and_scene_change(self):
        runner = ShadowLifeLoop(self.root, session_id='shadow-test', boot_id='synthetic-boot')
        first = [runner.process(e) for e in self.events[:4]]
        self.assertEqual(first[2]['kind'],'released')
        self.assertTrue(first[3]['policy_allowed'])
        self.assertFalse(first[3]['dispatch_performed'])
        sid=runner.session.session_id
        restarted = ShadowLifeLoop(self.root, restore=True, boot_id='synthetic-boot')
        self.assertEqual(restarted.session.session_id,sid)
        rest = [restarted.process(e) for e in self.events[4:]]
        self.assertEqual(rest[0]['blocked_reason'],'duplicate_event')
        self.assertEqual(rest[2]['blocked_reason'],'speaking_turn')
        self.assertEqual(rest[-1]['blocked_reason'],'stale_evidence')
        self.assertEqual(restarted.session.execution['lease_id'],None)
        self.assertEqual(restarted.session.strategy['goal_revision'],1)
        self.assertEqual(restarted.http_paths,[])
        self.assertTrue(all(not item['dispatch_performed'] for item in first+rest))

    def test_wrong_release_and_boot_fail_closed_without_dispatch(self):
        runner=ShadowLifeLoop(self.root, session_id='shadow-test',boot_id='synthetic-boot')
        runner.process(self.events[0])
        runner.process(self.events[1])
        bad={**self.events[2], 'lease_id':'other', 'event_id':'bad-release'}
        with self.assertRaises(LifeLoopError):runner.process(bad)
        self.assertEqual(runner.session.execution['lease_id'],'lease-1')
        runner.process(self.events[2])
        with self.assertRaises(LifeLoopError):
            ShadowLifeLoop(self.root,restore=True,boot_id='different-boot')
        self.assertEqual(runner.http_paths,[])

    def test_stationary_evidence_is_separate_from_moving_observer(self):
        runner=ShadowLifeLoop(self.root,session_id='shadow-test',boot_id='synthetic-boot')
        runner.process(self.events[0]);runner.process(self.events[1])
        premature={**self.events[3],'event_id':'premature'}
        self.assertEqual(runner.process(premature)['blocked_reason'],'motion_owner_active')
        runner.process(self.events[2])
        wrong={**self.events[3],'event_id':'wrong', 'release_id':'other'}
        self.assertEqual(runner.process(wrong)['blocked_reason'],'release_mismatch')
        self.assertEqual(runner.http_paths,[])

    def test_real_cli_replays_checkpoint_with_network_calls_forbidden(self):
        script=Path(__file__).resolve().parents[1]/'scripts'/'run_beep_life_loop.py'
        argv=['run_beep_life_loop.py','--output-root',str(self.root)]
        output=io.StringIO()
        with patch.object(sys,'argv',argv), redirect_stdout(output), \
             patch.object(socket.socket,'connect',side_effect=AssertionError('network forbidden')), \
             patch.object(urllib.request,'urlopen',side_effect=AssertionError('HTTP forbidden')):
            runpy.run_path(str(script),run_name='__main__')
        report=json.loads(output.getvalue())
        trace=[json.loads(line) for line in (self.root/'life_loop_trace.jsonl').read_text().splitlines()]
        self.assertEqual(len(trace),report['events'])
        self.assertEqual(report['http_paths'],[])
        self.assertFalse(any(row['dispatch_performed'] or row['motor_authority'] for row in trace))
        self.assertEqual([row['blocked_reason'] for row in trace if row['blocked_reason']],
                         ['duplicate_event','speaking_turn','stale_evidence'])


if __name__=='__main__':unittest.main()
