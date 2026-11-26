"""Host-only supervised-wave CLI against the real loopback bridge handler."""
from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

from beep_bridge import beep_bridge as bridge
from scripts import run_beep_life_loop


class SupervisedWaveHttpTests(unittest.TestCase):
    def setUp(self):
        with bridge.state_lock:
            self.original_state = deepcopy(bridge.state)
        bridge._reset_motion_state_for_tests()
        bridge._reset_mission_state_for_tests()
        now = time.time()
        with bridge.state_lock:
            bridge.state.update(moving=False, posture='standing', posture_uncertain=False,
                                scan_seen=True, scan_at=now)
            bridge.state['slam'].update(pose_at=now, map_at=now, pose_valid=True,
                                        map_width=4, map_height=4, resolution_m=.1)
        self.server = bridge.ThreadingHTTPServer(('127.0.0.1', 0), bridge.Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.origin = f'http://127.0.0.1:{self.server.server_port}'

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(3)
        bridge._reset_motion_state_for_tests()
        bridge._reset_mission_state_for_tests()
        with bridge.state_lock:
            bridge.state.clear()
            bridge.state.update(self.original_state)

    def test_supervised_wave_cli_uses_current_bridge_contract_without_hardware(self):
        sdk_calls = []
        def fake_sdk(**kwargs):
            sdk_calls.append((kwargs.get('name'), kwargs.get('action_id')))
            return {'ok': True, 'dry_run': False,
                    'trick': bridge.resolve_trick(name=kwargs.get('name'),
                                                  action_id=kwargs.get('action_id'))}
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            audio = root / 'greeting.mp3'
            audio.write_bytes(b'ID3' + b'pre-rendered-fixture')
            output = root / 'result'
            argv = ['run_beep_life_loop.py', '--supervised-wave', '--bridge-url', self.origin,
                    '--memory-file', str(root / 'memory_bank.json'), '--output-root', str(output),
                    '--audio-file', str(audio), '--physical-confirmed', '--no-cloud-vision']
            played = Mock(return_value=True)
            text_output = io.StringIO()
            with (patch.object(bridge.map_revisions, 'status', return_value={
                    'map_revision': 2, 'status': 'verified'}),
                  patch.object(bridge, 'sdk_trick', side_effect=fake_sdk),
                  patch('scripts.run_social_voice.bridge_playback', played),
                  patch.object(sys, 'argv', argv), redirect_stdout(text_output)):
                run_beep_life_loop.main()
            report = json.loads((output / 'supervised_wave_result.json').read_text())
            self.assertEqual(json.loads(text_output.getvalue()), report)
            self.assertTrue(report['dispatch_performed'])
            self.assertTrue(report['audio_confirmed'])
            self.assertTrue(report['recovery']['software_standing'])
            self.assertTrue(report['recovery']['slam_usable'])
            self.assertTrue(report['no_cloud_vision'])
            self.assertFalse(report['physical_posture_verified'])
            self.assertEqual(sdk_calls, [('wave_hand', None), (None, bridge.POSTURE_ACTIONS['stand'])])
            played.assert_called_once_with(audio.read_bytes(), 3.0, self.origin)
            self.assertIsNone(bridge.snapshot()['motion_lease_id'])
            self.assertEqual(bridge.snapshot()['posture'], 'standing')


if __name__ == '__main__':
    unittest.main()
