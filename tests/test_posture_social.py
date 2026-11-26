"""Host-only HTTP posture and stationary social-view contracts (no robot I/O)."""
import io
import json
import threading
import unittest
import urllib.error
import urllib.request
from unittest.mock import Mock, patch

from PIL import Image

from beep_bridge import beep_bridge as bridge
from beep_eyes.social_view import upward_social_turn, SocialViewError


def jpeg(color):
    output = io.BytesIO()
    Image.new('RGB', (160, 120), color).save(output, 'JPEG')
    return output.getvalue()


class PostureHttpTests(unittest.TestCase):
    def setUp(self):
        bridge._reset_motion_state_for_tests()
        bridge._reset_mission_state_for_tests()
        bridge.state['moving'] = False
        self.server = bridge.ThreadingHTTPServer(('127.0.0.1', 0), bridge.Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.origin = f'http://127.0.0.1:{self.server.server_port}'

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(1)
        bridge._reset_motion_state_for_tests()
        bridge._reset_mission_state_for_tests()
        bridge.state['moving'] = False

    def request(self, path, body=None):
        req = urllib.request.Request(self.origin + path,
            data=None if body is None else json.dumps(body).encode(),
            headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=3) as response:
                return response.status, json.load(response)
        except urllib.error.HTTPError as error:
            try:
                return error.code, json.load(error)
            finally:
                error.close()

    def test_sit_stand_and_locomotion_refusals(self):
        calls = []
        def gesture(*, action_id, settle_s):
            calls.append((action_id, settle_s))
            return {'ok': True}
        with patch.object(bridge, 'sdk_trick', side_effect=gesture), \
             patch.object(bridge, 'sdk_init', side_effect=AssertionError('no SDK for blocked movement')):
            code, sit = self.request('/posture', {'posture': 'sit'})
            self.assertEqual(code, 200)
            self.assertEqual(sit['status']['posture'], 'sitting')
            self.assertFalse(sit['status']['moving'])
            self.assertIsNone(sit['status']['motion_lease_id'])
            self.assertEqual(self.request('/status')[1]['posture'], 'sitting')
            for path, body in (('/move', {'action': 'turnleft', 'duration': .3}),
                               ('/mission/start', {'mode': 'coverage', 'duration_s': 90})):
                code, denied = self.request(path, body)
                self.assertEqual(code, 409)
                self.assertTrue(denied.get('busy') or denied.get('reason') == 'posture_sitting')
            with self.assertRaises(bridge.MotionBusy):
                bridge.run_action('forward', .2)
            with self.assertRaises(bridge.MotionBusy):
                bridge.sdk_send('turnleft')
            code, stand = self.request('/posture', {'posture': 'stand'})
            self.assertEqual(code, 200)
            self.assertEqual(stand['posture'], 'standing')
            self.assertEqual(calls, [(bridge.POSTURE_ACTIONS['sit'], 4.0),
                                     (bridge.POSTURE_ACTIONS['stand'], 2.0)])

    def test_moving_mission_and_invalid_requests_refuse_without_sdk(self):
        with patch.object(bridge, 'sdk_trick', side_effect=AssertionError('SDK must not run')):
            bridge.state['moving'] = True
            self.assertEqual(self.request('/posture', {'posture': 'sit'})[0], 409)
            bridge.state['moving'] = False
            with bridge.mission_lock:
                bridge._active_mission = {'state': 'running'}
            self.assertEqual(self.request('/posture', {'posture': 'sit'})[0], 409)
            with bridge.mission_lock:
                bridge._active_mission = None
            for body in ({'posture': 'crouch'}, {'posture': 'sit', 'duration': 1}, {}):
                self.assertEqual(self.request('/posture', body)[0], 400)
            self.assertEqual(self.request('/action', {'name': 'sit'})[0], 400)
            self.assertEqual(self.request('/action', {'name': 'reset'})[0], 400)

    def test_failed_stand_keeps_sitting_and_blocks_mission(self):
        with patch.object(bridge, 'sdk_trick', side_effect=({'ok': True}, {'ok': False})):
            self.assertEqual(self.request('/posture', {'posture': 'sit'})[0], 200)
            code, stand = self.request('/posture', {'posture': 'stand'})
            self.assertEqual(code, 409)
            self.assertEqual(stand['posture'], 'sitting')
            self.assertFalse(stand['status']['posture_action_acknowledged'])
            self.assertEqual(self.request('/posture', {'posture': 'sit'})[0], 409)
            with self.assertRaises(bridge.MotionBusy):
                bridge.begin_motion('unowned_walk')

    def test_sit_lease_excludes_mission_start_while_gesture_in_progress(self):
        entered, release = threading.Event(), threading.Event()
        def gesture(**_kwargs):
            entered.set()
            release.wait(2)
            return {'ok': True}
        result = []
        with patch.object(bridge, 'sdk_trick', side_effect=gesture):
            worker = threading.Thread(target=lambda: result.append(self.request('/posture', {'posture': 'sit'})))
            worker.start()
            self.assertTrue(entered.wait(1))
            self.assertEqual(self.request('/mission/start', {'mode': 'coverage'})[0], 409)
            release.set()
            worker.join(3)
            self.assertFalse(worker.is_alive())
            self.assertEqual(result[0][0], 200)
            self.assertEqual(bridge.state['posture'], 'sitting')

    def test_mocked_sit_frame_greeting_stand_and_failure_cleanup(self):
        calls = []
        class FakeVoice:
            def read_authority(self):
                return None, None, True
            def initiate_turn(self, *, trigger, visual_context):
                calls.append((trigger, visual_context, bridge.state['posture']))
                return {'state': 'initiated'}
        voice = FakeVoice()
        with patch.object(bridge, 'sdk_trick', return_value={'ok': True}), \
             patch.object(bridge, 'capture_frame', return_value=jpeg('gray')) as frame:
            with self.assertRaises(SocialViewError):
                upward_social_turn(self.origin, voice, lambda jpg: 'A person near BEEP.')
            self.assertEqual(bridge.state['posture'], 'standing')
            result = upward_social_turn(self.origin, voice,
                                        lambda jpg: 'A person near BEEP.', opt_in=True)
            self.assertEqual(result, {'state': 'initiated'})
            frame.assert_called_once()
            self.assertEqual(calls, [('person_noticed', 'A person near BEEP.', 'sitting')])
            self.assertEqual(bridge.state['posture'], 'standing')
            with self.assertRaisesRegex(RuntimeError, 'visual failure'):
                upward_social_turn(self.origin, voice,
                                   lambda jpg: (_ for _ in ()).throw(RuntimeError('visual failure')),
                                   opt_in=True)
            self.assertEqual(bridge.state['posture'], 'standing')

    def test_local_dark_social_frame_blocks_captioner_and_speaks_reaction(self):
        class FakeVoice:
            def read_authority(self): return None, None, True
            def synthesize(self, text):
                self.text = text
                return b'pre-rendered-audio'
            def play(self, audio):
                self.audio = audio
                return True
        voice = FakeVoice()
        caption = Mock(side_effect=AssertionError('image provider must not be called'))
        with patch.object(bridge, 'sdk_trick', return_value={'ok': True}), \
             patch.object(bridge, 'capture_frame', return_value=jpeg('black')) as frame:
            result = upward_social_turn(self.origin, voice, caption, opt_in=True)
            self.assertEqual(result['state'], 'camera_occluded')
            self.assertTrue(result['audio_confirmed'])
            self.assertEqual(result['cloud_vision_calls'], 0)
            self.assertIn('lens cap', voice.text)
            self.assertEqual(voice.audio, b'pre-rendered-audio')
            self.assertEqual(bridge.state['posture'], 'standing')
            self.assertEqual(frame.call_count, 1)
        caption.assert_not_called()
        with self.assertRaisesRegex(SocialViewError, 'no-cloud-vision'):
            upward_social_turn(self.origin, voice, caption, opt_in=True, no_cloud_vision=True)

    def test_occlusion_reaction_withholds_playback_when_consent_revoked(self):
        caption = Mock(side_effect=AssertionError('capped image must not reach captioner'))
        for revoke_at in (3, 4):
            with self.subTest(revoke_at=revoke_at):
                class FakeVoice:
                    def __init__(self):
                        self.reads = 0
                        self.synthesized = False
                        self.played = False
                    def read_authority(self):
                        self.reads += 1
                        return ('boot', 'owner', self.reads < revoke_at)
                    def synthesize(self, text):
                        self.synthesized = True
                        return b'audio'
                    def play(self, audio):
                        self.played = True
                        return True
                voice = FakeVoice()
                with patch.object(bridge, 'sdk_trick', return_value={'ok': True}), \
                     patch.object(bridge, 'capture_frame', return_value=jpeg('black')):
                    result = upward_social_turn(self.origin, voice, caption, opt_in=True)
                self.assertFalse(result['audio_confirmed'])
                self.assertFalse(voice.played)
                self.assertEqual(voice.synthesized, revoke_at == 4)
                self.assertIn('voice_consent_revoked', result['audio_error'])
                self.assertEqual(bridge.state['posture'], 'standing')
        caption.assert_not_called()

    def test_helper_does_not_claim_stand_after_failed_cleanup(self):
        class FakeVoice:
            def read_authority(self):
                return None, None, True
            def initiate_turn(self, *, trigger, visual_context):
                return {'state': 'initiated'}
        with patch.object(bridge, 'sdk_trick', side_effect=({'ok': True}, {'ok': False})), \
             patch.object(bridge, 'capture_frame', return_value=jpeg('gray')):
            with self.assertRaises(urllib.error.HTTPError) as error:
                upward_social_turn(self.origin, FakeVoice(), lambda jpg: 'Person nearby.', opt_in=True)
            self.assertEqual(error.exception.code, 409)
            error.exception.close()
            self.assertEqual(bridge.state['posture'], 'sitting')


if __name__ == '__main__':
    unittest.main()
