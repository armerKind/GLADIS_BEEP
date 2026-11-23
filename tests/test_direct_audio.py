"""HTTP/audio contract tests with no Pi, ALSA or motor access."""
import http.client
import io
import json
import tempfile
import threading
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from beep_bridge import beep_bridge as bridge
from beep_agent.memory import SemanticMemory
from scripts import run_social_voice as social


class DirectAudioTests(unittest.TestCase):
    def setUp(self):
        self.server = bridge.ThreadingHTTPServer(('127.0.0.1', 0), bridge.Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)
        self.addCleanup(self.thread.join, 4)
        self.origin = f'http://127.0.0.1:{self.server.server_port}'

    def request(self, body=b'mp3', mime='audio/mpeg', path='/audio/play', headers=None):
        conn = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=4)
        conn.request('POST', path, body=body,
                     headers={'Content-Type': mime, **(headers or {})})
        response = conn.getresponse()
        status, data = response.status, json.loads(response.read())
        conn.close()
        return status, data

    def test_valid_wav_mp3_gain_and_config(self):
        seen = []
        with patch.object(bridge, 'play_bridge_audio', side_effect=lambda audio, gain: seen.append((audio, gain))):
            self.assertEqual(self.request(b'mp3')[0], 200)

            code, data = self.request(b'RIFFabcdWAVE', 'audio/wav', '/audio/play?gain=100')
            self.assertEqual((code, data['ok']), (200, True))
            self.assertIn('elapsed_s', data)
            self.assertEqual(self.request(b'mp3', path='/audio/play?gain=-10')[0], 200)
        self.assertEqual(seen, [(b'mp3', 3.0), (b'RIFFabcdWAVE', 6.0), (b'mp3', 0.5)])
        conn = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=4)
        conn.request('GET', '/config')
        cfg = json.loads(conn.getresponse().read());conn.close()
        self.assertEqual(cfg['audio_playback'], {'endpoint': 'POST /audio/play'})

    def test_oversize_invalid_media_gain_and_failed_playback(self):
        with patch.object(bridge, 'play_bridge_audio') as player:
            self.assertEqual(self.request(b'x', headers={'Content-Length':'2000001'})[0], 413)
            self.assertEqual(self.request(b'bad', 'text/plain')[0], 415)
            self.assertEqual(self.request(b'x', path='/audio/play?gain=nan')[0], 400)
            self.assertEqual(self.request(b'x', path='/audio/play?gain=1&gain=2')[0], 400)
            player.assert_not_called()
        with patch.object(bridge, 'play_bridge_audio', side_effect=RuntimeError('speaker')):
            self.assertEqual(self.request()[0], 502)
        self.assertFalse(bridge.handler_failure_requires_stop('/audio/play', RuntimeError('speaker')))

    def test_concurrent_audio_is_409_while_status_is_responsive(self):
        entered, release = threading.Event(), threading.Event()
        responses = []
        def blocked(audio, gain):
            entered.set()
            if not release.wait(3): raise RuntimeError('test timed out')
        with patch.object(bridge, 'play_bridge_audio', side_effect=blocked):
            worker = threading.Thread(target=lambda: responses.append(self.request()))
            worker.start()
            self.assertTrue(entered.wait(2))
            self.assertEqual(self.request()[0], 409)
            conn = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=4)
            conn.request('GET', '/mission')
            status = json.loads(conn.getresponse().read())['status']
            conn.close()
            self.assertIsNone(status['motion_lease_id'])
            release.set();worker.join(4)
        self.assertEqual(responses[0][0], 200)

    def test_real_pipeline_uses_headphones_highpass_and_only_audio_tools(self):
        listing = 'card 2: Headphones [bcm2835 Headphones], device 0: bcm2835 Headphones\n'
        results = [SimpleNamespace(stdout=listing), SimpleNamespace(),
                   SimpleNamespace(stdout=b'RIFF' + bytes(80)), SimpleNamespace(returncode=0)]
        with patch.object(bridge.subprocess, 'run', side_effect=results) as run:
            bridge.play_bridge_audio(b'mp3', 3.0)
        self.assertEqual(run.call_count, 4)
        ffmpeg = run.call_args_list[2].args[0]
        self.assertIn('highpass=f=300,volume=3.000,alimiter=limit=0.95:level=0', ffmpeg)
        self.assertEqual(run.call_args_list[-1].args[0][-1], 'plughw:2,0')

    def test_social_bridge_playback_receipt_and_failure(self):
        payload = b'RIFFabcdWAVE' + bytes(100)

        with patch.object(bridge, 'play_bridge_audio'):
            self.assertTrue(social.bridge_playback(payload, 3.0, self.origin))
        with patch.object(bridge, 'play_bridge_audio', side_effect=RuntimeError('fail')):
            with self.assertRaises(Exception):
                social.bridge_playback(b'mp3', 3.0, self.origin)

    def test_speak_cli_direct_bridge_commits_only_after_http_receipt(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'memory_bank.json'
            args = social.parse_args(['--speak', '--memory-file', str(path),
                '--bridge-url', self.origin, '--heard-text', 'Hello BEEP',
                '--reply', 'Hello from the host'])
            def played(audio, gain):
                self.assertFalse(path.exists())
                self.assertEqual((audio, gain), (b'mp3', 3.0))
            with (patch.object(bridge, 'play_bridge_audio', side_effect=played),
                  patch.object(social, 'edge_sonia', return_value=b'mp3'),
                  redirect_stdout(io.StringIO()) as output):
                social.speak_turn(SemanticMemory(path), args)
            receipt = json.loads(output.getvalue())
            self.assertEqual(receipt['state'], 'replied')
            self.assertIn('http_playback_s', receipt)
            self.assertIn('memory_commit_s', receipt)
            self.assertEqual(SemanticMemory(path).conversations[-1]['spoken_reply'], args.reply)


if __name__ == '__main__':
    unittest.main()
