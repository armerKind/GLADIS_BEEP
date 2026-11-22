"""No-hardware regressions for the stationary social audio CLI."""
import io
import struct
import sys
import unittest
from contextlib import redirect_stdout
from types import SimpleNamespace
from unittest.mock import patch

from scripts import run_social_voice as social


class SocialAudioCliTests(unittest.TestCase):
    def test_gain_default_clamp_and_finite_validation(self):
        for value, expected in [(None, 4.0), ('0.2', 1.0), ('4.5', 4.5), ('100', 8.0)]:
            argv = ['run_social_voice.py', '--calibrate-mic']
            if value is not None:
                argv += ['--speaker-gain', value]
            with patch.object(sys, 'argv', argv):
                args = social.parse_args()
            self.assertEqual(args.speaker_gain, expected)
            self.assertEqual(social.speaker_filter(args.speaker_gain),
                             f'volume={expected:.3f},alimiter=limit=0.95:level=0')
        for invalid in ('nan', 'inf', '-inf'):
            with patch.object(sys, 'argv', ['run_social_voice.py', '--calibrate-mic',
                                            f'--speaker-gain={invalid}']), redirect_stdout(io.StringIO()):
                with self.assertRaises(SystemExit):
                    social.parse_args()
        with patch.object(sys, 'argv', ['run_social_voice.py']):
            with self.assertRaises(SystemExit):
                social.parse_args()

    def test_vad_suggestion_uses_noise_ceiling_and_speech_distribution(self):
        result = social.suggest_vad([18] * 90 + [20] * 10, [35] * 80 + [44] * 20)
        self.assertEqual(result['noise_floor_rms'], 18)
        self.assertEqual(result['noise_p95_rms'], 20)
        self.assertEqual(result['speech_p75_rms'], 35)
        self.assertEqual((result['onset_rms'], result['offset_rms']), (27.5, 23.8))
        weak = social.suggest_vad([20] * 100, [22] * 100)
        self.assertIsNone(weak['onset_rms'])
        self.assertIn('weak', weak['warning'])

    def test_calibration_is_local_and_does_not_write_raw_audio(self):
        class FakeStream:
            def __enter__(self): return self
            def __exit__(self, *_): return False
            def read(self, count):
                assert count == 320
                return struct.pack('<320h', *([50 if phases[0] == 2 else 20] * 320)), False
        class FakeSd:
            @staticmethod
            def query_devices():
                return [{'name': 'USB PnP Sound Device', 'max_input_channels': 1}]
            @staticmethod
            def RawInputStream(**kwargs):
                self.assertEqual((kwargs['samplerate'], kwargs['blocksize']), (16000, 320))
                return FakeStream()
        phases = [0]
        def cue(card):
            self.assertEqual(card, '3')
            phases[0] += 1
        args = SimpleNamespace(mic_name='USB PnP Sound Device')
        listing = 'card 3: Headphones [bcm2835 Headphones], device 0: Headphones\n'
        with patch.object(social.subprocess, 'run', return_value=SimpleNamespace(stdout=listing)):
            with redirect_stdout(io.StringIO()):
                result = social.calibrate_mic(args, sd=FakeSd(), cue=cue)
        self.assertEqual(phases, [2])
        self.assertEqual((result['noise_floor_rms'], result['speech_rms']), (20, 50))
        self.assertEqual((result['onset_rms'], result['offset_rms']), (35, 27.5))

    def test_capture_overflow_fails_calibration(self):
        class Overflow:
            def __enter__(self): return self
            def __exit__(self, *_): return False
            def read(self, _): return bytes(640), True
        sd = SimpleNamespace(RawInputStream=lambda **_: Overflow())
        with self.assertRaisesRegex(RuntimeError, 'overflow'):
            social.capture_rms(sd, 1)


if __name__ == '__main__':
    unittest.main()
