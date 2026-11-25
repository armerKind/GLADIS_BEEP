"""Offline runner gates: occluded windows never become cloud payloads."""
from pathlib import Path
from types import SimpleNamespace
import io
import json
import tempfile
import unittest
from unittest.mock import Mock, patch

from PIL import Image
from beep_eyes.contact_sheet import CapturedFrame
from scripts import run_moving_eyes as eyes


def jpeg(color):
    output = io.BytesIO()
    Image.new('RGB', (160, 120), color).save(output, 'JPEG')
    return output.getvalue()


class FakeCapture:
    def __init__(self, frames):
        self.frames = frames
        self.start = Mock()
        self.stop = Mock()
        self.ring = SimpleNamespace(
            wait_for=lambda *_: True,
            latest=lambda *_: [SimpleNamespace(frame=frames[-1])],
        )

    def temporal_window(self, *_args, **_kwargs):
        return self.frames, b'locally-rendered-contact-sheet', {'frames': []}

    def stats(self):
        return {'captured': len(self.frames), 'errors': 0, 'buffered': len(self.frames)}


def options(root, **changes):
    values = dict(base_url='http://fake-local', panel=4, fps=3., duration=5., timeout=1.,
        inference_interval=0., model=None, credential_source='environment', hermes_home=None,
        mock_response=None, allow_no_mission=False, max_inferences=1, output_root=root,
        memory_file=None, place_label=None, goal=None, target='scene', mark=False,
        bias_mission=False, resume_session=None, resume_checkpoint=None,
        no_cloud_vision=False, vision_switch_file=root / 'control',
        occlusion_audio_file=None, audio_gain=3.)
    values.update(changes)
    return SimpleNamespace(**values)


class MovingEyesPrivacyTests(unittest.TestCase):
    def test_capped_window_speaks_once_then_switches_without_model_call(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            frames = [CapturedFrame('f00', 1., jpeg('black')),
                      CapturedFrame('f01', 2., jpeg('gray'))]
            capture = FakeCapture(frames)  # Older black panel is also private.
            model = Mock()
            model.analyze.side_effect = AssertionError('Gemini called on capped panel')
            reads = [1, 1, 0]
            def read(_origin, path, _timeout):
                if path == '/mission':
                    return {'mission': {'id': 'm1', 'state': 'running'} if reads.pop(0) else None}
                if path == '/slam_map':
                    return {'ok': False, 'reason': 'offline'}
                raise AssertionError(path)
            def speak(*_args, **_kwargs):
                (root / 'control').write_text('no-cloud-vision\n')
                return True
            with patch.object(eyes, 'parse_args', return_value=options(root)), \
                 patch.object(eyes, 'fetch_json', side_effect=read), \
                 patch.object(eyes, 'fetch_slam_map', return_value={'ok': False}), \
                 patch.object(eyes, 'ContinuousMovingCapture', return_value=capture), \
                 patch.object(eyes, 'gemini_client', return_value=model), \
                 patch.object(eyes, 'speak_camera_occluded', side_effect=speak) as voice:
                self.assertEqual(eyes.main(), 0)
            model.analyze.assert_not_called()
            voice.assert_called_once()
            capture.start.assert_called_once()
            capture.stop.assert_called_once()
            run = next(root.glob('*-moving-*'))
            report = json.loads((run / 'summary.json').read_text())
            events = [json.loads(line) for line in (run / 'trace.jsonl').read_text().splitlines()]
            self.assertEqual(report['vision_mode'], 'no_cloud_vision')
            self.assertEqual(report['inferences'], 0)
            self.assertEqual(report['occlusion_events'], 1)
            self.assertEqual(events[0]['state'], 'camera_occluded')
            self.assertTrue(events[0]['audio_confirmed'])
            self.assertEqual(events[0]['occluded_frame_cloud_calls'], 0)

    def test_occluded_runner_holds_without_operator_switch_until_mission_ends(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture = FakeCapture([CapturedFrame('f00', 1., jpeg('black'))])
            counts = [0]
            def read(_origin, path, _timeout):
                self.assertEqual(path, '/mission')
                counts[0] += 1
                return {'mission': {'id': 'm1', 'state': 'running'} if counts[0] <= 2 else None}
            with patch.object(eyes, 'parse_args', return_value=options(root)), \
                 patch.object(eyes, 'fetch_json', side_effect=read), \
                 patch.object(eyes, 'fetch_slam_map', return_value={'ok': False}), \
                 patch.object(eyes, 'ContinuousMovingCapture', return_value=capture), \
                 patch.object(eyes, 'gemini_client', return_value=Mock(analyze=Mock(side_effect=AssertionError('cloud forbidden')))), \
                 patch.object(eyes, 'speak_camera_occluded', return_value=True) as voice:
                self.assertEqual(eyes.main(), 0)
            voice.assert_called_once()
            run = next(root.glob('*-moving-*'))
            report = json.loads((run / 'summary.json').read_text())
            self.assertEqual(report['vision_mode'], 'awaiting_no_cloud_vision')
            self.assertEqual(report['inferences'], 0)
            self.assertFalse((root / 'control').exists())

    def test_startup_no_cloud_skips_camera_credentials_and_model(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            capture = FakeCapture([CapturedFrame('f0', 1., jpeg('gray'))])
            mission_calls = [0]
            def read(_origin, path, _timeout):
                self.assertEqual(path, '/mission')
                mission_calls[0] += 1
                return {'mission': {'id': 'm1', 'state': 'running'} if mission_calls[0] == 1 else None}
            with patch.object(eyes, 'parse_args', return_value=options(root, no_cloud_vision=True)), \
                 patch.object(eyes, 'fetch_json', side_effect=read), \
                 patch.object(eyes, 'ContinuousMovingCapture', return_value=capture), \
                 patch.object(eyes, 'gemini_client', side_effect=AssertionError('credentials forbidden')), \
                 patch.object(eyes, 'fetch_slam_map', side_effect=AssertionError('image context forbidden')), \
                 patch.object(eyes, 'speak_camera_occluded', side_effect=AssertionError('no capture')):
                self.assertEqual(eyes.main(), 0)
            capture.start.assert_not_called()
            run = next(root.glob('*-moving-*'))
            report = json.loads((run / 'summary.json').read_text())
            self.assertEqual(report['vision_mode'], 'no_cloud_vision')
            self.assertEqual(report['inferences'], 0)
            self.assertFalse(report['dispatch_performed'])


if __name__ == '__main__':
    unittest.main()
