"""Host-only privacy regressions: no capped JPEG reaches a provider."""
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from PIL import Image, ImageDraw

from beep_eyes.privacy import CameraPrivacyError, inspect_frame, VisionMode
from beep_eyes.gemini import GeminiVisionClient
from scripts.run_moving_eyes import speak_camera_occluded


def jpeg(color, *, hud=False):
    image = Image.new('RGB', (160, 120), color)
    if hud:
        ImageDraw.Draw(image).rectangle((0, 0, 159, 9), fill='white')
    output = io.BytesIO()
    image.save(output, 'JPEG')
    return output.getvalue()


class VisionPrivacyTests(unittest.TestCase):
    def test_black_and_dark_capped_frame_even_with_vendor_hud(self):
        for frame in (jpeg('black'), jpeg((12, 12, 12), hud=True)):
            with self.subTest(frame=frame[:20]):
                result = inspect_frame(frame)
                self.assertTrue(result.occluded)
                self.assertLess(result.mean_luminance, 25)
        self.assertTrue(inspect_frame(jpeg('white')).occluded)
        self.assertFalse(inspect_frame(jpeg('gray')).occluded)
        image = Image.new('RGB', (160, 120), (3, 3, 3))
        ImageDraw.Draw(image).rectangle((60, 40, 110, 95), fill=(180, 180, 180))
        output = io.BytesIO(); image.save(output, 'JPEG')
        self.assertFalse(inspect_frame(output.getvalue()).occluded)

    def test_invalid_and_truncated_frame_fail_closed(self):
        for frame in (b'', b'not-jpeg', jpeg('gray')[:-30]):
            with self.subTest(size=len(frame)), self.assertRaises(CameraPrivacyError):
                inspect_frame(frame)

    def test_gemini_transport_never_receives_capped_or_undecodable_frame(self):
        transport = Mock(side_effect=AssertionError('cloud request forbidden'))
        client = GeminiVisionClient('sentinel', transport=transport)
        for frame in (jpeg('black'), jpeg((10, 10, 10)), b'bad'):
            with self.subTest(frame=len(frame)), self.assertRaises(CameraPrivacyError):
                client.analyze({'packet_id': 'x'}, jpeg('gray'), frame)
        transport.assert_not_called()

    def test_fixed_reaction_uses_audio_playback_without_any_image(self):
        with patch('scripts.run_social_voice.edge_sonia', return_value=b'ID3-audio') as tts, \
             patch('scripts.run_social_voice.bridge_playback', return_value=True) as play:
            self.assertTrue(speak_camera_occluded('http://local', audio_file=None, gain=3.0))
        text, voice = tts.call_args.args
        self.assertIn('camera has gone dark', text)
        self.assertEqual(voice, 'en-GB-SoniaNeural')
        play.assert_called_once_with(b'ID3-audio', 3.0, 'http://local')

    def test_vision_mode_switch_is_one_way_and_missing_file_is_not_consent(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'vision_mode'
            mode = VisionMode(path)
            self.assertFalse(mode.no_cloud_vision)
            path.write_text('no-cloud-vision\n')
            self.assertTrue(mode.refresh())
            path.write_text('cloud\n')
            self.assertTrue(mode.refresh())
            self.assertTrue(mode.no_cloud_vision)
            self.assertFalse(VisionMode(path, no_cloud_vision=True).cloud_allowed)


if __name__ == '__main__':
    unittest.main()
