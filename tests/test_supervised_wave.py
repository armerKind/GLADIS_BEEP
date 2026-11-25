"""Host-only fake I/O for opt-in supervised wave/greeting/recovery."""
import unittest
from unittest.mock import Mock
from beep_agent.memory import SemanticMemory
from beep_agent.supervised_wave import supervised_wave_cycle, SupervisedWaveError
from pathlib import Path
import tempfile


def state(now=10.):
    return {'mission':None,'status':{'version':'0.22.15-gesture-posture-fence-offline',
      'boot_id':'boot','robot_monotonic_s':now,
      'moving':False,'motion_lease_id':None,'posture':'standing',
      'posture_uncertain':False,'fall_suspected':False,'scan_age_s':.1,
      'slam':{'map_epoch':1,'map_revision':2,'map_revision_status':'verified','usable':True}}}


class WaveTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.bank=SemanticMemory(Path(self.temp.name)/'memory_bank.json')
        self.adapter=Mock()
        self.adapter.mission.side_effect=[state(),state(11.),state(12.),state(13.),state(14.)]
        self.adapter.execute_gesture_intent.return_value=Mock(dispatched=True,
            response={'ok':True,'trick':{'name':'wave_hand','id':13}})
        def successful(intent,admission,**kwargs):
            admission.claim(intent)
            return self.adapter.execute_gesture_intent.return_value
        self.adapter.execute_gesture_intent.side_effect=successful
        self.adapter._json.return_value={'ok':True,'posture':'standing'}
        self.synth=Mock(return_value=b'ID3'+b'X'*20)
        self.play=Mock(return_value=True)

    def run_cycle(self,**kwargs):
        return supervised_wave_cycle(self.adapter,self.bank,
            event_id='operator-1',physical_posture_confirmed=True,
            synthesize=self.synth,play=self.play,**kwargs)

    def test_wave_audio_stand_sequence_and_no_fake_physical_proof(self):
        outcome=self.run_cycle()
        self.assertEqual(outcome['wave_receipt']['trick']['id'],13)
        self.assertTrue(outcome['audio_confirmed'])
        self.assertEqual(outcome['recovery']['software_standing'],True)
        self.assertFalse(outcome['physical_posture_verified'])
        self.assertEqual(self.adapter._json.call_args_list[0].args,
                         ('/posture',{'posture':'stand'}))
        self.assertEqual(self.play.call_count,1)
        self.assertTrue(self.adapter.execute_gesture_intent.call_args.kwargs['physical_posture_confirmed'])
        self.assertTrue(self.adapter.execute_gesture_intent.call_args.args[0].stationary_preflight)
        self.assertEqual(self.adapter._json.call_count,1)
        self.assertEqual(self.bank.conversations,[])

    def test_active_owner_denies_all_side_effects(self):
        bad=state();bad['mission']={'id':'active','state':'running'}
        self.adapter.mission.side_effect=[bad]
        with self.assertRaises(SupervisedWaveError):self.run_cycle()
        self.adapter.execute_gesture_intent.assert_not_called()
        self.play.assert_not_called()
        self.adapter._json.assert_not_called()

    def test_tts_failure_or_bad_version_aborts_before_wave(self):
        self.synth.side_effect=RuntimeError('synthetic TTS outage')
        with self.assertRaises(SupervisedWaveError):self.run_cycle()
        self.adapter.mission.assert_not_called()
        self.adapter.execute_gesture_intent.assert_not_called()
        self.adapter._json.assert_not_called()
        self.synth.side_effect=None
        bad=state();bad['status']['version']='older'
        self.adapter.mission.side_effect=[bad]
        with self.assertRaises(SupervisedWaveError):self.run_cycle()
        self.adapter.execute_gesture_intent.assert_not_called()

    def test_slam_change_after_stand_fails_cycle_even_with_sdk_ack(self):
        changed=state(14.);changed['status']['slam']['usable']=False
        self.adapter.mission.side_effect=[state(),state(11.),state(12.),state(13.),changed]
        with self.assertRaises(SupervisedWaveError) as ctx:self.run_cycle()
        self.assertTrue(ctx.exception.report['recovery']['software_standing'])
        self.assertFalse(ctx.exception.report['recovery']['slam_usable'])

    def test_audio_failure_attempts_stand_but_does_not_claim_speech(self):
        self.play.return_value=False
        with self.assertRaises(SupervisedWaveError) as ctx:self.run_cycle()
        self.assertEqual(ctx.exception.report['recovery']['software_standing'],True)
        self.assertFalse(ctx.exception.report['audio_confirmed'])
        self.assertEqual(self.adapter._json.call_count,1)

    def test_new_owner_after_wave_suppresses_unsafe_posture_request(self):
        busy=state(12.);busy['mission']={'id':'new','state':'running'}
        self.adapter.mission.side_effect=[state(),busy,busy]
        with self.assertRaises(SupervisedWaveError) as ctx:self.run_cycle()
        self.assertEqual(ctx.exception.report['recovery']['reason'],'owner_changed')
        self.adapter._json.assert_not_called()
        self.play.assert_not_called()

    def test_lost_wave_reply_no_retry_and_only_safe_stand(self):
        self.adapter.execute_gesture_intent.side_effect=TimeoutError('unknown')
        # Fake the admission claim that the adapter makes *before* its POST.
        def claimed(intent,admission,**kwargs):
            admission.claim(intent)
            raise TimeoutError('unknown')
        self.adapter.execute_gesture_intent.side_effect=claimed
        with self.assertRaises(SupervisedWaveError) as ctx:self.run_cycle()
        self.assertEqual(ctx.exception.report['recovery']['software_standing'],True)
        self.assertEqual(self.adapter.execute_gesture_intent.call_count,1)
        self.play.assert_not_called()


if __name__=='__main__':unittest.main()
