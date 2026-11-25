"""Host fake-I/O contracts for supervised gesture admission; never contacts BEEP."""
import threading
import unittest
from unittest.mock import patch

from beep_agent.body import BridgeBodyAdapter, BodyDispatchError
from beep_agent.gesture_policy import GestureAdmission, GestureAdmissionError, GestureIntent


def fixture():
    intent = GestureIntent(decision_id='d1', session_id='s', boot_id='boot',
        name='wave_hand', event_id='e1', observed_at_robot=10.,
        issued_at_robot=10.5, expires_at_robot=13., map_epoch=1,
        map_revision=2, released_mission_id='m1', released_lease_id='l1',
        release_id='r1', rollout_mode='supervised')
    admission = GestureAdmission(session_id='s', boot_id='boot',
        released_mission_id='m1', released_lease_id='l1', release_id='r1')
    status = {'boot_id':'boot', 'robot_monotonic_s':11., 'moving':False,
        'motion_lease_id':None, 'posture':'standing', 'posture_uncertain':False,
        'fall_suspected':False, 'scan_age_s':.1,
        'slam':{'map_epoch':1,'map_revision':2,'map_revision_status':'verified'}}
    readback = {'mission':{'id':'m1','lease_id':'l1','release_id':'r1',
                           'state':'completed'},'status':status}
    receipt = {'ok':True,'dry_run':False,'trick':{'name':'wave_hand','id':13}}
    return intent, admission, readback, receipt


class IntentDispatchTests(unittest.TestCase):
    def test_explicit_stationary_preflight_requires_no_latest_owner(self):
        intent, gate, readback, receipt = fixture()
        from dataclasses import replace
        intent=replace(intent, released_mission_id='stationary_preflight',
                       released_lease_id='none', release_id='d1',stationary_preflight=True)
        gate=GestureAdmission(session_id='s',boot_id='boot',
            released_mission_id='stationary_preflight', released_lease_id='none',
            release_id='d1')
        readback={**readback,'mission':None}
        adapter=BridgeBodyAdapter('http://invalid')
        with patch.object(adapter,'_json',side_effect=[readback,receipt,readback]) as http:
            result=adapter.execute_gesture_intent(intent,gate,physical_posture_confirmed=True)
        self.assertTrue(result.dispatched)
        self.assertEqual(http.call_args_list[1].args[1],{'name':'wave_hand'})
        bad={**readback,'mission':{'id':'new','state':'running'}}
        second=replace(intent,decision_id='d2',event_id='e2',release_id='d2')
        gate2=GestureAdmission(session_id='s',boot_id='boot',
            released_mission_id='stationary_preflight',released_lease_id='none',
            release_id='d2')
        with patch.object(adapter,'_json',return_value=bad) as http:
            with self.assertRaises(GestureAdmissionError):
                adapter.execute_gesture_intent(second,gate2,physical_posture_confirmed=True)
            http.assert_called_once_with('/mission')

    def test_preflight_denies_without_http_action(self):
        intent, gate, readback, _ = fixture()
        adapter = BridgeBodyAdapter('http://invalid')
        with patch.object(adapter, '_json', return_value=readback) as http:
            with self.assertRaises(GestureAdmissionError):
                adapter.execute_gesture_intent(intent, gate, physical_posture_confirmed=False)
            http.assert_called_once_with('/mission')

    def test_receipt_cleanup_and_postflight_do_not_invent_physical_posture(self):
        intent, gate, readback, receipt = fixture()
        adapter = BridgeBodyAdapter('http://invalid')
        with patch.object(adapter, '_json', side_effect=[readback,receipt,readback]) as http:
            result = adapter.execute_gesture_intent(intent,gate,physical_posture_confirmed=True)
        self.assertTrue(result.dispatched)
        self.assertEqual([c.args[0] for c in http.call_args_list],
                         ['/mission','/action','/mission'])
        self.assertEqual(result.response['bridge_lease_released'],True)
        self.assertEqual(gate.uncertain_reason,'post_action_physical_posture_unverified')
        with patch.object(adapter,'_json') as http:
            with self.assertRaises(GestureAdmissionError):
                adapter.execute_gesture_intent(intent,gate,physical_posture_confirmed=True)
            http.assert_called_once_with('/mission')

    def test_timeout_after_request_claims_once_and_attempts_cleanup(self):
        intent, gate, readback, _ = fixture()
        adapter = BridgeBodyAdapter('http://invalid')
        with patch.object(adapter, '_json', side_effect=[readback,BodyDispatchError('timeout'),
                                                         readback]) as http:
            with self.assertRaises(BodyDispatchError):
                adapter.execute_gesture_intent(intent,gate,physical_posture_confirmed=True)
        self.assertEqual([c.args[0] for c in http.call_args_list],
                         ['/mission','/action','/mission'])
        self.assertEqual(gate.uncertain_reason,'gesture_outcome_uncertain')
        self.assertIn('d1',gate.claimed)

    def test_lost_postflight_and_changed_owner_fail_closed(self):
        for final in (BodyDispatchError('readback timeout'),{'mission':{'id':'new'},'status':{}}):
            intent, gate, readback, receipt = fixture()
            adapter=BridgeBodyAdapter('http://invalid')
            with patch.object(adapter,'_json',side_effect=[readback,receipt,final]):
                with self.assertRaises(BodyDispatchError):
                    adapter.execute_gesture_intent(intent,gate,physical_posture_confirmed=True)
            self.assertIsNotNone(gate.uncertain_reason)

    def test_busy_race_after_preflight_is_not_retried(self):
        intent, gate, readback, _ = fixture()
        adapter = BridgeBodyAdapter('http://invalid')
        entered, release = threading.Event(), threading.Event()
        calls=[]; errors=[]
        def fake(path, body=None):
            calls.append(path)
            if path=='/mission':return readback
            if path=='/action':
                entered.set();release.wait(2)
                return {'ok':False,'http_status':409,'error':'busy'}
            return {'ok':True}
        with patch.object(adapter,'_json',side_effect=fake):
            worker=threading.Thread(target=lambda: self._run(adapter,intent,gate,errors))
            worker.start();self.assertTrue(entered.wait(2));release.set();worker.join(3)
        self.assertEqual(len(errors),1)
        self.assertEqual(calls.count('/action'),1)
        self.assertEqual(calls.count('/stop'),0)

    @staticmethod
    def _run(adapter,intent,gate,errors):
        try:adapter.execute_gesture_intent(intent,gate,physical_posture_confirmed=True)
        except Exception as exc:errors.append(exc)


if __name__ == '__main__':unittest.main()
