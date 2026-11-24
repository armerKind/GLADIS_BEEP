"""Vendor-table gesture mapping and host-only motion fences (no robot SDK)."""
import json
import threading
import unittest
import urllib.error
import urllib.request
from unittest.mock import patch

from beep_bridge import beep_bridge as bridge
from beep_agent import BridgeBodyAdapter, skill, BodyDispatchError
from beep_agent.skills import SkillValidationError
from beep_agent.skills import GESTURE_IDS, GESTURE_ALIASES, GESTURE_CONTEXTS, gesture_for_context

VENDOR_ACTIONS = {
    1: 'lie_down', 2: 'stand_up', 3: 'crawl', 4: 'turn_around',
    5: 'mark_time', 6: 'squat', 7: 'turn_roll', 8: 'turn_pitch',
    9: 'turn_yaw', 10: 'three_axis', 11: 'pee', 12: 'sit',
    13: 'wave_hand', 14: 'stretch', 15: 'wave_body', 16: 'swing',
    17: 'pray', 18: 'seek', 19: 'handshake', 255: 'reset',
}
RESERVED = {'sit', 'stand_up', 'reset'}

class RepertoireTests(unittest.TestCase):
    def setUp(self):
        bridge._reset_motion_state_for_tests()
        bridge._reset_mission_state_for_tests()
        bridge.state['moving'] = False
        self.server = bridge.ThreadingHTTPServer(('127.0.0.1', 0), bridge.Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.origin = f'http://127.0.0.1:{self.server.server_port}'

    def tearDown(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join(2)
        bridge._reset_motion_state_for_tests(); bridge._reset_mission_state_for_tests()
        bridge.state['moving'] = False

    def post(self, body):
        req = urllib.request.Request(self.origin+'/action',
            data=json.dumps(body).encode(), headers={'Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=4) as r:
                return r.status, json.load(r)
        except urllib.error.HTTPError as e:
            with e:
                return e.code, json.load(e)

    def test_complete_vendor_table_and_aliases(self):
        self.assertEqual({m['id']: name for name, m in bridge.TRICK_ACTIONS.items()}, VENDOR_ACTIONS)
        self.assertEqual(GESTURE_IDS, {name: meta['id'] for name, meta in bridge.TRICK_ACTIONS.items()
                                       if name not in RESERVED})
        for name, meta in bridge.TRICK_ACTIONS.items():
            self.assertGreater(meta['duration_s'], 0)
            self.assertLessEqual(meta['duration_s'], 8)
            if name not in RESERVED:
                for alias in meta['aliases']:
                    self.assertEqual(GESTURE_ALIASES[alias], name)
        for alias, canonical in {'prey':'pray', 'urinate':'pee', 'wave':'wave_hand',
                                 'shake_hand':'handshake', 'march_in_place':'mark_time'}.items():
            self.assertEqual(bridge.resolve_trick(name=alias)['name'], canonical)
        for key in RESERVED:
            with self.assertRaises(SkillValidationError):
                skill('gesture', name=key, reason='Posture is separate.')
        for context, name in GESTURE_CONTEXTS.items():
            self.assertEqual(gesture_for_context(context, 'Only a suggestion.').arguments['name'], name)

    def test_reserved_posture_ids_refused_even_as_raw_id(self):
        with patch.object(bridge, 'sdk_trick', side_effect=AssertionError('no SDK')):
            for action_id in (2, 12, 255):
                for body in ({'id':action_id}, {'name':VENDOR_ACTIONS[action_id]}):
                    self.assertEqual(self.post(body)[0], 400)

    def test_stationary_unowned_no_mission_fences(self):
        with patch.object(bridge, 'sdk_trick', side_effect=AssertionError('no SDK')) as sdk:
            bridge.state['moving'] = True
            self.assertEqual(self.post({'name':'pray'})[0], 409)
            bridge.state['moving'] = False
            bridge.state['posture'] = 'sitting'
            self.assertEqual(self.post({'name':'pee'})[0], 409)
            bridge.state['posture'] = 'standing'
            with bridge.mission_lock: bridge._active_mission = {'state':'running'}
            self.assertEqual(self.post({'name':'wave'})[0], 409)
            with bridge.mission_lock: bridge._active_mission = None
            sdk.assert_not_called()

    def test_owned_http_gesture_and_concurrent_refusal_without_sdk_motion(self):
        entered, release = threading.Event(), threading.Event()
        observed = []
        def fake_gesture(**kwargs):
            observed.append((kwargs, bridge.state['motion_lease_id']))
            entered.set()
            release.wait(3)
            return {'ok': True, 'trick':bridge.resolve_trick(name=kwargs['name'])}
        result = []
        with patch.object(bridge, 'sdk_trick', side_effect=fake_gesture):
            worker = threading.Thread(target=lambda: result.append(self.post({'name':'wave'})))
            worker.start()
            self.assertTrue(entered.wait(2))
            self.assertEqual(self.post({'name':'pray'})[0], 409)
            release.set();worker.join(4)
        self.assertEqual(result[0][0], 200)
        self.assertEqual(result[0][1]['trick']['id'], 13)
        self.assertIsNotNone(observed[0][1])
        self.assertIsNone(bridge.state['motion_lease_id'])
        self.assertTrue(bridge.snapshot()['posture_uncertain'])
        self.assertEqual(self.post({'name':'pray'})[0],409)

    def test_adapter_dispatch_requires_supervised_and_validated_name(self):
        adapter = BridgeBodyAdapter('http://beep.invalid')
        with patch.object(adapter, '_json') as http:
            shadow = adapter.dispatch(skill('gesture', name='prey', reason='Ask nicely.'))
            self.assertFalse(shadow.dispatched)
            stationary = adapter.dispatch(skill('gesture', name='pray', reason='Ask nicely.'), rollout_mode='stationary')
            self.assertFalse(stationary.dispatched)
            http.assert_not_called()
        stationary = {'mission':None, 'status':{'moving':False,'motion_lease_id':None,'posture':'standing'}}
        with patch.object(adapter, '_json', side_effect=[stationary, {'ok':True,'dry_run':False,'trick':{'id':17,'name':'pray'}}]) as http:
            result = adapter.dispatch(skill('gesture', name='prey', reason='Ask nicely.'), rollout_mode='supervised')
            self.assertTrue(result.ok)
            self.assertEqual(http.call_args_list[0].args, ('/mission',))
            self.assertEqual(http.call_args_list[1].args, ('/action', {'name':'pray'}))
        for unsafe in ({'mission':{'id':'busy'},'status':stationary['status']},
                       {'mission':None,'status':{**stationary['status'],'posture':'sitting'}},
                       {'mission':None,'status':{**stationary['status'],'moving':True}},
                       {'mission':None,'status':{**stationary['status'],'motion_lease_id':'lease'}}):
            with patch.object(adapter, '_json', return_value=unsafe) as http:
                with self.assertRaises(BodyDispatchError):
                    adapter.perform_gesture('wave', rollout_mode='supervised')
                http.assert_called_once_with('/mission')
        with patch.object(adapter, '_json') as http:
            for name in ('sit','reset','stand_up','raw_11','nope'):
                with self.assertRaises((SkillValidationError, BodyDispatchError)):
                    adapter.perform_gesture(name, rollout_mode='supervised')
            http.assert_not_called()

    def test_every_named_agent_gesture_routes_as_name_not_raw_id(self):
        adapter = BridgeBodyAdapter('http://beep.invalid')
        stationary = {'mission':None, 'status':{'moving':False,'motion_lease_id':None,'posture':'standing'}}
        for name, action_id in GESTURE_IDS.items():
            with self.subTest(name=name):
                receipt = {'ok':True,'dry_run':False,'trick':{'name':name,'id':action_id}}
                with patch.object(adapter, '_json', side_effect=[stationary, receipt]) as http:
                    result = adapter.perform_gesture(name, rollout_mode='supervised')
                self.assertTrue(result.dispatched)
                self.assertEqual(http.call_args_list[1].args, ('/action', {'name':name}))

    def test_posture_changing_action_latches_uncertain_until_explicit_stand(self):
        def fake_action(**kw):
            return {'ok': True, 'dry_run': False,
                    'trick': bridge.resolve_trick(name=kw.get('name'), action_id=kw.get('action_id'))}
        with patch.object(bridge, 'sdk_trick', side_effect=fake_action):
            status, receipt = self.post({'name':'lie_down'})
            self.assertEqual(status, 200)
            self.assertTrue(receipt['ok'])
            self.assertTrue(bridge.snapshot()['posture_uncertain'])
            self.assertEqual(self.post({'name':'wave'})[0], 409)
            request = urllib.request.Request(self.origin+'/posture',
                data=b'{"posture":"stand"}', headers={'Content-Type':'application/json'})
            with urllib.request.urlopen(request, timeout=4) as response:
                standing = json.load(response)
            self.assertTrue(standing['ok'])
            self.assertFalse(standing.get('unchanged', False))
            self.assertFalse(bridge.snapshot()['posture_uncertain'])

    def test_exact_release_fence_and_callback_failure_releases_lease(self):
        with bridge.mission_lock:
            bridge._latest_mission = {'id':'m1','lease_id':'l1','release_id':'r1',
                                      'state':'completed'}
        release = {'boot_id':bridge.BOOT_ID,'mission_id':'m1','lease_id':'l1',
                   'release_id':'r1'}
        with patch.object(bridge,'sdk_trick',side_effect=RuntimeError('fake SDK failure')) as sdk:
            self.assertEqual(self.post({'name':'wave_hand','release':{**release,'release_id':'wrong'}})[0],409)
            sdk.assert_not_called()
            status, _ = self.post({'name':'wave_hand','release':release})
            self.assertEqual(status,500)
            sdk.assert_called_once()
        self.assertIsNone(bridge.snapshot()['motion_lease_id'])

    def test_release_race_new_mission_rejected_before_callback(self):
        with bridge.mission_lock:
            bridge._latest_mission = {'id':'m1','lease_id':'l1','release_id':'r1',
                                      'state':'completed'}
            bridge._active_mission = {'id':'new','state':'running'}
        with patch.object(bridge,'sdk_trick',side_effect=AssertionError('no SDK')) as sdk:
            self.assertEqual(self.post({'name':'wave_hand','release':{
                'boot_id':bridge.BOOT_ID,'mission_id':'m1','lease_id':'l1',
                'release_id':'r1'}})[0],409)
            sdk.assert_not_called()

if __name__ == '__main__': unittest.main()
