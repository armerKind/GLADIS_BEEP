"""Fixture-only shadow LifeLoop above mission leases. No motor or HTTP import.

This coordinator has no dispatcher, credential, robot URL, or permission to
convert its suggestions into actions. Stationary evidence is a separate input
from moving camera frames and requires an exact released owner.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import tempfile

from .fsutil import fsync_directory
from .gesture_policy import GesturePolicy
from .session import AgentSession, SessionIdentityError


class LifeLoopError(ValueError):
    pass


class ShadowLifeLoop:
    def __init__(self, root: Path, *, session_id: str | None = None,
                 boot_id: str, restore: bool = False):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.checkpoint = self.root/'life_loop_checkpoint.json'
        self.journal = self.root/'life_loop_trace.jsonl'
        self.http_paths: list[str] = []  # Always empty; no transport is imported.
        if restore:
            raw=json.loads(self.checkpoint.read_text())
            if (set(raw) != {'schema_version','session','policy','pending','release',
                             'seen','speaking_turn'} or raw['schema_version'] != 1):
                raise LifeLoopError('invalid shadow checkpoint')
            try:
                self.session = AgentSession.from_dict(raw['session'],
                    expected_robot_id='beep',expected_boot_id=boot_id)
            except SessionIdentityError as exc:
                raise LifeLoopError('checkpoint boot or robot mismatch') from exc
            self.policy = GesturePolicy.from_dict(raw['policy'])
            self.pending = raw['pending']
            self.release = raw['release']
            self.seen = set(raw['seen'])
            self.speaking_turn = raw['speaking_turn']
            if (not isinstance(raw['seen'],list) or len(self.seen)!=len(raw['seen'])
                    or any(not isinstance(x,str) for x in self.seen)
                    or self.pending is not None and not isinstance(self.pending,dict)
                    or self.release is not None and not isinstance(self.release,dict)):
                raise LifeLoopError('invalid shadow checkpoint contents')
        else:
            if self.checkpoint.exists():
                raise LifeLoopError('explicit restore required for existing session')
            self.session=AgentSession(robot_id='beep',mission_id=None,
                                      boot_id=boot_id,session_id=session_id)
            self.policy=GesturePolicy()
            self.pending=None
            self.release=None
            self.seen=set()
            self.speaking_turn=None

    def _persist(self) -> None:
        data={'schema_version':1,'session':self.session.to_dict(),
              'policy':self.policy.to_dict(),'pending':self.pending,
              'release':self.release,'seen':sorted(self.seen),
              'speaking_turn':self.speaking_turn}
        path=None
        try:
            with tempfile.NamedTemporaryFile('w',dir=self.root,
                prefix='life_loop.',suffix='.tmp',delete=False) as stream:
                path=Path(stream.name)
                json.dump(data,stream,allow_nan=False,sort_keys=True)
                stream.flush();os.fsync(stream.fileno())
            os.replace(path,self.checkpoint)
            fsync_directory(self.root)
        finally:
            if path is not None:path.unlink(missing_ok=True)

    def process(self, event: dict) -> dict:
        if not isinstance(event,dict):raise LifeLoopError('event must be object')
        eid=event.get('event_id');kind=event.get('kind');now=event.get('at_robot')
        if (not isinstance(eid,str) or not 0<len(eid)<=128
                or kind not in ('mission_bound','moving_observation','mission_released',
                                'stationary_evidence','speaking_turn')
                or event.get('boot_id') != self.session.boot_id
                or type(now) not in (int,float) or not math.isfinite(now) or now<0):
            raise LifeLoopError('invalid fixture provenance')
        assert isinstance(now,(int,float))
        now=float(now)
        if eid in self.seen:
            return self._trace(eid,kind,'duplicate_fixture_event')
        result='observed'
        candidate=None
        if kind=='mission_bound':
            try:
                self.session.bind_motion(mission_id=event['mission_id'],
                    lease_id=event['lease_id'],boot_id=event['boot_id'])
            except (KeyError,SessionIdentityError) as exc:
                raise LifeLoopError('invalid mission binding') from exc
            self.release=None
            result='mission_bound'
        elif kind=='moving_observation':
            if ((event.get('mission_id'),event.get('lease_id')) !=
                    (self.session.mission_id,self.session.execution['lease_id'])
                    or self.session.execution['lease_id'] is None):
                raise LifeLoopError('moving observation lacks exact lease')
            fixture=event.get('perception_fixture')
            # Only repository-owned saved JSON; never a URL or user path.
            if not isinstance(fixture,str) or not fixture.endswith('.json') or Path(fixture).name != fixture:
                raise LifeLoopError('perception must be a saved local fixture')
            source=Path(__file__).resolve().parents[1]/'tests'/'fixtures'/fixture
            perception=json.loads(source.read_text())
            self.session.update(perception,observed_at=now,packet_id=eid,
                mission_id=event['mission_id'],lease_id=event['lease_id'])
            if event.get('goal_revision') is not None:
                revision=event['goal_revision']
                if type(revision) is not int or revision<self.session.strategy['goal_revision']:
                    raise LifeLoopError('invalid goal revision')
                self.session.strategy['goal_revision']=revision
            self.pending=event.get('candidate')
            if self.pending is not None and not isinstance(self.pending,dict):
                raise LifeLoopError('invalid moving candidate')
            result='moving_observation_only'
        elif kind=='mission_released':
            if not isinstance(event.get('release_id'),str) or not event['release_id']:
                raise LifeLoopError('missing exact release receipt')
            try:
                self.session.release_motion(mission_id=event.get('mission_id'),
                    lease_id=event.get('lease_id'))
            except SessionIdentityError as exc:
                raise LifeLoopError('wrong mission release') from exc
            self.release={'mission_id':event['mission_id'],
                          'lease_id':event['lease_id'],'release_id':event['release_id']}
            result='released'
        elif kind=='speaking_turn':
            turn=event.get('conversation_turn_id')
            if not isinstance(turn,str) or not turn:raise LifeLoopError('missing speech turn')
            self.speaking_turn=turn
            result='speaking_supersedes_gesture'
        elif kind=='stationary_evidence':
            candidate=event.get('candidate',self.pending)
            if self.session.execution['lease_id'] is not None:
                result='motion_owner_active'
            elif (self.release is None or event.get('release_id') != self.release['release_id']):
                result='release_mismatch'
            elif (event.get('stationary') is not True or event.get('posture_uncertain') is not False
                    or event.get('map_revision_status') != 'verified'
                    or type(event.get('map_epoch')) is not int
                    or type(event.get('map_revision')) is not int
                    or type(event.get('scan_age_s')) not in (int,float)
                    or not 0 <= event['scan_age_s'] <= .6):
                result='stationary_evidence_invalid'
            elif not isinstance(candidate,dict):
                result='no_candidate'
            elif (self.speaking_turn is not None and
                  self.speaking_turn==candidate.get('conversation_turn_id')):
                result='speaking_turn'
            else:
                try:
                    proposal=self.policy.consider(candidate['context'],
                        event_id=candidate['event_id'], entity_id=candidate.get('entity_id'),
                        observed_at=candidate['observed_at_robot'],now=now,
                        reason=candidate['reason'],
                        conversation_turn_id=candidate.get('conversation_turn_id'))
                except (KeyError,ValueError,TypeError) as exc:
                    raise LifeLoopError('invalid candidate') from exc
                result=proposal.reason
        self.seen.add(eid)
        trace=self._trace(eid,kind,result,candidate=candidate)
        self._persist()
        with self.journal.open('a') as stream:
            stream.write(json.dumps(trace,sort_keys=True,allow_nan=False)+'\n')
            stream.flush();os.fsync(stream.fileno())
        return trace

    def _trace(self,eid,kind,reason,*,candidate=None):
        selection=self.session.selection
        return {'event_id':eid,'kind':kind if reason!='released' else 'released',
            'session_id':self.session.session_id,'boot_id':self.session.boot_id,
            'mission_id':self.session.mission_id,
            'lease_id':self.session.execution['lease_id'],
            'release':self.release,'goal_revision':self.session.strategy['goal_revision'],
            'selected_goal':None if selection is None else selection.goal.name,
            'selected_drive':None if selection is None else selection.drive,
            'proposed':None if not isinstance(candidate,dict) else candidate.get('context'),
            'policy_allowed':reason=='proposed_shadow_only',
            'blocked_reason':(reason if kind=='stationary_evidence'
                              and reason!='proposed_shadow_only' else None),
            'dispatch_performed':False,'motor_authority':False,
            'observation_provenance':'synthetic_saved_fixture'}
