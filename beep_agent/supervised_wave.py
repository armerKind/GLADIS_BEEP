"""Explicit, one-shot supervised stationary wave and greeting.

No automatic policy promotion or locomotion. The caller must supply operator
posture confirmation and a bounded audio transport; no global /stop is used.
"""
from __future__ import annotations

from math import isfinite
from uuid import uuid4
from typing import Callable

from .body import BridgeBodyAdapter
from .gesture_policy import GestureAdmission, GestureAdmissionError, GestureIntent
from .memory import SemanticMemory


class SupervisedWaveError(RuntimeError):
    def __init__(self, reason: str, report: dict):
        super().__init__(reason)
        self.report = report


def greeting_for(bank: SemanticMemory, person_id: str | None = None) -> str:
    """Only an explicitly provided, exact bank ID may supply a name."""
    if not isinstance(bank, SemanticMemory):
        raise ValueError('semantic memory required')
    name = None
    if person_id is not None:
        if not isinstance(person_id,str) or not person_id.strip() or len(person_id)>128:
            raise ValueError('invalid person ID')
        entry=bank.people.get(person_id)
        name=entry.get('name') if entry else None
    if name:
        return f'Hello, {name}. I am BEEP. My social calibration is proceeding suspiciously well.'
    return 'Hello. I am BEEP. My social calibration is proceeding suspiciously well.'


def _unowned(readback: dict, *, boot: str) -> bool:
    if not isinstance(readback,dict) or not isinstance(readback.get('status'),dict):
        return False
    s=readback['status']; mission=readback.get('mission')
    return (s.get('boot_id')==boot and s.get('moving') is False
            and s.get('motion_lease_id') is None
            and (mission is None or isinstance(mission,dict)
                 and mission.get('state') in ('completed','cancelled','failed')))


def supervised_wave_cycle(adapter: BridgeBodyAdapter, bank: SemanticMemory, *,
        event_id: str, physical_posture_confirmed: bool,
        synthesize: Callable[[str],bytes], play: Callable[[bytes],bool],
        person_id: str | None = None) -> dict:
    """One operator-triggered wave; postflight stand is attempted only if unowned.

    A stand SDK acknowledgement cannot prove physical recovery. On ambiguous
    action/transport failure, never retry wave; ask the nearby observer.
    """
    report={'mode':'supervised_wave','event_id':event_id,'wave_receipt':None,
            'audio_confirmed':False,'recovery':None,
            'physical_posture_verified':False,'dispatch_performed':False}
    if physical_posture_confirmed is not True or not isinstance(event_id,str) or not event_id.strip():
        raise SupervisedWaveError('operator confirmation and event ID required',report)
    try:
        audio=synthesize(greeting_for(bank,person_id))
        if not isinstance(audio,bytes) or not 0<len(audio)<=2_000_000:
            raise ValueError('invalid bounded greeting audio')
    except Exception as exc:
        raise SupervisedWaveError('greeting could not be prepared; no gesture attempted',report) from exc
    try:
        preflight=adapter.mission()
    except Exception as exc:
        raise SupervisedWaveError('stationary preflight unavailable; no gesture attempted',report) from exc
    s=preflight.get('status') if isinstance(preflight,dict) else None
    if (not isinstance(s,dict) or not isinstance(s.get('boot_id'),str)
            or not _unowned(preflight,boot=s['boot_id'])):
        raise SupervisedWaveError('active or unknown owner',report)
    if s.get('version') != '0.22.16-idle-gesture-preflight-fence-offline':
        raise SupervisedWaveError('unverified bridge version',report)
    boot=s.get('boot_id'); now=s.get('robot_monotonic_s'); age=s.get('scan_age_s')
    slam=s.get('slam') or {}
    if (not boot or type(now) not in (int,float)
            or type(age) not in (int,float)
            or not isinstance(slam,dict)):
        raise SupervisedWaveError('invalid stationary clock or sensor readback',report)
    assert isinstance(now,(int,float)) and isinstance(age,(int,float))
    if not isfinite(now) or not isfinite(age):
        raise SupervisedWaveError('nonfinite stationary clock or sensor readback',report)
    epoch,revision=slam.get('map_epoch'),slam.get('map_revision')
    if type(epoch) is not int or type(revision) is not int or slam.get('usable') is not True:
        raise SupervisedWaveError('unverified map identity',report)
    latest=preflight.get('mission')
    decision_id='wave-'+uuid4().hex
    if latest is None:
        mission_id,lease_id,release_id='stationary_preflight','none',decision_id
        stationary=True
    else:
        mission_id,lease_id,release_id=(latest.get(k) for k in ('id','lease_id','release_id'))
        stationary=False
    try:
        intent=GestureIntent(decision_id=decision_id,session_id='supervised-'+boot,
            boot_id=boot,name='wave_hand',event_id=event_id,
            observed_at_robot=max(0.,now-age),issued_at_robot=now,
            expires_at_robot=now+4.,map_epoch=epoch,
            map_revision=revision,released_mission_id=mission_id,
            released_lease_id=lease_id,release_id=release_id,
            rollout_mode='supervised',stationary_preflight=stationary)
        admission=GestureAdmission(session_id=intent.session_id,boot_id=boot,
            released_mission_id=mission_id,released_lease_id=lease_id,release_id=release_id)
        admission.check(intent,preflight,physical_posture_confirmed=True)
    except (GestureAdmissionError,TypeError,ValueError) as exc:
        raise SupervisedWaveError('stationary admission refused: '+str(exc),report) from exc
    error=None
    try:
        result=adapter.execute_gesture_intent(intent,admission,
            physical_posture_confirmed=True)
        report['dispatch_performed']=bool(result.dispatched)
        report['wave_receipt']=result.response
        if not result.dispatched:
            raise RuntimeError('wave was not dispatched')
        before_audio=adapter.mission()
        if not _unowned(before_audio,boot=boot):
            raise RuntimeError('owner changed before greeting')
        before_play=adapter.mission()
        if not _unowned(before_play,boot=boot):
            raise RuntimeError('owner changed before audio playback')
        report['audio_confirmed']=play(audio) is True
        if not report['audio_confirmed']:
            raise RuntimeError('audio playback acknowledgement missing')
    except Exception as exc:
        error=exc
    finally:
        # Admission claims immediately before the POST. A refused preflight
        # must not trigger an unrelated posture command.
        if admission.claimed:
            try:
                before_stand=adapter.mission()
                if not _unowned(before_stand,boot=boot):
                    report['recovery']={'software_standing':False,'reason':'owner_changed'}
                else:
                    stand=adapter._json('/posture',{'posture':'stand'})
                    after=adapter.mission()
                    status=after.get('status') if isinstance(after,dict) else None
                    good=(isinstance(stand,dict) and stand.get('ok') is True
                          and _unowned(after,boot=boot) and isinstance(status,dict)
                          and status.get('posture')=='standing'
                          and status.get('posture_uncertain') is False)
                    final_slam=status.get('slam') or {} if isinstance(status,dict) else {}
                    slam_ok=(isinstance(final_slam,dict) and final_slam.get('usable') is True
                        and final_slam.get('map_epoch')==epoch
                        and final_slam.get('map_revision_status')=='verified')
                    report['recovery']={'software_standing':bool(good),
                                        'slam_usable':bool(slam_ok),
                                        'reason':'acknowledged' if good else 'unverified'}
            except Exception as exc:
                report['recovery']={'software_standing':False,
                                    'reason':'readback_or_stand_failed'}
                if error is None:error=exc
    if (error is not None or not report['recovery']
            or not report['recovery']['software_standing']
            or not report['recovery'].get('slam_usable')):
        raise SupervisedWaveError('wave/greeting/recovery incomplete; human posture check required',report) from error
    return report
