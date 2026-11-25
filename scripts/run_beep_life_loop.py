#!/usr/bin/env python3
"""Shadow fixture replay by default; supervised one-shot wave only by opt-in."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from beep_agent.life_loop import ShadowLifeLoop  # noqa: E402


def main() -> None:
    parser=argparse.ArgumentParser(description=__doc__)
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--shadow',action='store_true',help='offline fixture replay (default)')
    modes.add_argument('--supervised-wave',action='store_true',
                       help='one operator-triggered wave + audio + stand; never automatic')
    parser.add_argument('--fixture',type=Path,
        default=ROOT/'tests'/'fixtures'/'life_loop_shadow_events.json')
    parser.add_argument('--output-root',type=Path,required=True,
                        help='local checkpoint and trace directory')
    parser.add_argument('--split-at',type=int,default=4,
                        help='simulate process restart before this event')
    parser.add_argument('--bridge-url',default=None)
    parser.add_argument('--memory-file',type=Path,default=None)
    parser.add_argument('--no-cloud-vision',action='store_true',
                        help='explicit privacy latch; shadow and supervised-wave never use cloud images')
    parser.add_argument('--person-id',default=None,
                        help='exact operator-confirmed memory ID; no visual recognition')
    parser.add_argument('--audio-file',type=Path,default=None,
                        help='optional pre-rendered MP3/WAV greeting')
    parser.add_argument('--physical-confirmed',action='store_true',
                        help='nearby operator has confirmed stable standing and clear floor')
    parser.add_argument('--gain',type=float,default=3.)
    args=parser.parse_args()
    if args.supervised_wave:
        if (not args.bridge_url or args.memory_file is None or not args.physical_confirmed
                or not 0.1 <= args.gain <= 3.):
            parser.error('--supervised-wave needs --bridge-url, --memory-file, '
                         '--physical-confirmed and gain within 0.1..3')
        from uuid import uuid4
        from beep_agent.body import BridgeBodyAdapter
        from beep_agent.memory import SemanticMemory
        from beep_agent.supervised_wave import supervised_wave_cycle, SupervisedWaveError
        from scripts.run_social_voice import bridge_playback, edge_sonia, supplied_audio
        bank=SemanticMemory(args.memory_file)
        adapter=BridgeBodyAdapter(args.bridge_url, timeout_s=20.)
        synth=(lambda _: supplied_audio(args.audio_file)) if args.audio_file else (
               lambda text: edge_sonia(text,'en-GB-SoniaNeural'))
        play=lambda audio: bridge_playback(audio,args.gain,args.bridge_url)
        try:
            result=supervised_wave_cycle(adapter,bank,event_id='operator-'+uuid4().hex,
                physical_posture_confirmed=True,synthesize=synth,play=play,
                person_id=args.person_id)
        except SupervisedWaveError as exc:
            result={**exc.report,'error':str(exc),'no_cloud_vision':args.no_cloud_vision}
            args.output_root.mkdir(parents=True,exist_ok=True)
            (args.output_root/'supervised_wave_result.json').write_text(json.dumps(result,indent=2)+'\n')
            print(json.dumps(result,sort_keys=True))
            raise SystemExit(1) from exc
        result={**result,'no_cloud_vision':args.no_cloud_vision}
        args.output_root.mkdir(parents=True,exist_ok=True)
        (args.output_root/'supervised_wave_result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,sort_keys=True))
        return
    # The CLI accepts local fixture JSON only; URLs cannot become I/O endpoints.
    if args.fixture.suffix != '.json' or not args.fixture.is_file():
        parser.error('fixture must be an existing local JSON file')
    events=json.loads(args.fixture.read_text())
    if not isinstance(events,list) or not 0<args.split_at<len(events):
        parser.error('fixture must be a list with a valid restart boundary')
    boot=events[0]['boot_id']
    runner=ShadowLifeLoop(args.output_root,session_id='offline-life-loop',boot_id=boot)
    trace=[runner.process(item) for item in events[:args.split_at]]
    runner=ShadowLifeLoop(args.output_root,boot_id=boot,restore=True)
    trace.extend(runner.process(item) for item in events[args.split_at:])
    if runner.http_paths or any(t['dispatch_performed'] or t['motor_authority'] for t in trace):
        raise RuntimeError('shadow-only invariant violated')
    summary={'mode':'synthetic_shadow_only','fixture':args.fixture.name,
        'no_cloud_vision':args.no_cloud_vision,
        'events':len(trace),'session_id':runner.session.session_id,
        'selected_goal':trace[-1]['selected_goal'],
        'selected_drive':trace[-1]['selected_drive'],
        'goal_revision':runner.session.strategy['goal_revision'],
        'policy_allowed':sum(1 for t in trace if t['policy_allowed']),
        'blocked_reasons':[t['blocked_reason'] for t in trace if t['blocked_reason']],
        'http_paths':runner.http_paths,'dispatch_performed':False,
        'checkpoint':str(runner.checkpoint),'trace':str(runner.journal),
        'restart_replayed':True}
    print(json.dumps(summary,sort_keys=True,indent=2))


if __name__=='__main__':main()
