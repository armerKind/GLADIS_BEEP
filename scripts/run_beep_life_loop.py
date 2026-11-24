#!/usr/bin/env python3
"""Replay saved synthetic LifeLoop windows. Strictly no robot or motor I/O."""
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
    parser.add_argument('--shadow',action='store_true',required=True,
                        help='mandatory shadow-only mode; no dispatch option exists')
    parser.add_argument('--fixture',type=Path,
        default=ROOT/'tests'/'fixtures'/'life_loop_shadow_events.json')
    parser.add_argument('--output-root',type=Path,required=True,
                        help='local checkpoint and trace directory')
    parser.add_argument('--split-at',type=int,default=4,
                        help='simulate process restart before this event')
    args=parser.parse_args()
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
