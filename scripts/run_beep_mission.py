#!/usr/bin/env python3
"""Run perceptions through the embodied executive with optional typed dispatch."""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from beep_agent import AgentSession, BridgeBodyAdapter, EmbodiedGoal


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("responses", type=Path, nargs="+")
    parser.add_argument("--goal", help="Override autonomous goal selection")
    parser.add_argument("--target", default="box")
    parser.add_argument("--mark", action="store_true")
    parser.add_argument("--mode", choices=("shadow", "stationary", "supervised"), default="shadow")
    parser.add_argument("--dispatch", action="store_true", help="dispatch the final typed skill; requires --mode supervised")
    parser.add_argument("--base-url", default="http://192.168.8.88:8766")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.dispatch and args.mode != "supervised":
        raise SystemExit("--dispatch requires --mode supervised")
    fixed_goal = EmbodiedGoal(args.goal, args.target, args.mark) if args.goal else None
    session = AgentSession(robot_id="BEEP", mission_id="replay", fixed_goal=fixed_goal)
    trace = []
    final_decision = None
    for sequence, path in enumerate(args.responses, 1):
        perception = json.loads(path.read_text())
        snapshot = session.update(perception, observed_at=time.time(),
                                  packet_id=f"replay-{sequence}:{path.name}", mission_id="replay")
        decision = session.decide(rollout_mode=args.mode)
        selection = session.selection
        final_decision = decision
        trace.append({"source": str(path), "world": snapshot, "goal_selection": None if selection is None else {"goal": selection.goal.name, "drive": selection.drive, "rationale": selection.rationale}, "decision": decision.to_dict(), "dispatch_performed": False})
    dispatch_result = None
    if args.dispatch:
        assert final_decision is not None
        dispatch_result = BridgeBodyAdapter(args.base_url).dispatch(final_decision.skill_call, rollout_mode=args.mode).to_dict()
    result = {"goal": trace[-1]["decision"]["goal"], "target_query": args.target,
              "session_id": session.session_id, "mode": args.mode, "trace": trace,
              "final_world": session.world.snapshot(),
              "dispatch_performed": bool(dispatch_result and dispatch_result["dispatched"]),
              "dispatch_result": dispatch_result}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, sort_keys=True))
    print(json.dumps({
        "ok": True, "goal": result["goal"], "target_query": args.target,
        "observations": len(trace), "next_skill": trace[-1]["decision"]["skill_call"]["name"],
        "target_id": trace[-1]["decision"]["target_id"], "dispatch_performed": result["dispatch_performed"],
        "dispatch_result": dispatch_result,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
