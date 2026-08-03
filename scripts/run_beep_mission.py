#!/usr/bin/env python3
"""Replay perceptions through the persistent embodied executive without dispatch."""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from beep_agent import EmbodiedExecutive, EmbodiedGoal, GoalArbiter, WorldModel


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("responses", type=Path, nargs="+")
    parser.add_argument("--goal", help="Override autonomous goal selection")
    parser.add_argument("--target", default="box")
    parser.add_argument("--mark", action="store_true")
    parser.add_argument("--mode", choices=("shadow", "stationary", "supervised"), default="shadow")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    world = WorldModel()
    fixed_goal = EmbodiedGoal(args.goal, args.target, args.mark) if args.goal else None
    arbiter = GoalArbiter()
    executive = EmbodiedExecutive(world, fixed_goal) if fixed_goal else None
    trace = []
    for sequence, path in enumerate(args.responses, 1):
        perception = json.loads(path.read_text())
        snapshot = world.update(perception, observed_at=time.time(), packet_id=f"replay-{sequence}:{path.name}")
        selection = None if fixed_goal else arbiter.select(world)
        if selection is not None:
            executive = EmbodiedExecutive(world, selection.goal)
        assert executive is not None
        decision = executive.decide(rollout_mode=args.mode)
        trace.append({"source": str(path), "world": snapshot, "goal_selection": None if selection is None else {"goal": selection.goal.name, "drive": selection.drive, "rationale": selection.rationale}, "decision": decision.to_dict(), "dispatch_performed": False})
    result = {"goal": trace[-1]["decision"]["goal"], "target_query": args.target, "mode": args.mode, "trace": trace, "final_world": world.snapshot(), "dispatch_performed": False}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, sort_keys=True))
    print(json.dumps({
        "ok": True, "goal": result["goal"], "target_query": args.target,
        "observations": len(trace), "next_skill": trace[-1]["decision"]["skill_call"]["name"],
        "target_id": trace[-1]["decision"]["target_id"], "dispatch_performed": False,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
