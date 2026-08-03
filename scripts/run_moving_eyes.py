#!/usr/bin/env python3
"""Continuously perceive while BEEP moves using fresh 4/6/9-frame windows.

The capture thread keeps filling a bounded ring while the main thread performs one
Gemini request at a time. There is no inference queue: after a result arrives, the
next request uses the newest complete window and silently supersedes stale evidence.
This process observes and updates the semantic world model; it never commands motors.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time
import uuid

from beep_agent import EmbodiedExecutive, GoalArbiter, WorldModel
from beep_eyes.contact_sheet import fetch_json
from beep_eyes.gemini import GeminiError, GeminiVisionClient
from beep_eyes.moving_window import ContinuousMovingCapture
from beep_eyes.policy import evaluate_proposal
from beep_eyes.schema import validate_perception_response


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://192.168.8.88:8766")
    parser.add_argument("--panel", type=int, choices=(4, 6, 9), default=9)
    parser.add_argument("--fps", type=float, default=3.0)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--model", default=None)
    parser.add_argument("--credential-source", choices=("auto", "environment", "hermes"), default="auto")
    parser.add_argument("--hermes-home", default=None)
    parser.add_argument("--mock-response", type=Path)
    parser.add_argument("--allow-no-mission", action="store_true", help="Permit stationary/bench capture")
    parser.add_argument("--max-inferences", type=int, default=0, help="Zero means until duration or mission end")
    parser.add_argument("--output-root", type=Path, default=Path("captures/eyes/moving"))
    return parser.parse_args()


def gemini_client(args: argparse.Namespace) -> GeminiVisionClient:
    if args.credential_source == "environment":
        return GeminiVisionClient.from_environment(model=args.model, timeout_s=args.timeout)
    if args.credential_source == "hermes":
        return GeminiVisionClient.from_hermes_pool(
            hermes_home=args.hermes_home, model=args.model, timeout_s=args.timeout,
        )
    try:
        return GeminiVisionClient.from_environment(model=args.model, timeout_s=args.timeout)
    except GeminiError:
        return GeminiVisionClient.from_hermes_pool(
            hermes_home=args.hermes_home, model=args.model, timeout_s=args.timeout,
        )


def write_window(directory: Path, frames, contact_sheet: bytes, packet: dict, response: dict,
                 decision: dict, semantic: dict) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for frame in frames:
        (directory / f"{frame.frame_id}.jpg").write_bytes(frame.jpeg)
    (directory / "contact_sheet.jpg").write_bytes(contact_sheet)
    (directory / "observation_packet.json").write_text(json.dumps(packet, indent=2, sort_keys=True))
    (directory / "model_response.json").write_text(json.dumps(response, indent=2, sort_keys=True))
    (directory / "policy_decision.json").write_text(json.dumps(decision, indent=2, sort_keys=True))
    (directory / "semantic_decision.json").write_text(json.dumps(semantic, indent=2, sort_keys=True))


def main() -> int:
    args = parse_args()
    if args.duration < 5 or args.duration > 600:
        raise SystemExit("--duration must be within [5, 600]")
    if args.max_inferences < 0 or args.max_inferences > 1000:
        raise SystemExit("--max-inferences must be within [0, 1000]")

    initial_mission = fetch_json(args.base_url, "/mission", args.timeout).get("mission")
    if not args.allow_no_mission and (
        not isinstance(initial_mission, dict) or initial_mission.get("state") not in {"starting", "running"}
    ):
        raise SystemExit("no active autonomous mission; moving eyes did not start")

    mock = validate_perception_response(json.loads(args.mock_response.read_text())) if args.mock_response else None
    client = None if mock is not None else gemini_client(args)
    run_id = time.strftime("%Y%m%dT%H%M%S") + "-moving-" + uuid.uuid4().hex[:8]
    output_dir = args.output_root / run_id
    output_dir.mkdir(parents=True, exist_ok=False)
    trace_path = output_dir / "trace.jsonl"
    capture = ContinuousMovingCapture(args.base_url, fps=args.fps, max_frames=max(18, args.panel * 2), timeout_s=min(args.timeout, 8.0))
    world = WorldModel()
    deadline = time.monotonic() + args.duration
    inference_count = 0
    capture.start()
    try:
        if not capture.ring.wait_for(args.panel, min(args.duration, max(args.timeout, args.panel / args.fps + 2.0))):
            raise RuntimeError(f"moving camera did not produce {args.panel} frames: {capture.stats()}")
        while time.monotonic() < deadline:
            mission_payload = fetch_json(args.base_url, "/mission", args.timeout)
            mission = mission_payload.get("mission")
            if not args.allow_no_mission and (
                not isinstance(mission, dict) or mission.get("state") not in {"starting", "running"}
            ):
                break
            frames, sheet, packet = capture.temporal_window(args.panel)
            packet["mission"] = mission
            requested_latest = frames[-1].frame_id
            started_at = time.time()
            if mock is not None:
                response = mock
            else:
                assert client is not None
                response = client.analyze(packet, sheet, frames[-1].jpeg)
            completed_at = time.time()
            newest_latest = capture.ring.latest(args.panel)[-1].frame.frame_id
            packet["inference"] = {
                "started_at": started_at,
                "completed_at": completed_at,
                "latency_s": round(completed_at - started_at, 3),
                "requested_latest_frame_id": requested_latest,
                "newest_frame_id_at_completion": newest_latest,
                "newer_window_available": newest_latest != requested_latest,
            }
            world.update(response, observed_at=completed_at, packet_id=f"{run_id}-{inference_count:04d}")
            selection = GoalArbiter().select(world)
            semantic = {
                "selection": asdict(selection),
                "next_decision": EmbodiedExecutive(world, selection.goal).decide(rollout_mode="shadow").to_dict(),
                "dispatch_performed": False,
            }
            policy = evaluate_proposal(response, packet["bridge_status"], mode="shadow")
            window_dir = output_dir / f"w{inference_count:04d}-{requested_latest}"
            write_window(window_dir, frames, sheet, packet, response, policy, semantic)
            event = {
                "window": inference_count,
                "latest_frame_id": requested_latest,
                "newer_window_available": newest_latest != requested_latest,
                "latency_s": packet["inference"]["latency_s"],
                "scene": response["scene"]["summary"],
                "confidence": response["confidence"],
                "next_skill": semantic["next_decision"]["skill_call"]["name"],
                "dispatch_performed": False,
            }
            with trace_path.open("a") as trace:
                trace.write(json.dumps(event, sort_keys=True) + "\n")
            print(json.dumps(event, sort_keys=True), flush=True)
            inference_count += 1
            if args.max_inferences and inference_count >= args.max_inferences:
                break
    finally:
        capture.stop()

    summary = {
        "ok": True,
        "run_id": run_id,
        "output_dir": str(output_dir),
        "panel_count": args.panel,
        "inferences": inference_count,
        "capture": capture.stats(),
        "dispatch_performed": False,
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True))
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
