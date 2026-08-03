"""BEEP eyes: temporal capture -> multipanel perception -> policy evaluation.

This initial runner is intentionally proposal-only. It never dispatches bridge
movement, gesture, or speech endpoints in any mode.
"""
from __future__ import annotations

import argparse
import json
import time
import uuid
from pathlib import Path

from beep_eyes.contact_sheet import build_contact_sheet, capture_sequence, fetch_json, load_replay_sequence, write_capture_artifacts
from beep_eyes.gemini import GeminiError, GeminiVisionClient
from beep_eyes.policy import evaluate_proposal
from beep_eyes.schema import validate_perception_response


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://192.168.8.88:8766")
    parser.add_argument("--replay-dir", type=Path, help="Use saved f*.jpg frames instead of contacting BEEP")
    parser.add_argument("--frames", type=int, default=4)
    parser.add_argument("--interval", type=float, default=1.5)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--model", default=None)
    parser.add_argument("--credential-source", choices=("auto", "environment", "hermes"), default="auto")
    parser.add_argument("--hermes-home", default=None, help="Hermes profile directory used for protected Gemini credentials")
    parser.add_argument("--mode", choices=("shadow", "stationary", "supervised"), default="shadow")
    parser.add_argument("--mock-response", type=Path, help="Use a saved model response instead of calling Gemini")
    parser.add_argument("--output-root", type=Path, default=Path("captures/eyes"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_id = time.strftime("%Y%m%dT%H%M%S") + "-" + uuid.uuid4().hex[:8]
    output_dir = args.output_root / run_id
    if args.replay_dir:
        frames, status = load_replay_sequence(args.replay_dir, args.frames)
    else:
        frames, status = capture_sequence(args.base_url, args.frames, args.interval, args.timeout)
    contact_sheet = build_contact_sheet(frames)
    packet = write_capture_artifacts(output_dir, frames, status, contact_sheet)
    prompt_packet = {key: value for key, value in packet.items() if key != "artifacts"}

    if args.mock_response:
        model_response = validate_perception_response(json.loads(args.mock_response.read_text()))
        provider = {"type": "mock", "source": str(args.mock_response)}
    else:
        if args.credential_source == "environment":
            client = GeminiVisionClient.from_environment(model=args.model, timeout_s=args.timeout)
        elif args.credential_source == "hermes":
            client = GeminiVisionClient.from_hermes_pool(
                hermes_home=args.hermes_home, model=args.model, timeout_s=args.timeout
            )
        else:
            try:
                client = GeminiVisionClient.from_environment(model=args.model, timeout_s=args.timeout)
            except GeminiError:
                client = GeminiVisionClient.from_hermes_pool(
                    hermes_home=args.hermes_home, model=args.model, timeout_s=args.timeout
                )
        model_response = client.analyze(prompt_packet, contact_sheet, frames[-1].jpeg)
        provider = {"type": "gemini", "model": client.model}

    policy_status = status if args.replay_dir else fetch_json(args.base_url, "/status", args.timeout)
    decision = evaluate_proposal(model_response, policy_status, mode=args.mode)
    (output_dir / "model_response.json").write_text(json.dumps(model_response, indent=2, sort_keys=True))
    (output_dir / "policy_decision.json").write_text(json.dumps(decision, indent=2, sort_keys=True))
    summary = {
        "ok": True,
        "run_id": run_id,
        "output_dir": str(output_dir),
        "provider": provider,
        "mode": args.mode,
        "dispatch_performed": False,
        "scene": model_response["scene"]["summary"],
        "confidence": model_response["confidence"],
        "escalate": model_response["escalate"],
        "proposed_skills": [step["skill"] for step in model_response["plan"]["steps"]],
        "eligible_skills": [item["skill"] for item in decision["steps"] if item["eligible"]],
    }
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
