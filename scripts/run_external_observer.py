#!/usr/bin/env python3
"""Replay external-camera evidence through the bounded BEEP observer."""

import argparse
import json
import os
from pathlib import Path
import sys
import time

from beep_observer import (
    BridgeStopSink,
    DirectoryFrameSource,
    EvidenceStore,
    ExternalObserver,
    MetadataRiskEvaluator,
    ObserverConfig,
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", required=True, help="images with optional <image>.json risk sidecars")
    parser.add_argument("--evidence-dir", required=True, help="private local evidence directory")
    parser.add_argument("--source-name", default="external-room-camera")
    parser.add_argument("--mode", choices=("record", "stop"), default="record")
    parser.add_argument("--bridge-url", default="")
    parser.add_argument("--token-env", default="BEEP_OBSERVER_STOP_TOKEN")
    parser.add_argument("--stop-cooldown", type=float, default=2.0)
    parser.add_argument("--poll-interval", type=float, default=0.0)
    return parser.parse_args()


def main():
    args = parse_args()
    token = os.environ.get(args.token_env, "")
    if args.mode == "stop" and (not args.bridge_url or not token):
        raise SystemExit("stop mode requires --bridge-url and a non-empty token environment variable")
    sink = BridgeStopSink(args.bridge_url, token) if args.mode == "stop" else None
    observer = ExternalObserver(
        DirectoryFrameSource(args.source_dir, source_name=args.source_name),
        MetadataRiskEvaluator(),
        EvidenceStore(Path(args.evidence_dir)),
        ObserverConfig(mode=args.mode, stop_cooldown_s=args.stop_cooldown),
        sink,
    )
    processed = 0
    while True:
        event = observer.run_once()
        if event is None:
            break
        processed += 1
        print(json.dumps({
            "event_id": event["event_id"],
            "risk": event["risk"],
            "reasons": event["reasons"],
            "retained_path": event["retained_path"],
            "stop": event["stop"],
        }, sort_keys=True))
        if args.poll_interval > 0:
            time.sleep(args.poll_interval)
    print(json.dumps({"processed": processed, "mode": args.mode, "evidence_dir": str(Path(args.evidence_dir))}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
