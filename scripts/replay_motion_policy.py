#!/usr/bin/env python3
"""Replay captured BEEP LiDAR observations through old and current steering policy."""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from beep_bridge import beep_bridge as bridge  # noqa: E402


def old_fluent_steering(sectors, current_action=None):
    values = bridge.validated_sector_values(sectors)
    if values is None:
        return "forward"
    left_open = min(values["front_left"], values["left"])
    right_open = min(values["front_right"], values["right"])
    if current_action in ("curveleft", "curveright") and values["front"] < 1.15:
        return current_action
    correction_needed = values["front"] < 1.15 or abs(left_open - right_open) >= 0.16
    if not correction_needed:
        return "forward"
    if left_open >= right_open + 0.08:
        return "curveleft"
    if right_open >= left_open + 0.08:
        return "curveright"
    return "forward"


def load_observations(root: Path):
    rows = []
    for path in sorted(root.glob("**/observation_packet.json")):
        try:
            packet = json.loads(path.read_text())
        except (OSError, json.JSONDecodeError):
            continue
        status = packet.get("bridge_status") or {}
        sectors = status.get("sectors")
        if bridge.validated_sector_values(sectors) is None:
            continue
        relative = path.relative_to(root)
        run_id = relative.parts[0] if len(relative.parts) > 1 else str(path.parent)
        rows.append({"path": str(path), "run_id": run_id, "sectors": sectors,
                     "recorded": status.get("last_command")})
    return rows


def replay(rows):
    old_counts, new_counts, recorded_counts = Counter(), Counter(), Counter()
    old_current = new_current = None
    current_run = None
    changed = []
    for row in rows:
        run_id = row.get("run_id", "single-run")
        if run_id != current_run:
            old_current = new_current = None
            current_run = run_id
        primary, _duration, _reason = bridge.choose_explore_action(row["sectors"])
        if primary == "forward":
            old_current = old_fluent_steering(row["sectors"], old_current)
            new_current = bridge.choose_fluent_steering(row["sectors"], new_current)
        else:
            # Collision/recovery policy precedes fluent steering in production.
            old_current = new_current = primary
        old_counts[old_current] += 1
        new_counts[new_current] += 1
        recorded_counts[row["recorded"] or "unknown"] += 1
        if old_current != new_current:
            changed.append({"path": row["path"], "old": old_current,
                            "new": new_current, "sectors": row["sectors"]})
    total = len(rows)
    turning = lambda counts: sum(v for k, v in counts.items() if k.startswith("curve") or k.startswith("turn"))
    return {
        "observations": total,
        "runs": len({row.get("run_id", "single-run") for row in rows}),
        "recorded_commands": dict(recorded_counts),
        "old_policy": dict(old_counts),
        "new_policy": dict(new_counts),
        "old_turning_fraction": round(turning(old_counts) / total, 3) if total else 0,
        "new_turning_fraction": round(turning(new_counts) / total, 3) if total else 0,
        "old_forward_fraction": round(old_counts["forward"] / total, 3) if total else 0,
        "new_forward_fraction": round(new_counts["forward"] / total, 3) if total else 0,
        "changed_count": len(changed),
        "changed_examples": changed[:10],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root", nargs="?", type=Path, default=ROOT / "captures" / "eyes" / "moving")
    args = parser.parse_args()
    print(json.dumps(replay(load_observations(args.root)), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
