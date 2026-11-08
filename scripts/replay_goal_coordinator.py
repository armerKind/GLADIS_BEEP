#!/usr/bin/env python3
"""Offline replay: recorded cap responses + synthetic local lease transitions.

No sockets, bridge imports, motor dispatch, metric target or physical claims.
The historical packets are qualitative observations; map/lease transitions are
explicitly synthetic fixtures, never inferred from their unverified timestamps.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from beep_agent.coordinator import CoordinatorContext, GoalCoordinator  # noqa: E402
from beep_agent.session import AgentSession  # noqa: E402
from beep_agent.strategy import GoalProposal, StrategicOwner  # noqa: E402

FIXTURES = ROOT / "tests" / "fixtures"


def replay() -> dict:
    session = AgentSession(robot_id="beep", mission_id=None, boot_id="synthetic-boot",
                           session_id="offline-replay")
    session.update(json.loads((FIXTURES / "box_far_response.json").read_text()),
                   observed_at=1.0, packet_id="initial-qualitative")
    proposal = GoalProposal.from_dict(dict(schema_version=1, proposal_id="self-directed-explore",
        session_id=session.session_id, boot_id=session.boot_id, map_epoch=1, map_revision=1,
        base_goal_revision=0, observed_world_sequence=session.world.sequence,
        goal_name="explore", target_track_id=None, issued_at_robot=1.0,
        expires_at_robot=3.0, source_event_ids=[],
        reason="Investigate novel space while preserving embodied continuity."))
    StrategicOwner(session).submit(proposal, now_robot=2.0, map_epoch=1, map_revision=1)
    owner = GoalCoordinator(session)
    def context(*, revision=1, mission=None, lease=None, moving=False, now=2.0):
        return CoordinatorContext("synthetic-boot", 1, revision, "synthetic-grid",
                                  True, True, now, mission, lease, moving)
    decisions = [owner.tick(context()).action]
    owner.bind_confirmed(context(mission="synthetic-mission", lease="synthetic-lease",
                                 moving=True), goal_revision=1)
    receipts = []
    previous_capture = -1.0
    for index in (0, 1):
        path = FIXTURES / f"cap_confirmation_window{index}.json"
        saved = json.loads(path.read_text())
        frames = saved["packet"]["frames"]
        if len(frames) != 1 or not isinstance(saved["response"], dict):
            raise ValueError("recorded qualitative fixture has unexpected shape")
        frame = frames[0]
        captured = frame["captured_at"]
        if captured <= previous_capture or frame["motion"]["motion_lease_id"] != saved["packet"]["bridge_status"]["motion_lease_id"]:
            raise ValueError("recorded frame order or owner changed")
        previous_capture = captured
        # The fixture's captured_at is NOT a verified sensor-exposure stamp.
        # Never pass its pose or model plan as trusted metric routing evidence.
        if any("bbox" in obj for obj in saved["response"]["scene"]["objects"]):
            raise ValueError("this replay is restricted to qualitative detections")
        session.update(saved["response"], observed_at=2.1 + index,
                       packet_id=frame["frame_id"], mission_id="synthetic-mission",
                       lease_id="synthetic-lease")
        decision = owner.tick(context(mission="synthetic-mission", lease="synthetic-lease",
                                      moving=True, now=2.1 + index))
        if decision.action != "continue_local" or decision.motor_authority:
            raise ValueError("shadow replay unexpectedly gained motor authority")
        decisions.append(decision.action)
        receipts.append({"source": path.name, "frame_id": frame["frame_id"],
                         "evidence_quality": "qualitative_unverified_exposure",
                         "source_lease_id": frame["motion"]["motion_lease_id"]})
    owner.complete_segment(context(mission="synthetic-mission", lease="synthetic-lease", now=4.0),
                           result="blocked", release_id="synthetic-release")
    decisions.extend((owner.tick(context(now=4.1)).action,
                      owner.tick(context(revision=2, now=4.2)).action))
    assert session.selection is not None
    return {"schema_version": 1, "mode": "offline_shadow",
            "map_and_lease_context": "synthetic_not_recorded_provenance",
            "recorded_observations": receipts, "goal": session.selection.goal.name,
            "goal_revision": session.strategy["goal_revision"],
            "decisions": decisions, "dispatch_performed": False,
            "route_validated": False, "physical_stop_verified": False}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="optional JSON replay report")
    args = parser.parse_args()
    result = replay()
    payload = json.dumps(result, sort_keys=True, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload)
    print(payload, end="")


if __name__ == "__main__":
    main()
