#!/usr/bin/env python3
"""Continuously perceive while BEEP moves using fresh 4/6/9-frame windows.

The capture thread keeps filling a bounded ring while the main thread performs one
Gemini request at a time. There is no inference queue: after a result arrives, the
next request uses the newest complete window and silently supersedes stale evidence.
Shadow by default: each window updates the persistent AgentSession without
motor dispatch. An explicit --bias-mission may send short-lived direction advice
to the exact running bridge lease; onboard LiDAR still owns gait and safety.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import time
import uuid

# The companion launcher invokes this file directly, not with ``python -m``.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from beep_agent import AgentSession, BodyDispatchError, BridgeBodyAdapter, EmbodiedGoal, SemanticMemory
from beep_agent.live_bridge_observer import LiveBridgeObserver, LiveEvidenceUnavailable
from beep_eyes.contact_sheet import fetch_json, fetch_slam_map
from beep_eyes.gemini import GeminiError, GeminiVisionClient
from beep_eyes.moving_window import ContinuousMovingCapture
from beep_eyes.privacy import CameraOccluded, CameraPrivacyError, VisionMode, ensure_visible
from beep_eyes.schema import validate_perception_response


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://192.168.8.88:8766")
    parser.add_argument("--panel", type=int, choices=(4, 6, 9), default=9)
    parser.add_argument("--fps", type=float, default=3.0)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--inference-interval", type=float, default=0.0,
                        help="minimum seconds between completed semantic windows; capture continues while waiting")
    parser.add_argument("--model", default=None)
    parser.add_argument("--credential-source", choices=("auto", "environment", "hermes"), default="auto")
    parser.add_argument("--hermes-home", default=None)
    parser.add_argument("--mock-response", type=Path)
    parser.add_argument("--no-cloud-vision", action="store_true",
                        help="latch out all external image requests; monitor LiDAR mission without camera")
    parser.add_argument("--vision-switch-file", type=Path,
                        help="runtime one-way switch: write no-cloud-vision to this local file")
    parser.add_argument("--occlusion-audio-file", type=Path,
                        help="pre-rendered MP3/WAV for the one-time camera-occluded warning")
    parser.add_argument("--audio-gain", type=float, default=3.0)
    parser.add_argument("--allow-no-mission", action="store_true", help="Permit stationary/bench capture")
    parser.add_argument("--max-inferences", type=int, default=0, help="Zero means until duration or mission end")
    parser.add_argument("--output-root", type=Path, default=Path("captures/eyes/moving"))
    parser.add_argument("--memory-file", type=Path,
                        help="Opt-in cross-boot semantic JSON bank (outside per-run captures)")
    parser.add_argument("--place-label", help="Operator-confirmed place label for memory; never infer a room from model prose")
    parser.add_argument("--goal", help="Bind one semantic goal to the whole moving mission")
    parser.add_argument("--target", default="scene", help="Target query for --goal")
    parser.add_argument("--mark", action="store_true")
    parser.add_argument("--bias-mission", action="store_true",
                        help="Opt in to 4-second, lease-fenced target-position advice; no motor dispatch")
    parser.add_argument("--resume-session", type=Path,
                        help="Resume only when robot, mission identity and freshness match")
    parser.add_argument("--resume-checkpoint", type=Path,
                        help="Resume a Stack 2 checkpoint and adjacent journal under exact bridge identity")
    return parser.parse_args()


def tracked_target_direction(session: AgentSession, response: dict) -> str | None:
    """Use only the current packet's confidently tracked attention entity.

    A qualitative image position is not a map bearing or a route. Untracked
    direction labels, unknown positions and low-confidence entities abstain.
    """
    if response["attention"]["target_type"] not in ("person", "object"):
        return None
    if response["confidence"] < .65:
        return None
    track_id = session.world.attention_track_id
    track = session.world.tracks.get(track_id) if track_id else None
    if (track is None or track.missed_packets != 0 or track.confidence < .65
            or track.position not in ("left", "center", "right")):
        return None
    return {"left": "left", "center": "straight", "right": "right"}[track.position]


def advise_live_mission(adapter: BridgeBodyAdapter, *, session: AgentSession,
                        stack2: dict, response: dict, current: dict, before: dict,
                        expected_mission_id: str | None) -> dict:
    """Refresh ownership after inference; never let advice initiate locomotion."""
    result = {"attempted": False, "accepted": False, "reason": "not_eligible"}
    if (stack2.get("coordinator_action") != "continue_local"
            or stack2.get("motor_authority") is not False
            or session.selection is None or session.selection.goal.name != "explore"):
        return result
    direction = tracked_target_direction(session, response)
    if direction is None:
        return {**result, "reason": "no_confident_tracked_target"}
    # The bridge's request-start clock is approximate, not an exposure stamp.
    # Refuse a hint from a window already older than its own short TTL.
    acquired, now = before.get("robot_monotonic_s"), current.get("robot_monotonic_s")
    if (not isinstance(acquired, (int, float)) or not isinstance(now, (int, float))
            or not 0 <= now - acquired <= 4.0):
        return {**result, "reason": "qualitative_view_too_old"}
    if (not isinstance(session.boot_id, str) or current.get("boot_id") != session.boot_id
            or current.get("motion_lease_id") != session.execution.get("lease_id")
            or not expected_mission_id or session.mission_id != expected_mission_id):
        return {**result, "reason": "owner_changed_before_advice"}
    try:
        fresh = adapter.mission().get("mission")
        if (not isinstance(fresh, dict) or fresh.get("state") not in ("starting", "running")
                or fresh.get("id") != expected_mission_id
                or fresh.get("lease_id") != session.execution.get("lease_id")):
            return {**result, "reason": "owner_changed_before_advice"}
        result = {"attempted": True, "accepted": False, "direction": direction,
                  "reason": "bridge_rejected"}
        adapter.bias_mission(boot_id=session.boot_id, mission_id=expected_mission_id,
                             lease_id=fresh["lease_id"], preferred_direction=direction, ttl_s=4.0)
        return {**result, "accepted": True, "reason": "advisory_accepted"}
    except BodyDispatchError as exc:
        return {**result, "reason": f"advisory_unavailable:{type(exc).__name__}"}


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
                 semantic: dict, slam_map: dict | None = None) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for frame in frames:
        (directory / f"{frame.frame_id}.jpg").write_bytes(frame.jpeg)
    (directory / "contact_sheet.jpg").write_bytes(contact_sheet)
    artifacts = {"contact_sheet": str(directory / "contact_sheet.jpg"),
                 "latest_frame": str(directory / f"{frames[-1].frame_id}.jpg")}
    if isinstance(slam_map, dict) and slam_map.get("ok") is True:
        map_path = directory / "slam_map.json"
        map_path.write_text(json.dumps(slam_map, separators=(",", ":")))
        artifacts["slam_map"] = str(map_path)
    packet["artifacts"] = artifacts
    (directory / "observation_packet.json").write_text(json.dumps(packet, indent=2, sort_keys=True))
    (directory / "model_response.json").write_text(json.dumps(response, indent=2, sort_keys=True))
    (directory / "semantic_decision.json").write_text(json.dumps(semantic, indent=2, sort_keys=True))


def speak_camera_occluded(base_url: str, *, audio_file: Path | None, gain: float) -> bool:
    """Speak a fixed reaction; no camera bytes enter TTS or the bridge audio POST."""
    from scripts.run_social_voice import bridge_playback, edge_sonia, supplied_audio
    text = ("My camera has gone dark. If that is a lens cap, excellent privacy; "
            "less excellent sightseeing. Please switch me to no-cloud vision mode.")
    audio = supplied_audio(audio_file) if audio_file else edge_sonia(text, 'en-GB-SoniaNeural')
    return bridge_playback(audio, gain, base_url)


def trace_event(path: Path, event: dict) -> None:
    with path.open('a') as stream:
        stream.write(json.dumps(event, sort_keys=True) + '\n')
    print(json.dumps(event, sort_keys=True), flush=True)


def main() -> int:
    args = parse_args()
    bias_mission_enabled = bool(getattr(args, "bias_mission", False))
    if args.duration < 5 or args.duration > 600:
        raise SystemExit("--duration must be within [5, 600]")
    if args.max_inferences < 0 or args.max_inferences > 1000:
        raise SystemExit("--max-inferences must be within [0, 1000]")
    if args.inference_interval < 0 or args.inference_interval > 600:
        raise SystemExit("--inference-interval must be within [0, 600]")
    if args.resume_session:
        raise SystemExit("legacy session-only resume lacks journal/lease identity; use --resume-checkpoint")
    if bias_mission_enabled and args.allow_no_mission:
        raise SystemExit("--bias-mission requires a live autonomous mission")
    if getattr(args, 'no_cloud_vision', False) and (bias_mission_enabled or args.mock_response):
        raise SystemExit('--no-cloud-vision cannot use visual bias or mock vision responses')
    audio_gain = getattr(args, 'audio_gain', 3.0)
    if not math.isfinite(audio_gain) or not 0.5 <= audio_gain <= 6.0:
        raise SystemExit('--audio-gain must be finite and within 0.5..6')
    memory_path = getattr(args, "memory_file", None)
    place_label = getattr(args, "place_label", None)
    if place_label and not memory_path:
        raise SystemExit("--place-label requires --memory-file")
    memory = SemanticMemory(memory_path) if memory_path is not None else None

    initial_mission = fetch_json(args.base_url, "/mission", args.timeout).get("mission")
    if not args.allow_no_mission and (
        not isinstance(initial_mission, dict) or initial_mission.get("state") not in {"starting", "running"}
    ):
        raise SystemExit("no active autonomous mission; moving eyes did not start")

    raw_mock = json.loads(args.mock_response.read_text()) if args.mock_response else None
    # A JSON array of validated packets exercises changing scenes through the
    # real runner without manufacturing repeated temporal observations.
    mock_sequence = ([validate_perception_response(item) for item in raw_mock]
                     if isinstance(raw_mock, list) else None)
    if mock_sequence is not None and not mock_sequence:
        raise SystemExit("--mock-response sequence must not be empty")
    mock = (validate_perception_response(raw_mock)
            if raw_mock is not None and mock_sequence is None else None)
    run_id = time.strftime("%Y%m%dT%H%M%S") + "-moving-" + uuid.uuid4().hex[:8]
    output_dir = args.output_root / run_id
    output_dir.mkdir(parents=True, exist_ok=False)
    mode = VisionMode(getattr(args, 'vision_switch_file', None) or output_dir / 'vision_mode',
                      no_cloud_vision=getattr(args, 'no_cloud_vision', False))
    mode.refresh()  # A pre-existing runtime switch forbids credential loading.
    client = None if mode.no_cloud_vision or mock is not None or mock_sequence is not None else gemini_client(args)
    trace_path = output_dir / "trace.jsonl"
    # Keep up to thirty seconds of camera history, not only the last three
    # seconds of a nine-panel request. Inference never blocks the capture ring.
    capture = ContinuousMovingCapture(args.base_url, fps=args.fps,
                                      max_frames=min(90, max(18, int(args.fps * 30))),
                                      timeout_s=min(args.timeout, 8.0))
    mission_id = initial_mission.get("id") if isinstance(initial_mission, dict) else None
    fixed_goal = EmbodiedGoal(args.goal, args.target, args.mark) if args.goal else None
    session = AgentSession(robot_id="BEEP", mission_id=mission_id, fixed_goal=fixed_goal,
                           memory=memory)
    observer = None
    advisor = BridgeBodyAdapter(args.base_url, args.timeout) if bias_mission_enabled else None
    bias_accepted = 0
    session_path = output_dir / "agent_session.json"
    deadline = time.monotonic() + args.duration
    inference_count = 0
    inference_errors = 0
    repeated_window_skips = 0
    last_processed_frame_id: str | None = None
    last_inference_finished = 0.0
    capture_started = False
    awaiting_switch = False
    occlusion_events = 0
    if mode.cloud_allowed:
        capture.start()
        capture_started = True
    try:
        if capture_started and not capture.ring.wait_for(1, min(args.duration, args.timeout)):
            raise RuntimeError(f"moving camera did not produce a frame: {capture.stats()}")
        while time.monotonic() < deadline:
            mode.refresh()
            if mode.no_cloud_vision or awaiting_switch:
                if capture_started:
                    capture.stop()
                    capture_started = False
                if not args.allow_no_mission:
                    monitored = fetch_json(args.base_url, "/mission", args.timeout).get("mission")
                    if not isinstance(monitored, dict) or monitored.get("state") not in {"starting", "running"}:
                        break
                time.sleep(min(0.25, max(0., deadline - time.monotonic())))
                continue
            if mock_sequence is not None and inference_count >= len(mock_sequence):
                break
            wait_s = last_inference_finished + args.inference_interval - time.monotonic()
            if inference_count and wait_s > 0:
                time.sleep(min(0.25, wait_s))
                continue
            mission_payload = fetch_json(args.base_url, "/mission", args.timeout)
            mission = mission_payload.get("mission")
            if not args.allow_no_mission and (
                not isinstance(mission, dict) or mission.get("state") not in {"starting", "running"}
            ):
                break
            if (last_processed_frame_id is not None and
                    capture.ring.latest(1)[-1].frame.frame_id == last_processed_frame_id):
                repeated_window_skips += 1
                capture.ring.wait_for_newer(last_processed_frame_id,
                    min(0.25, max(0.0, deadline - time.monotonic())))
                continue
            try:
                slam_map = fetch_slam_map(args.base_url, args.timeout)
            except Exception as exc:
                slam_map = {"ok": False, "reason": f"fetch_failed:{type(exc).__name__}"}
            frames, sheet, packet = capture.temporal_window(args.panel, slam_map=slam_map)
            # Contact sheets contain bright labels; inspect every *raw* JPEG,
            # including the newest ring sample, before any provider call.
            if mode.refresh():
                continue
            try:
                for frame in frames:
                    ensure_visible(frame.jpeg)
                ensure_visible(capture.ring.latest(1)[-1].frame.jpeg)
            except CameraPrivacyError as exc:
                awaiting_switch = True
                occlusion_events += 1
                event = {"state": "camera_occluded", "window": inference_count,
                         "latest_frame_id": frames[-1].frame_id,
                         "reason": "dark_or_unreadable_local_frame",
                         "dispatch_performed": False, "occluded_frame_cloud_calls": 0,
                         "vision_switch_file": str(mode.switch_file)}
                if isinstance(exc, CameraOccluded):
                    event["mean_luminance"] = exc.visibility.mean_luminance
                    event["luminance_stddev"] = exc.visibility.luminance_stddev
                try:
                    event["audio_confirmed"] = speak_camera_occluded(args.base_url,
                        audio_file=getattr(args, 'occlusion_audio_file', None), gain=audio_gain)
                except Exception as audio_exc:
                    event["audio_confirmed"] = False
                    event["audio_error"] = type(audio_exc).__name__
                trace_event(trace_path, event)
                continue
            packet["semantic_target"] = None if args.target == "scene" else args.target
            packet["vision_context"] = {
                "previous_scene_summary": session.world.last_scene_summary,
                "previous_changes": session.world.last_changes[-4:],
                **({"semantic_memory": memory.prompt_summary()} if memory is not None else {}),
                "execution_outcome": session.execution["last_outcome"],
                "current_frame_diversity": [
                    {"frame_id": frame.frame_id,
                     "difference_from_previous": meta["quality"].get("difference_from_previous"),
                     "brightness": meta["quality"].get("brightness"),
                     "edge_stddev": meta["quality"].get("edge_stddev")}
                    for frame, meta in zip(frames, packet["frames"])
                ],
            }
            packet["mission"] = mission
            requested_latest = frames[-1].frame_id
            # The ring may advance between the cheap preflight and panel build.
            # Never give the supervisor (or a model) a second event for one
            # acquisition, even if the image content happens to look identical.
            if requested_latest == last_processed_frame_id:
                repeated_window_skips += 1
                capture.ring.wait_for_newer(requested_latest,
                    min(0.25, max(0.0, deadline - time.monotonic())))
                continue
            if mode.refresh():
                continue
            last_processed_frame_id = requested_latest
            started_at = time.time()
            try:
                if mock_sequence is not None:
                    response = mock_sequence[inference_count]
                elif mock is not None:
                    response = mock
                else:
                    if mode.refresh():
                        continue
                    assert client is not None
                    response = client.analyze(packet, sheet, frames[-1].jpeg)
            except GeminiError as exc:
                # Invalid model output must not terminate the independent
                # LiDAR/map mission or manufacture a motor command. Keep
                # collecting new views and retry at the next window.
                inference_errors += 1
                event = {"window": inference_count, "model_error": str(exc)[:500],
                         "latest_frame_id": requested_latest,
                         "mission_id": mission_id, "dispatch_performed": False}
                with trace_path.open("a") as trace:
                    trace.write(json.dumps(event, sort_keys=True) + "\n")
                print(json.dumps(event, sort_keys=True), flush=True)
                inference_count += 1
                last_inference_finished = time.monotonic()
                if args.max_inferences and inference_count >= args.max_inferences:
                    break
                continue
            completed_at = time.time()
            newest_latest = capture.ring.latest(1)[-1].frame.frame_id
            packet["inference"] = {
                "started_at": started_at,
                "completed_at": completed_at,
                "latency_s": round(completed_at - started_at, 3),
                "requested_latest_frame_id": requested_latest,
                "newest_frame_id_at_completion": newest_latest,
                "newer_window_available": newest_latest != requested_latest,
            }
            # Use the real bridge acquisition clock and exact lease/map identity.
            # Host capture time remains qualitative; there is no metric camera TF.
            current_status_for_bias = None
            before_status_for_bias = None
            if not args.allow_no_mission:
                # Inference can outlive several occupancy publications. Pair
                # a fresh map with a fresh status, preserving the older frame's
                # capture-time snapshot solely as qualitative provenance.
                current_map = fetch_slam_map(args.base_url, args.timeout)
                current = fetch_json(args.base_url, "/status", args.timeout)
                after = packet["bridge_status"]
                before = after.get("capture_before") or {}
                try:
                    if not isinstance(mission, dict):
                        raise LiveEvidenceUnavailable("mission disappeared during inference")
                    if observer is None:
                        observer = LiveBridgeObserver(output_dir, current, mission, current_map,
                            resume_checkpoint=getattr(args, "resume_checkpoint", None),
                            fixed_goal=fixed_goal)
                        session = observer.session
                        session.memory = memory
                    stack2 = observer.observe(frame_id=f"{run_id}-{requested_latest}",
                        before=before, after=after, current=current,
                        mission=mission, slam_map=current_map, perception=response)
                    current_status_for_bias = current
                    before_status_for_bias = before
                except LiveEvidenceUnavailable as exc:
                    event = {"window": inference_count, "provenance_withheld": str(exc),
                             "latest_frame_id": requested_latest, "dispatch_performed": False}
                    with trace_path.open("a") as trace:
                        trace.write(json.dumps(event, sort_keys=True) + "\n")
                    print(json.dumps(event, sort_keys=True), flush=True)
                    inference_count += 1
                    last_inference_finished = time.monotonic()
                    if args.max_inferences and inference_count >= args.max_inferences:
                        break
                    continue
            else:
                stack2 = {"status": "bench_only", "motor_authority": False}
                session.update(response, observed_at=frames[-1].captured_at,
                               packet_id=f"{run_id}-{inference_count:04d}")
            # Only accepted observations become durable notes. Anonymous
            # packet IDs are session-scoped, never promoted to named people.
            if memory is not None:
                try:
                    memory.record_observation(response, session_id=session.session_id,
                        observed_at=frames[-1].captured_at, place_label=place_label)
                except (OSError, ValueError) as exc:
                    print(json.dumps({"memory_error": type(exc).__name__,
                                      "window": inference_count}), flush=True)
            decision = session.decide(rollout_mode="shadow").to_dict()
            assert session.selection is not None
            bias = {"attempted": False, "accepted": False, "reason": "disabled"}
            if (advisor is not None and observer is not None
                    and current_status_for_bias is not None and before_status_for_bias is not None):
                bias = advise_live_mission(advisor, session=session, stack2=stack2,
                    response=response, current=current_status_for_bias,
                    before=before_status_for_bias,
                    expected_mission_id=mission_id)
                bias_accepted += int(bias["accepted"])
            semantic = {
                "session_id": session.session_id,
                "mission_id": session.mission_id,
                "selection": asdict(session.selection),
                "next_decision": decision,
                "rollout_mode": "shadow",
                "dispatch_performed": False,
                "mission_bias": bias,
                "stack2": stack2,
            }
            session.save(session_path)
            window_dir = output_dir / f"w{inference_count:04d}-{requested_latest}"
            write_window(window_dir, frames, sheet, packet, response, semantic, slam_map)
            event = {
                "window": inference_count,
                "distinct_appearance_count": packet["distinct_appearance_count"],
                "candidate_frame_count": packet["candidate_frame_count"],
                "latest_frame_id": requested_latest,
                "newer_window_available": newest_latest != requested_latest,
                "latency_s": packet["inference"]["latency_s"],
                "scene": response["scene"]["summary"],
                "confidence": response["confidence"],
                "next_skill": decision["skill_call"]["name"],
                "dispatch_performed": False,
                "mission_bias": bias,
            }
            with trace_path.open("a") as trace:
                trace.write(json.dumps(event, sort_keys=True) + "\n")
            print(json.dumps(event, sort_keys=True), flush=True)
            inference_count += 1
            last_inference_finished = time.monotonic()
            if args.max_inferences and inference_count >= args.max_inferences:
                break
    finally:
        if capture_started:
            capture.stop()

    summary = {
        "ok": True,
        "run_id": run_id,
        "output_dir": str(output_dir),
        "panel_count": args.panel,
        "inference_interval_s": args.inference_interval,
        "inferences": inference_count,
        "inference_errors": inference_errors,
        "repeated_window_skips": repeated_window_skips,
        "occlusion_events": occlusion_events,
        "vision_mode": ("no_cloud_vision" if mode.no_cloud_vision else
                        "awaiting_no_cloud_vision" if awaiting_switch else "cloud_vision"),
        "vision_switch_file": str(mode.switch_file),
        "capture": capture.stats(),
        "dispatch_performed": False,
        "bias_mission_enabled": bias_mission_enabled,
        "bias_accepted": bias_accepted,
        "agent_session": str(session_path),
        "agent_session_id": session.session_id,
        "stack2_checkpoint": None if observer is None else str(observer.checkpoint.path),
        "event_journal": None if observer is None else str(observer.journal.path),
        "mission_id": session.mission_id,
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True))
    print(json.dumps(summary, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
