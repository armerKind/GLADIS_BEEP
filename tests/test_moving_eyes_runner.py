import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from PIL import Image
from beep_agent.memory import SemanticMemory


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "eyes_response.json"


def jpeg():
    output = io.BytesIO()
    Image.new("RGB", (160, 120), "gray").save(output, format="JPEG")
    return output.getvalue()


class MovingEyesRunnerTests(unittest.TestCase):
    def test_opt_in_memory_persists_outside_run_and_informs_next_window(self):
        self._exercise_moving_mission(memory_enabled=True)

    def test_mock_moving_mission_builds_panel_and_semantic_plan_without_dispatch(self):
        self._exercise_moving_mission()

    def test_opt_in_bias_tracks_center_left_right_without_motor_dispatch(self):
        for position, direction in (("center", "straight"), ("left", "left"), ("right", "right")):
            with self.subTest(position=position):
                response = json.loads(FIXTURE.read_text())
                response["scene"]["people"][0]["position"] = position
                response["scene"]["hazards"] = []  # Glass in the literal fixture must withhold advice.
                self._exercise_moving_mission(bias_mission=True, response_override=response,
                                              expected_direction=direction)

    def test_opt_in_bias_abstains_without_tracked_attention(self):
        response = json.loads(FIXTURE.read_text())
        response["attention"]["target_type"] = "none"
        response["attention"]["target_id"] = None
        response["scene"]["hazards"] = []
        self._exercise_moving_mission(bias_mission=True, response_override=response)

    def test_opt_in_bias_abstains_when_live_owner_changes_at_readback(self):
        response = json.loads(FIXTURE.read_text())
        response["scene"]["hazards"] = []
        self._exercise_moving_mission(bias_mission=True, response_override=response,
                                      replace_owner_on_bias_read=True,
                                      skipped_reason="owner_changed_before_advice")

    def test_person_enters_after_object_and_both_safe_windows_keep_bias(self):
        box = json.loads((ROOT / "tests" / "fixtures" / "box_far_response.json").read_text())
        person = json.loads(FIXTURE.read_text())
        # The published fixtures BOTH contain hazards. Remove only those for
        # the person-entry transition; literal glass must remain a hold case.
        box["scene"]["hazards"] = []
        person["scene"]["hazards"] = []
        self._exercise_moving_mission(bias_mission=True, response_override=[box, person],
                                      expected_directions=["straight", "straight"],
                                      expected_revisions=[1, 1])

    def test_hazard_to_clear_explore_revision_recovers_on_same_shadow_lease(self):
        box_with_hazard = json.loads((ROOT / "tests" / "fixtures" / "box_far_response.json").read_text())
        person = json.loads(FIXTURE.read_text())
        person["scene"]["hazards"] = []
        self._exercise_moving_mission(bias_mission=True,
            response_override=[box_with_hazard, person], expected_directions=[None, "straight"],
            expected_actions=["goal_revision_changed", "continue_local"],
            expected_revisions=[1, 2])

    def test_literal_box_and_person_fixtures_withhold_bias_for_reported_hazards(self):
        box = json.loads((ROOT / "tests" / "fixtures" / "box_far_response.json").read_text())
        person = json.loads(FIXTURE.read_text())
        self._exercise_moving_mission(bias_mission=True, response_override=[box, person],
            expected_actions=["goal_revision_changed", "goal_revision_changed"],
            expected_revisions=[1, 1])

    def _exercise_moving_mission(self, *, bias_mission=False, response_override=None,
                                 expected_direction=None, replace_owner_on_bias_read=False,
                                 skipped_reason=None, expected_directions=None,
                                 expected_actions=None, expected_revisions=None,
                                 memory_enabled=False):
        frame = jpeg()
        seen_posts = []
        map_counter = [0]
        started_at = time.time()

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path.startswith("/frame.jpg"):
                    body = frame
                    content_type = "image/jpeg"
                elif self.path == "/slam_map":
                    map_counter[0] += 1
                    map_snapshot_id = f"grid-{map_counter[0]}"
                    body = json.dumps({
                        "ok": True, "boot_id": "boot-live", "map_epoch": 0,
                        "map_revision": 0, "map_revision_status": "verified",
                        "map_snapshot_id": map_snapshot_id, "frame_id": "map", "width": 4, "height": 4,
                        "resolution_m": 0.1, "origin_x": -0.2, "origin_y": -0.2,
                        "origin_yaw": 0.0, "map_age_s": 0.1, "known_cells": 12,
                        "occupied_cells": 4,
                        "data": [0, 0, 100, 100, -1, -1, 50, 100,
                                 0, 0, -1, -1, 0, 0, 0, 100],
                    }).encode()
                    content_type = "application/json"
                elif self.path == "/status":
                    body = json.dumps({
                        "moving": True,
                        "boot_id": "boot-live", "robot_monotonic_s": time.monotonic(),
                        "motion_lease_id": "lease-1",
                        "last_command": "forward",
                        "scan_age_s": 0.02,
                        "pose": {"x": 0.1, "y": 0.2, "yaw": 0.3, "confidence": 0.9,
                                 "source": "guarded_cartographer_slam"},
                        "sectors": {"front": 1.2},
                        "slam": {"usable": True, "pose_valid": True, "pose_age_s": 0.03,
                                 "map_epoch": 0, "map_revision": 0,
                                 "map_revision_status": "verified", "map_snapshot_id": f"grid-{max(1, map_counter[0])}",
                                 "map_age_s": 0.1, "pose_guard_reason": "moving_update"},
                    }).encode()
                    content_type = "application/json"
                elif self.path == "/mission":
                    # Only the body adapter's exact-owner advisory preflight
                    # sees the replacement; camera/runner polling is irrelevant.
                    replaced = (replace_owner_on_bias_read and
                                self.headers.get('User-Agent') == 'beep-middle-layer/1.0')
                    body = json.dumps({"mission": {
                        "id": "mission-other" if replaced else "mission-1",
                        "lease_id": "lease-other" if replaced else "lease-1",
                        "state": "running", "started_at": started_at,
                        "duration_s": 90, "elapsed_s": time.time() - started_at,
                    }}).encode()
                    content_type = "application/json"
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                if self.path != "/mission/bias":
                    seen_posts.append({"unexpected": self.path})
                    self.send_error(405)
                    return
                size = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(size))
                seen_posts.append(payload)
                body = json.dumps({"ok": True, "accepted": True,
                                   "motor_authority": False, "semantic_execution_enabled": False,
                                   "mission_id": payload["mission_id"], "lease_id": payload["lease_id"]}).encode()
                self.send_response(202)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, format, *args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)

        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                output_root = Path(directory) / "runs"
                response_path = FIXTURE
                if response_override is not None:
                    response_path = Path(directory) / "fixture.json"
                    response_path.write_text(json.dumps(response_override))
                command = [
                    sys.executable, "scripts/run_moving_eyes.py",
                    "--base-url", f"http://127.0.0.1:{server.server_port}",
                    "--panel", "4", "--fps", "6", "--duration", "5",
                    "--inference-interval", "0.1",
                    "--max-inferences", "2", "--mock-response", str(response_path),
                    "--output-root", str(output_root),
                ]
                memory_path = Path(directory) / "persistent" / "memory_bank.json"
                if memory_enabled:
                    command.extend(("--memory-file", str(memory_path),
                                    "--place-label", "living_room"))
                if bias_mission:
                    command.append("--bias-mission")
                environment = dict(os.environ, PYTHONPATH=str(ROOT))
                result = subprocess.run(command, cwd=ROOT, env=environment, text=True,
                                        capture_output=True, timeout=20)
                self.assertEqual(result.returncode, 0, result.stderr)
                run_directories = list(output_root.iterdir())
                self.assertEqual(len(run_directories), 1)
                run = run_directories[0]
                summary = json.loads((run / "summary.json").read_text())
                windows = list(run.glob("w*"))
                self.assertEqual(summary["inferences"], 2)
                self.assertEqual(summary["panel_count"], 4)
                self.assertEqual(summary["inference_interval_s"], 0.1)
                self.assertFalse(summary["dispatch_performed"])
                self.assertEqual(len(windows), 2)
                packet = json.loads((windows[0] / "observation_packet.json").read_text())
                if memory_enabled:
                    bank = SemanticMemory(memory_path)
                    self.assertEqual(len(bank.people), 1)
                    self.assertEqual(bank.places["living_room"]["label"], "living_room")
                    later = json.loads((sorted(windows)[1] / "observation_packet.json").read_text())
                    self.assertIn("living_room", later["vision_context"]["semantic_memory"])
                    self.assertIn("Person", later["vision_context"]["semantic_memory"])
                else:
                    self.assertNotIn("semantic_memory", packet["vision_context"])
                semantics = [json.loads((window / "semantic_decision.json").read_text())
                             for window in sorted(windows)]
                self.assertEqual(packet["capture_mode"], "moving_temporal_ring")
                self.assertEqual(len(packet["frames"]), 1)
                self.assertEqual(packet["requested_panel_count"], 4)
                self.assertEqual(packet["candidate_frame_count"], 1)
                self.assertIsNone(packet["semantic_target"])
                self.assertTrue(packet["navigation_context"]["map_available"])
                self.assertTrue(packet["navigation_context"]["pose_valid"])
                self.assertAlmostEqual(packet["navigation_context"]["view_heading_deg"], 17.2, places=1)
                self.assertTrue(Path(packet["artifacts"]["slam_map"]).exists())
                with Image.open(run / windows[0].name / "contact_sheet.jpg") as panel:
                    self.assertEqual(panel.size, (960, 760))
                self.assertFalse(semantics[0]["dispatch_performed"])
                self.assertEqual(semantics[0]["rollout_mode"], "shadow")
                self.assertIn("skill_call", semantics[0]["next_decision"])
                for window in windows:
                    for name in ("observation_packet.json", "model_response.json",
                                 "semantic_decision.json"):
                        self.assertTrue((window / name).exists(), name)
                    self.assertFalse((window / "policy_decision.json").exists())
                self.assertEqual(semantics[0]["session_id"], semantics[1]["session_id"])
                self.assertEqual(semantics[0]["mission_id"], "mission-1")
                self.assertTrue((run / "agent_session.json").exists())
                self.assertTrue((run / "stack2_checkpoint.json").exists())
                records = [json.loads(row) for row in (run / "event_journal.jsonl").read_text().splitlines()]
                self.assertEqual(len(records), 2)
                self.assertGreaterEqual(map_counter[0], 3)
                self.assertEqual([row["map_revision"] for row in records], [0, 0])
                self.assertNotEqual(records[0]["map_snapshot_id"], records[1]["map_snapshot_id"])
                self.assertTrue(all(row["timestamp_quality"] == "ingest_approximate"
                                    and row["capture_local_transform"] is None for row in records))
                self.assertTrue(all(not item["stack2"]["motor_authority"] for item in semantics))
                actions = expected_actions or (["goal_revision_changed"] * 2
                                               if response_override is None else ["continue_local"] * 2)
                self.assertEqual([item["stack2"]["coordinator_action"] for item in semantics], actions)
                if expected_revisions is not None:
                    self.assertEqual([item["stack2"]["goal_revision"] for item in semantics],
                                     expected_revisions)
                self.assertEqual(summary["bias_mission_enabled"], bias_mission)
                directions = (expected_directions if expected_directions is not None else
                              [expected_direction] * 2)
                sent = [direction for direction in directions if direction is not None]
                self.assertEqual(len(seen_posts), len(sent))
                self.assertEqual(summary["bias_accepted"], len(sent))
                self.assertEqual([item["mission_bias"]["accepted"] for item in semantics],
                                 [direction is not None for direction in directions])
                for payload, direction in zip(seen_posts, sent):
                    self.assertEqual(payload, {
                        "boot_id": "boot-live", "mission_id": "mission-1", "lease_id": "lease-1",
                        "ttl_s": 4.0, "preferred_direction": direction,
                        "preferred_bearing_rad": None,
                    })
                if skipped_reason:
                    self.assertTrue(all(item["mission_bias"]["reason"] == skipped_reason
                                        for item in semantics))
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
