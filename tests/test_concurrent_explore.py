import argparse
import io
import json
import subprocess
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest.mock import Mock

from PIL import Image

from scripts.run_beep_explore import run


class FakeBridge:
    def __init__(self, *, ready=True):
        self.ready = ready
        self.calls = []
        self.active = None
        self.lease = None

    def mission(self):
        return {"mission": self.active,
                "status": {"moving": bool(self.active and self.active["state"] == "running"),
                           "motion_lease_id": self.lease,
                           "scan_age_s": 0.03 if self.ready else 2.0,
                           "slam": {"usable": self.ready}}}

    def _json(self, path, body):
        self.calls.append((path, body))
        if path == "/mission/start":
            self.lease = "lease-exact"
            self.active = {"id": "mission-exact", "lease_id": self.lease, "state": "running"}
            return {"accepted": True, "mission": dict(self.active)}
        if path == "/mission/cancel":
            if self.active and body == {"mission_id": self.active["id"], "lease_id": self.lease}:
                self.active["state"] = "cancelled"
                self.lease = None
                return {"ok": True, "cancelled": True}
        raise AssertionError((path, body))


def options():
    return argparse.Namespace(base_url="http://beep", duration=5, panel=9, fps=3.0,
                              timeout=1.0, inference_interval=2.0, target="scene",
                              mock_response=None, max_inferences=1, output_root="captures/test")


class ConcurrentExploreTests(unittest.TestCase):
    def test_real_child_and_http_bridge_keep_mission_running_during_perception(self):
        state = {"mission": None, "lease": None, "starts": 0, "cancels": 0,
                 "captured_while_running": 0}
        image = io.BytesIO()
        Image.new("RGB", (160, 120), "gray").save(image, format="JPEG")

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/mission":
                    payload = {"mission": state["mission"], "status": self.status()}
                elif self.path == "/status":
                    payload = self.status()
                elif self.path == "/slam_map":
                    payload = {"ok": False, "reason": "fake_map"}
                elif self.path.startswith("/frame.jpg"):
                    if state["mission"] and state["mission"]["state"] == "running":
                        state["captured_while_running"] += 1
                    self.reply(image.getvalue(), "image/jpeg")
                    return
                else:
                    self.send_error(404)
                    return
                self.reply(json.dumps(payload).encode(), "application/json")

            def status(self):
                return {"moving": bool(state["lease"]), "motion_lease_id": state["lease"],
                        "scan_age_s": 0.03, "slam": {"usable": True, "pose_valid": True},
                        "pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}, "sectors": {"front": 2.0}}

            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                if self.path == "/mission/start":
                    self_outer = state
                    self_outer["starts"] += 1
                    self_outer["lease"] = "lease-http"
                    self_outer["mission"] = {"id": "mission-http", "lease_id": "lease-http",
                                             "state": "running"}
                    payload = {"accepted": True, "mission": self_outer["mission"]}
                elif self.path == "/mission/cancel":
                    if body != {"mission_id": "mission-http", "lease_id": "lease-http"}:
                        self.send_error(409)
                        return
                    state["cancels"] += 1
                    state["mission"]["state"] = "cancelled"
                    state["lease"] = None
                    payload = {"ok": True, "cancelled": True}
                else:
                    self.send_error(405)
                    return
                self.reply(json.dumps(payload).encode(), "application/json")

            def reply(self, body, content_type):
                self.send_response(200)
                self.send_header("Content-Type", content_type)
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
                args = options()
                args.base_url = f"http://127.0.0.1:{server.server_port}"
                args.mock_response = Path(__file__).parent / "fixtures" / "eyes_response.json"
                args.output_root = Path(directory)
                result = run(args)
                self.assertEqual(result["eyes_exit_code"], 0, result["eyes_error"])
                self.assertEqual(result["final_mission_state"], "cancelled")
                self.assertTrue(result["original_lease_released"])
                self.assertEqual(state["starts"], 1)
                self.assertEqual(state["cancels"], 1)
                self.assertGreaterEqual(state["captured_while_running"], 1)
                self.assertEqual(len(list(Path(directory).glob("*/summary.json"))), 1)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_starts_coverage_then_eyes_shadow_while_mission_continues(self):
        bridge = FakeBridge()

        def model_waiting(command, **kwargs):
            assert bridge.active is not None
            self.assertEqual(bridge.active["state"], "running")
            self.assertEqual(bridge.lease, "lease-exact")
            self.assertEqual([path for path, _ in bridge.calls], ["/mission/start"])
            self.assertEqual(bridge.calls[0][1]["mode"], "coverage")
            # The child is shadow-only by construction: no arming flag exists.
            self.assertNotIn("--arm", command)
            self.assertNotIn("--mode", command)
            self.assertIn("run_moving_eyes.py", command[1])
            self.assertLessEqual(kwargs["timeout"], 25)
            time.sleep(0.25)  # Simulated slow model; onboard owner keeps its gait.
            self.assertEqual(bridge.active["state"], "running")
            self.assertEqual(bridge.lease, "lease-exact")
            self.assertEqual([path for path, _ in bridge.calls], ["/mission/start"])
            return subprocess.CompletedProcess(command, 0, stdout='{"inferences":1}', stderr="")

        result = run(options(), adapter=bridge, runner=model_waiting)
        self.assertTrue(result["accepted"])
        self.assertEqual(result["eyes_exit_code"], 0)
        self.assertIsNotNone(bridge.active)
        self.assertEqual(bridge.active["state"], "cancelled")
        self.assertIsNone(bridge.lease)
        self.assertEqual(bridge.calls[1], ("/mission/cancel",
                                           {"mission_id": "mission-exact", "lease_id": "lease-exact"}))

    def test_model_failure_still_cancels_exact_mission(self):
        bridge = FakeBridge()
        with self.assertRaisesRegex(RuntimeError, "model failed"):
            run(options(), adapter=bridge, runner=Mock(side_effect=RuntimeError("model failed")))
        self.assertIsNotNone(bridge.active)
        self.assertEqual(bridge.active["state"], "cancelled")
        self.assertIsNone(bridge.lease)

    def test_no_start_when_map_or_scan_unusable(self):
        bridge = FakeBridge(ready=False)
        with self.assertRaisesRegex(RuntimeError, "fresh scan"):
            run(options(), adapter=bridge, runner=Mock())
        self.assertEqual(bridge.calls, [])

    def test_never_cancels_replacement_mission(self):
        bridge = FakeBridge()
        def replaced(*_, **__):
            bridge.active = {"id": "replacement", "lease_id": "lease-new", "state": "running"}
            bridge.lease = "lease-new"
            return subprocess.CompletedProcess([], 0, stdout="", stderr="")
        run(options(), adapter=bridge, runner=replaced)
        self.assertEqual([path for path, _ in bridge.calls], ["/mission/start"])
        self.assertEqual(bridge.lease, "lease-new")

    def test_terminal_mission_waits_for_lease_release_without_cancel(self):
        class FinishingBridge(FakeBridge):
            finish_polls = 0
            def mission(self):
                if self.active is not None and self.active["state"] == "completed":
                    self.finish_polls += 1
                    if self.finish_polls >= 3:
                        self.lease = None
                return super().mission()
        bridge = FinishingBridge()
        def finishes(*_, **__):
            assert bridge.active is not None
            bridge.active["state"] = "completed"
            return subprocess.CompletedProcess([], 0, stdout="", stderr="")
        result = run(options(), adapter=bridge, runner=finishes)
        self.assertEqual(result["final_mission_state"], "completed")
        self.assertTrue(result["original_lease_released"])
        self.assertEqual([path for path, _ in bridge.calls], ["/mission/start"])


if __name__ == "__main__":
    unittest.main()
