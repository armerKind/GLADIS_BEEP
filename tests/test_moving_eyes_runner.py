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


ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "eyes_response.json"


def jpeg():
    output = io.BytesIO()
    Image.new("RGB", (160, 120), "gray").save(output, format="JPEG")
    return output.getvalue()


class MovingEyesRunnerTests(unittest.TestCase):
    def test_mock_moving_mission_builds_panel_and_semantic_plan_without_dispatch(self):
        frame = jpeg()
        seen_posts = []
        started_at = time.time()

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path.startswith("/frame.jpg"):
                    body = frame
                    content_type = "image/jpeg"
                elif self.path == "/status":
                    body = json.dumps({
                        "moving": True,
                        "motion_lease_id": "lease-1",
                        "last_command": "forward",
                        "scan_age_s": 0.02,
                        "sectors": {"front": 1.2},
                        "slam": {"usable": True},
                    }).encode()
                    content_type = "application/json"
                elif self.path == "/mission":
                    body = json.dumps({"mission": {
                        "id": "mission-1", "state": "running", "started_at": started_at,
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
                seen_posts.append(self.path)
                self.send_error(405)

            def log_message(self, format, *args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                command = [
                    sys.executable, "scripts/run_moving_eyes.py",
                    "--base-url", f"http://127.0.0.1:{server.server_port}",
                    "--panel", "4", "--fps", "6", "--duration", "5",
                    "--inference-interval", "0.1",
                    "--max-inferences", "2", "--mock-response", str(FIXTURE),
                    "--output-root", directory,
                ]
                environment = dict(os.environ, PYTHONPATH=str(ROOT))
                result = subprocess.run(command, cwd=ROOT, env=environment, text=True,
                                        capture_output=True, timeout=20)
                self.assertEqual(result.returncode, 0, result.stderr)
                run_directories = list(Path(directory).iterdir())
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
                semantics = [json.loads((window / "semantic_decision.json").read_text())
                             for window in sorted(windows)]
                self.assertEqual(packet["capture_mode"], "moving_temporal_ring")
                self.assertEqual(len(packet["frames"]), 4)
                self.assertFalse(semantics[0]["dispatch_performed"])
                self.assertEqual(semantics[0]["session_id"], semantics[1]["session_id"])
                self.assertEqual(semantics[0]["mission_id"], "mission-1")
                self.assertTrue((run / "agent_session.json").exists())
                self.assertEqual(seen_posts, [])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
