import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from beep_bridge import beep_bridge as bridge
from beep_observer import (
    Assessment,
    BridgeStopSink,
    DirectoryFrameSource,
    EvidenceStore,
    ExternalObserver,
    Frame,
    MetadataRiskEvaluator,
    ObserverConfig,
    RiskLevel,
    StopOutcome,
)


class ListSource:
    def __init__(self, frames):
        self.frames = iter(frames)

    def read(self):
        return next(self.frames, None)


class FakeStopSink:
    def __init__(self, authenticated=True):
        self._authenticated = authenticated
        self.calls = []

    @property
    def authenticated(self):
        return self._authenticated

    def stop(self, assessment):
        self.calls.append(assessment)
        return StopOutcome(True, True, "observer_stop_delivered", mission_id="mission-7")


class ExternalObserverTests(unittest.TestCase):
    def frame(self, sequence, risk="clear"):
        return Frame("fixture", sequence, 1000.0 + sequence, b"private-image", ".jpg",
                     {"risk": risk, "reasons": ["contact_risk"] if risk == "stop" else [],
                      "metrics": {"clearance_m": 0.05 if risk == "stop" else 1.0}})

    def test_record_only_never_calls_stop_and_retains_only_risky_frame(self):
        with tempfile.TemporaryDirectory() as root:
            source = ListSource([self.frame(1, "clear"), self.frame(2, "stop")])
            sink = FakeStopSink()
            observer = ExternalObserver(source, MetadataRiskEvaluator(), EvidenceStore(root),
                                        ObserverConfig(mode="record"), sink)
            clear_event = observer.run_once()
            stop_event = observer.run_once()

            assert clear_event is not None
            assert stop_event is not None
            self.assertEqual(clear_event["stop"]["reason"], "risk_below_stop")
            self.assertIsNone(clear_event["retained_path"])
            self.assertEqual(stop_event["stop"]["reason"], "record_only_mode")
            self.assertTrue(Path(stop_event["retained_path"]).exists())
            self.assertEqual(sink.calls, [])
            lines = Path(root, "events.jsonl").read_text(encoding="utf-8").splitlines()
            self.assertEqual(len(lines), 2)
            self.assertNotIn("private-image", "\n".join(lines))

    def test_stop_mode_requires_authentication_and_rate_limits(self):
        frames = [self.frame(1, "stop"), self.frame(2, "stop")]
        with tempfile.TemporaryDirectory() as root:
            sink = FakeStopSink(authenticated=True)
            ticks = iter([10.0, 10.5])
            observer = ExternalObserver(ListSource(frames), MetadataRiskEvaluator(), EvidenceStore(root),
                                        ObserverConfig(mode="stop", stop_cooldown_s=2.0), sink,
                                        clock=lambda: next(ticks))
            first = observer.run_once()
            second = observer.run_once()

            assert first is not None
            assert second is not None
            self.assertTrue(first["stop"]["delivered"])
            self.assertEqual(second["stop"]["reason"], "stop_rate_limited")
            self.assertEqual(len(sink.calls), 1)

        with tempfile.TemporaryDirectory() as root:
            unauth = FakeStopSink(authenticated=False)
            observer = ExternalObserver(ListSource([self.frame(3, "stop")]), MetadataRiskEvaluator(),
                                        EvidenceStore(root), ObserverConfig(mode="stop"), unauth)
            event = observer.run_once()
            assert event is not None
            self.assertEqual(event["stop"]["reason"], "stop_sink_unauthenticated")
            self.assertEqual(unauth.calls, [])

    def test_directory_source_loads_bounded_sidecar_schema(self):
        with tempfile.TemporaryDirectory() as root:
            image = Path(root, "001.jpg")
            image.write_bytes(b"jpeg")
            Path(root, "001.jpg.json").write_text(json.dumps({
                "captured_at": 42.0, "risk": "watch", "reasons": ["low_clearance"]
            }), encoding="utf-8")
            frame = DirectoryFrameSource(root, source_name="room-corner").read()
            assert frame is not None
            self.assertEqual(frame.source, "room-corner")
            self.assertEqual(frame.captured_at, 42.0)
            self.assertEqual(frame.metadata["risk"], "watch")

    def test_replay_cli_produces_private_local_evidence(self):
        with tempfile.TemporaryDirectory() as root:
            source_dir = Path(root, "source")
            evidence_dir = Path(root, "evidence")
            source_dir.mkdir()
            Path(source_dir, "001.jpg").write_bytes(b"jpeg")
            Path(source_dir, "001.jpg.json").write_text(
                json.dumps({"risk": "watch", "reasons": ["fixture"]}), encoding="utf-8")
            env = dict(os.environ, PYTHONPATH=str(Path(__file__).resolve().parents[1]))
            completed = subprocess.run([
                sys.executable, "scripts/run_external_observer.py",
                "--source-dir", str(source_dir), "--evidence-dir", str(evidence_dir),
            ], cwd=Path(__file__).resolve().parents[1], env=env, text=True,
               stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10, check=True)
            self.assertIn('"processed": 1', completed.stdout)
            self.assertTrue(Path(evidence_dir, "events.jsonl").exists())

            env.pop("BEEP_OBSERVER_STOP_TOKEN", None)
            refused = subprocess.run([
                sys.executable, "scripts/run_external_observer.py",
                "--source-dir", str(source_dir), "--evidence-dir", str(evidence_dir),
                "--mode", "stop", "--bridge-url", "http://127.0.0.1:8766",
            ], cwd=Path(__file__).resolve().parents[1], env=env, text=True,
               stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10)
            self.assertNotEqual(refused.returncode, 0)
            self.assertIn("requires --bridge-url", refused.stderr)

    def test_bridge_sink_sends_exact_current_mission_only(self):
        assessment = Assessment("event-1", "cam", 1, 1.0, 2.0, RiskLevel.STOP, ("contact",), {})
        sink = BridgeStopSink("http://beep.invalid:8766", "secret")
        calls = []

        def request(method, path, payload=None):
            calls.append((method, path, payload))
            if path == "/mission":
                return {"mission": {"id": "mission-9"}}
            return {"ok": True, "reason": "observer_stop_delivered"}

        with patch.object(sink, "_request", side_effect=request):
            outcome = sink.stop(assessment)
        self.assertTrue(outcome.delivered)
        self.assertEqual(outcome.mission_id, "mission-9")
        self.assertEqual(calls[1][1], "/observer/stop")
        self.assertEqual(calls[1][2]["mission_id"], "mission-9")

    def test_bridge_observer_capability_is_authenticated_exact_and_rate_limited(self):
        mission = {"id": "mission-4", "lease_id": "lease-4", "state": "running",
                   "started_at": 1.0, "finished_at": None, "duration_s": 120.0}
        cancelled = {"ok": True, "cancelled": True, "reason": "observer_stop_delivered"}
        with patch.object(bridge, "OBSERVER_STOP_TOKEN", "secret"), \
             patch.object(bridge, "_active_mission", mission), \
             patch.object(bridge, "observer_last_stop_at", None), \
             patch.object(bridge, "cancel_autonomous_mission", return_value=dict(cancelled)) as cancel, \
             patch.object(bridge, "remember", return_value=None):
            rejected = bridge.observer_stop_request("wrong", "mission-4", now=10.0)
            stale = bridge.observer_stop_request("secret", "mission-old", now=10.0)
            delivered = bridge.observer_stop_request("secret", "mission-4", "event-4", now=10.0)
            limited = bridge.observer_stop_request("secret", "mission-4", "event-5", now=10.5)

        self.assertEqual(rejected["status_code"], 403)
        self.assertEqual(stale["reason"], "mission_id_mismatch")
        self.assertTrue(delivered["ok"])
        self.assertEqual(limited["status_code"], 429)
        cancel.assert_called_once_with("mission-4", source="external_observer")


if __name__ == "__main__":
    unittest.main()
