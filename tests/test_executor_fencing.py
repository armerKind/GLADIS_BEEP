"""Offline fencing contract. Fake transport only: no bridge/hardware imports."""
import copy
import json
import multiprocessing
from pathlib import Path
import tempfile
import threading
import unittest
from typing import Any
from unittest.mock import patch

from beep_agent.body import BodyDispatchError
from beep_agent.executor import ClosedLoopExecutor
from beep_agent.executive import EmbodiedGoal
from beep_agent.session import AgentSession, SessionIdentityError
from beep_bridge.semantic_intent import ExecutorFence, IntentMailbox, validate_intent


def claim(instance="one", generation=0, takeover=False):
    return dict(mission_id="mission", lease_id="lease", session_id="session",
                executor_instance_id=instance, expected_generation=generation, takeover=takeover)


def intent(fence, decision="decision", latest=101.0):
    return dict(fence, decision_id=decision, frame_id="frame-" + decision,
                observed_at=latest - 0.5, latest_frame_at=latest,
                expires_at=latest + 4.5, kind="continue")


class FakeBody:
    def __init__(self):
        self.mailbox = IntentMailbox(require_fencing=True)
        self.claims = []
        self.submissions = []
        self.safe = True
        self.delivery = "normal"
        self.lose_claim = False
        self.lock = threading.RLock()

    def mission(self):
        with self.lock:
            return dict(id="mission", lease_id="lease", state="running",
                        executor_fencing_enabled=True, semantic_intents_enabled=True,
                        executor_fence=self.mailbox.fence_snapshot(),
                        intent_outcomes=self.mailbox.snapshot())

    def claim_executor(self, request):
        with self.lock:
            self.claims.append(copy.deepcopy(request))
            result = self.mailbox.claim(request, safe_to_takeover=self.safe)
            if self.lose_claim:
                self.lose_claim = False
                raise BodyDispatchError("lost claim reply")
            return result

    def submit_intent(self, value):
        with self.lock:
            self.submissions.append(copy.deepcopy(value))
            if self.delivery == "lost_before":
                raise BodyDispatchError("lost before admission")
            result = self.mailbox.submit(value, 102.0)
            if self.delivery == "lost_after":
                raise BodyDispatchError("lost after admission")
            if self.delivery == "malformed":
                return {"ok": True, "state": "completed"}
            return result


def stale_process(executor, mission, queue):
    try:
        executor.retry("decision", mission=mission, now=102.0)
    except SessionIdentityError as exc:
        queue.put(str(exc))
    else:
        queue.put("unexpected submission")


class MailboxFencingTests(unittest.TestCase):
    def setUp(self):
        self.box = IntentMailbox(require_fencing=True)
        self.first = self.box.claim(claim())["fence"]

    def test_strict_typed_generation_and_fields(self):
        for bad in (True, 0, -1, 1.0, "1"):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                validate_intent(intent(dict(self.first, executor_generation=bad)), 102.0)
        with self.assertRaises(ValueError):
            ExecutorFence.from_dict(dict(self.first, extra="no"))
        self.assertFalse(self.box.claim(dict(claim(), expected_generation=True))["ok"])

    def test_unclaimed_fenced_mailbox_rejects_legacy_and_forged(self):
        box = IntentMailbox(require_fencing=True)
        packet = intent(self.first)
        self.assertFalse(box.submit(packet, 102.0)["ok"])
        del packet["executor_instance_id"]
        del packet["executor_generation"]
        self.assertFalse(box.submit(packet, 102.0)["ok"])

    def test_initial_claim_is_idempotent_and_competing_claim_is_cas_rejected(self):
        self.assertEqual(self.box.claim(claim())["fence"], self.first)
        self.assertEqual(self.box.claim(claim("two"))["reason"], "executor_generation_conflict")

    def test_resume_requires_explicit_takeover_and_server_quiescence(self):
        self.assertFalse(self.box.claim(claim("two", 1), safe_to_takeover=True)["ok"])
        self.assertFalse(self.box.claim(claim("two", 1, True))["ok"])
        self.assertTrue(self.box.claim(claim("two", 1, True), safe_to_takeover=True)["ok"])

    def test_takeover_fences_stale_new_and_duplicate_requests(self):
        packet = intent(self.first)
        self.box.submit(packet, 102.0)
        second = self.box.claim(claim("two", 1, True), safe_to_takeover=True)["fence"]
        for old in (packet, intent(self.first, "new", 102.0)):
            self.assertEqual(self.box.submit(old, 103.0)["reason"], "stale_executor_fence")
        self.assertIsNone(self.box.take(103.0))
        self.assertFalse(self.box.authorizes(packet, 103.0))
        self.assertTrue(self.box.submit(intent(second, "new", 102.0), 103.0)["ok"])
        self.assertFalse(self.box.claim(claim(), safe_to_takeover=True)["ok"])

    def test_takeover_keeps_in_flight_delivery_unknown_and_late_worker_cannot_revive(self):
        packet = intent(self.first)
        self.box.submit(packet, 102.0)
        self.box.take(102.0)
        result = self.box.claim(claim("two", 1, True), safe_to_takeover=True)
        self.assertEqual(result["intent_outcomes"][0]["state"], "unknown")
        self.box.outcome("decision", "completed", "late worker")
        self.assertEqual(self.box.snapshot()[0]["state"], "unknown")

    def test_dedup_even_after_expiry_and_mutated_id_rejected(self):
        packet = intent(self.first)
        self.box.submit(packet, 102.0)
        self.box.take(102.0)
        self.box.outcome("decision", "completed", "done")
        self.assertEqual(self.box.submit(packet, 200.0)["state"], "completed")
        self.assertIsNone(self.box.take(200.0))
        self.assertEqual(self.box.submit(dict(packet, kind="stop"), 102.0)["reason"], "decision_id_conflict")

    def test_closed_mailbox_cannot_be_reclaimed_or_accept_new_work(self):
        self.box.close("mission done")
        self.assertFalse(self.box.claim(claim())["ok"])
        self.assertFalse(self.box.submit(intent(self.first), 102.0)["ok"])

    def test_competing_takeovers_one_winner_under_bridge_lock(self):
        lock = threading.Lock()
        barrier = threading.Barrier(3)
        results = []
        def run(instance):
            barrier.wait()
            with lock:
                results.append(self.box.claim(claim(instance, 1, True), safe_to_takeover=True))
        workers = [threading.Thread(target=run, args=(name,)) for name in ("two", "three")]
        for worker in workers:
            worker.start()
        barrier.wait()
        for worker in workers:
            worker.join(2)
            self.assertFalse(worker.is_alive())
        self.assertEqual(sum(item["ok"] for item in results), 1)


class ExecutorFencingTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "session.json"
        self.body = FakeBody()
        self.session = AgentSession(robot_id="offline", mission_id="mission", session_id="session",
                                    fixed_goal=EmbodiedGoal("explore", "space"))
        self.executor = self.make(self.session)
        for name in ("socket.create_connection", "urllib.request.urlopen"):
            blocker = patch(name, side_effect=AssertionError("network forbidden"))
            blocker.start()
            self.addCleanup(blocker.stop)

    def make(self, session, path=None):
        return ClosedLoopExecutor(session, self.body, mode="supervised", armed=True,
                                  state_path=path or self.path)

    def acquire(self):
        return self.executor.acquire(self.body.mission())

    def stage(self):
        fence = self.acquire()
        packet = intent(fence.to_dict())
        self.executor.session.execution["issued"]["decision"] = {"intent": packet, "state": "sending"}
        self.executor.session.execution["latest_frame_at"] = 101.0
        self.executor.save()
        return packet

    def step(self, executor=None, latest=101.0):
        response = json.loads((Path(__file__).parent / "fixtures/eyes_response.json").read_text())
        response["scene"].update(people=[], objects=[], hazards=[])
        response["attention"] = {"target_type": "none", "target_id": None, "reason": "exploration"}
        response["plan"]["steps"] = [{"skill": "advance", "distance_m": 0.2, "reason": "clear"}]
        status = dict(scan_seen=True, scan_age_s=0.01, motion_lease_id="lease",
                      slam=dict(usable=True, pose_valid=True, pose_age_s=0.01, map_age_s=0.01),
                      sectors=dict.fromkeys(("front", "front_left", "front_right", "left", "right", "rear"), 2.0))
        return (executor or self.executor).step(response, packet_id=str(latest),
                frames=[dict(captured_at=latest, frame_id=str(latest), motion={"motion_lease_id": "lease"})],
                mission=self.body.mission(), status=status, now=latest + 1)

    def test_full_step_persists_fence_and_sending_before_io(self):
        original = self.body.submit_intent
        def check(value):
            packet = value
            saved = AgentSession.load(self.path)
            self.assertEqual(saved.execution["issued"][packet["decision_id"]]["state"], "sending")
            assert self.executor.fence is not None
            self.assertEqual(saved.execution["executor_fence"], self.executor.fence.to_dict())
            return original(packet)
        self.body.submit_intent = check
        result = self.step()
        self.assertEqual(result["state"], "accepted")
        self.assertEqual(len(self.body.submissions), 1)
        self.assertEqual(self.body.mailbox.take(102.0), result["intent"])

    def test_no_capability_fails_closed_without_claim_or_submit(self):
        mission = self.body.mission()
        mission.pop("executor_fencing_enabled")
        with self.assertRaisesRegex(SessionIdentityError, "unavailable"):
            self.executor.acquire(mission)
        self.assertEqual(self.body.claims, [])
        self.assertEqual(self.body.submissions, [])

    def test_resume_is_explicit_and_stale_local_process_cannot_submit_or_save(self):
        self.stage()
        old_snapshot = self.body.mission()
        resumed = self.make(AgentSession.load(self.path))
        with self.assertRaisesRegex(SessionIdentityError, "explicit"):
            resumed.acquire(self.body.mission())
        resumed.acquire(self.body.mission(), takeover=True)
        with self.assertRaisesRegex(SessionIdentityError, "stale local"):
            self.executor.retry("decision", mission=old_snapshot, now=102.0)
        with self.assertRaises(SessionIdentityError):
            self.executor.save()
        self.assertEqual(self.body.submissions, [])
        assert resumed.fence is not None
        self.assertEqual(AgentSession.load(self.path).execution["executor_fence"], resumed.fence.to_dict())

    def test_separate_process_stale_instance_is_fenced_before_transport(self):
        self.stage()
        snapshot = self.body.mission()
        resumed = self.make(AgentSession.load(self.path))
        resumed.acquire(snapshot, takeover=True)
        ctx = multiprocessing.get_context("fork")
        queue = ctx.Queue()
        process = ctx.Process(target=stale_process, args=(self.executor, snapshot, queue))
        process.start()
        process.join(3)
        if process.is_alive():
            process.terminate()
            process.join()
            self.fail("stale process hung")
        self.assertEqual(process.exitcode, 0)
        self.assertIn("stale local", queue.get(timeout=1))
        queue.close()

    def test_remote_clone_stale_snapshot_cannot_gain_mailbox_admission(self):
        packet = self.stage()
        old_snapshot = self.body.mission()
        other = self.make(AgentSession.load(self.path), self.path.with_name("clone.json"))
        other.acquire(old_snapshot, takeover=True)
        result = self.executor.retry("decision", mission=old_snapshot, now=102.0)
        self.assertEqual(result["state"], "unknown")
        self.assertEqual(self.body.mailbox.snapshot(), [])
        self.assertFalse(self.body.mailbox.authorizes(packet, 102.0))

    def test_lost_claim_reply_retry_deduplicates_without_new_generation(self):
        self.body.lose_claim = True
        with self.assertRaises(BodyDispatchError):
            self.acquire()
        self.assertIsNone(self.executor.fence)
        result = self.acquire()
        self.assertEqual(result.executor_generation, 1)
        self.assertEqual(self.body.claims[0], self.body.claims[1])

    def test_lost_submission_reply_reconciles_without_resending(self):
        self.stage()
        self.body.delivery = "lost_after"
        result = self.executor.retry("decision", mission=self.body.mission(), now=102.0)
        self.assertEqual(result["state"], "unknown")
        self.body.delivery = "normal"
        result = self.executor.retry("decision", mission=self.body.mission(), now=102.0)
        self.assertEqual(result["state"], "accepted")
        self.assertEqual(len(self.body.submissions), 1)

    def test_undelivered_retry_uses_identical_payload(self):
        packet = self.stage()
        self.body.delivery = "lost_before"
        self.executor.retry("decision", mission=self.body.mission(), now=102.0)
        self.body.delivery = "normal"
        self.assertEqual(self.executor.retry("decision", mission=self.body.mission(), now=102.0)["state"], "accepted")
        self.assertEqual(self.body.submissions, [packet, packet])
        self.assertEqual(len(self.body.mailbox.snapshot()), 1)

    def test_unknown_expiry_remains_unconfirmed_not_success(self):
        self.stage()
        self.body.delivery = "lost_before"
        self.executor.retry("decision", mission=self.body.mission(), now=102.0)
        result = self.executor.retry("decision", mission=self.body.mission(), now=110.0)
        self.assertEqual(result["state"], "expired_unconfirmed")
        self.assertFalse(result["ok"])
        self.assertEqual(len(self.body.submissions), 1)

    def test_malformed_success_ack_is_unknown(self):
        self.stage()
        self.body.delivery = "malformed"
        result = self.executor.retry("decision", mission=self.body.mission(), now=102.0)
        self.assertEqual(result["state"], "unknown")

    def test_stale_retry_rejection_cannot_resolve_original_unknown_delivery(self):
        self.stage()
        self.body.delivery = "lost_before"
        self.executor.retry("decision", mission=self.body.mission(), now=102.0)
        snapshot = self.body.mission()
        other = self.make(AgentSession.load(self.path), self.path.with_name("clone.json"))
        other.acquire(snapshot, takeover=True)
        self.body.delivery = "normal"
        result = self.executor.retry("decision", mission=snapshot, now=102.0)
        self.assertEqual(result["state"], "unknown")

    def test_takeover_recovers_missing_checkpoint_records_without_reissue(self):
        packet = self.stage()
        old = AgentSession.from_dict(self.executor.session.to_dict())
        old.execution["issued"] = {}
        old.execution["latest_frame_at"] = 0.0
        self.body.mailbox.submit(packet, 102.0)
        self.body.mailbox.take(102.0)
        resumed = self.make(old, self.path.with_name("recovered.json"))
        resumed.acquire(self.body.mission(), takeover=True)
        record = resumed.session.execution["issued"]["decision"]
        self.assertEqual(record["state"], "unknown")
        self.assertEqual(record["intent"], packet)
        self.assertEqual(resumed.session.execution["latest_frame_at"], 101.0)
        with self.assertRaisesRegex(SessionIdentityError, "previous executor"):
            resumed.retry("decision", mission=self.body.mission(), now=102.0)
        self.assertEqual(self.body.submissions, [])

    def test_terminal_outcome_does_not_regress_or_alias_caller_data(self):
        packet = self.stage()
        completed: dict[str, Any] = dict(ok=True, intent=packet, state="completed")
        self.executor.session.record_execution(completed)
        completed["intent"]["kind"] = "stop"
        stored = self.executor.session.execution["issued"]["decision"]
        self.assertEqual(stored["intent"]["kind"], "continue")
        self.executor.session.record_execution(dict(stored, state="accepted"))
        self.assertEqual(self.executor.session.execution["issued"]["decision"]["state"], "completed")

    def test_shadow_makes_zero_transport_calls(self):
        shadow = ClosedLoopExecutor(self.session, self.body)
        self.assertEqual(self.step(shadow)["state"], "shadow")
        self.assertEqual(self.body.claims, [])
        self.assertEqual(self.body.submissions, [])

    def test_corrupt_state_fails_closed_and_no_request_is_sent(self):
        self.path.write_text("not json")
        with self.assertRaises(json.JSONDecodeError):
            self.acquire()
        self.assertEqual(self.body.claims, [])

    def test_save_failure_prevents_intent_submission(self):
        self.acquire()
        with patch.object(self.executor.session, "save", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.step()
        self.assertEqual(self.body.submissions, [])


if __name__ == "__main__":
    unittest.main()
