"""Mission-owned, shadow-default semantic feedback loop.

Only bounded course changes and terminal stop/yield are supported. Never starts
or replaces a mission. Persist before sending, reconcile after uncertain I/O;
retrying a stored wire intent is safe at the onboard mailbox, not a new decision.
"""
from __future__ import annotations

from pathlib import Path
from contextlib import contextmanager
from functools import wraps
import fcntl
import json
import threading
import time
import uuid
from typing import Any

from beep_bridge.semantic_intent import ExecutorFence, MAX_EVIDENCE_AGE_S, MAX_INTENTS, sensor_gate, validate_intent
from beep_eyes.course import recommend_course
from .body import BodyDispatchError
from .session import AgentSession, SessionIdentityError


def serialized(method):
    @wraps(method)
    def locked(self, *args, **kwargs):
        with self._serialized(allow_claim=method.__name__ == "acquire"):
            return method(self, *args, **kwargs)
    return locked


class ClosedLoopExecutor:
    def __init__(self, session, body, *, mode="shadow", armed=False, state_path: Path | None = None):
        if mode not in {"shadow", "supervised"}:
            raise ValueError("executor mode must be shadow or supervised")
        if mode == "supervised" and (not armed or state_path is None):
            raise ValueError("supervised executor requires explicit arm and durable state_path")
        self.session, self.body = session, body
        self.mode, self.state_path = mode, state_path
        self.instance_id = "executor-" + uuid.uuid4().hex
        self.fence = None  # NEVER inherit a process identity from saved state.
        self._claim_request = None
        self._mutex = threading.RLock()
        self._lock_depth = 0

    @contextmanager
    def _serialized(self, *, allow_claim=False):
        with self._mutex:
            if self._lock_depth or self.state_path is None:
                yield
                return
            path = Path(self.state_path).resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.with_name(path.name + ".executor.lock").open("a") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX)
                try:
                    if self.fence is not None and not path.exists():
                        raise SessionIdentityError("executor checkpoint disappeared")
                    if self.fence is not None and path.exists():
                        saved = json.loads(path.read_text())["execution"].get("executor_fence")
                        if saved != self.fence.to_dict():
                            raise SessionIdentityError("stale local executor fence")
                    if self.fence is None and not allow_claim and path.exists():
                        if json.loads(path.read_text())["execution"].get("executor_fence") is not None:
                            raise SessionIdentityError("explicit safe resume takeover required")
                    if self.mode == "shadow" and path.exists():
                        if json.loads(path.read_text())["execution"].get("executor_fence") is not None:
                            raise SessionIdentityError("shadow executor cannot overwrite fenced state")
                    self._lock_depth += 1
                    try:
                        yield
                    finally:
                        self._lock_depth -= 1
                finally:
                    fcntl.flock(lock, fcntl.LOCK_UN)

    @serialized
    def save(self):
        if self.state_path is not None:
            if self.mode == "shadow" and self.session.execution.get("executor_fence") is not None:
                raise SessionIdentityError("shadow executor cannot overwrite fenced state")
            self.session.save(self.state_path)

    def _identity(self, mission):
        if not isinstance(mission, dict) or mission.get("id") != self.session.mission_id:
            raise SessionIdentityError("executor mission identity changed")
        saved_lease = self.session.execution["lease_id"]
        if saved_lease is not None and mission.get("lease_id") != saved_lease:
            raise SessionIdentityError("executor lease identity changed")

    @serialized
    def acquire(self, mission, *, takeover=False):
        """Explicit safe resume: acquire(mission, takeover=True).

        Transport interface: body.claim_executor(request), or the existing
        adapter's _json('/mission/executor/claim', request). No legacy fallback.
        Claim response: {ok, fence, intent_outcomes}; mission snapshot advertises
        executor_fencing_enabled=True and executor_fence (None before claim).
        """
        if self.mode != "supervised":
            raise SessionIdentityError("shadow executor cannot claim authority")
        self._identity(mission)
        if mission.get("executor_fencing_enabled") is not True:
            raise SessionIdentityError("onboard_executor_fencing_unavailable")
        if mission.get("state") != "running":
            raise SessionIdentityError("executor mission is not running")
        remote = mission.get("executor_fence")
        if remote is not None:
            remote = ExecutorFence.from_dict(remote)
            if (remote.mission_id, remote.lease_id, remote.session_id) != (
                    self.session.mission_id, mission.get("lease_id"), self.session.session_id):
                raise SessionIdentityError("executor owner identity changed")
        if self.fence is not None:
            if remote != self.fence:
                raise SessionIdentityError("stale remote executor fence")
            return self.fence
        if self._claim_request is None:
            if self.state_path is not None and Path(self.state_path).exists():
                restored = AgentSession.load(
                    self.state_path, expected_robot_id=self.session.robot_id,
                    expected_mission_id=self.session.mission_id)
                if restored.session_id != self.session.session_id:
                    raise SessionIdentityError("local executor session changed")
                self.session = restored
                self._identity(mission)
                disk = self.session.execution.get("executor_fence")
                if disk is not None and disk["executor_generation"] > (0 if remote is None else remote.executor_generation):
                    raise SessionIdentityError("stale mission snapshot for takeover")
            if (remote is not None or self.session.execution["issued"]
                    or self.session.execution.get("executor_fence")) and not takeover:
                raise SessionIdentityError("explicit safe resume takeover required")
            self._claim_request = {"mission_id": self.session.mission_id,
                                   "lease_id": mission.get("lease_id"),
                                   "session_id": self.session.session_id,
                                   "executor_instance_id": self.instance_id,
                                   "expected_generation": 0 if remote is None else remote.executor_generation,
                                   "takeover": takeover}
        request = self._claim_request
        # Save the attempted claim before I/O. A lost reply is NOT ownership.
        self.session.execution["pending_claim"] = dict(request)
        self.save()
        claim = getattr(self.body, "claim_executor", None)
        response = (claim(dict(request)) if claim is not None else
                    self.body._json("/mission/executor/claim", dict(request)))
        if not isinstance(response, dict) or response.get("ok") is not True:
            raise SessionIdentityError("executor claim rejected: " + str(response))
        fence = ExecutorFence.from_dict(response.get("fence"))
        expected = {k: request[k] for k in ExecutorFence.__annotations__ if k != "executor_generation"}
        expected["executor_generation"] = request["expected_generation"] + 1
        if fence.to_dict() != expected or not isinstance(response.get("intent_outcomes"), list):
            raise SessionIdentityError("invalid executor claim acknowledgment")
        # Recover records omitted by a stale local checkpoint from the atomic
        # claim snapshot. Never re-envelope an old intent with the new epoch.
        for outcome in response["intent_outcomes"]:
            intent = outcome.get("intent") or {}
            if intent.get("lease_id") != fence.lease_id:
                raise SessionIdentityError("recovered execution lease mismatch")
            self.session.record_execution(outcome)
            self.session.execution["latest_frame_at"] = max(
                self.session.execution["latest_frame_at"], intent["latest_frame_at"])
        self.session.execution["lease_id"] = fence.lease_id
        self.session.execution["executor_fence"] = fence.to_dict()
        self.session.execution.pop("pending_claim", None)
        self.fence = fence
        self.save()
        return fence

    @serialized
    def reconcile(self, mission, *, now=None):
        self._identity(mission)
        if self.fence is not None and mission.get("executor_fence") != self.fence.to_dict():
            raise SessionIdentityError("stale remote executor fence")
        now = time.time() if now is None else now
        outcomes = mission.get("intent_outcomes", [])
        for outcome in outcomes:
            intent = outcome.get("intent") or {}
            if intent.get("session_id") == self.session.session_id:
                old = self.session.execution["issued"].get(intent.get("decision_id"))
                if old is None or old.get("intent") != intent:
                    raise SessionIdentityError("unrecognized execution outcome")
                self.session.record_execution(outcome)
        # Once evidence has expired an absent uncertain submission cannot be
        # accepted later; release the durable ledger without reissuing it.
        onboard_ids = {(o.get("intent") or {}).get("decision_id") for o in outcomes}
        for key, record in list(self.session.execution["issued"].items()):
            intent = record.get("intent") or {}
            if (record.get("state") in {"sending", "unknown"} and key not in onboard_ids
                    and now >= intent.get("expires_at", float("inf"))):
                self.session.record_execution({"ok": False, "state": "expired_unconfirmed",
                                               "reason": "delivery_outcome_unconfirmed_after_expiry",
                                               "intent": intent})
        self.save()

    @serialized
    def step(self, response, *, packet_id, frames, mission: dict[str, Any], status, now=None):
        now = time.time() if now is None else now
        if self.mode == "supervised" and self.fence is None:
            self.acquire(mission)
            assert self.fence is not None
            # Claim is a newer atomic snapshot than the caller's mission view.
            mission = dict(mission, executor_fence=self.fence.to_dict(),
                           intent_outcomes=list(self.session.execution["issued"].values()))
        self.reconcile(mission, now=now)
        session = self.session
        # All frames must belong to the same lease, not just the last status.
        observed_at = min(frame["captured_at"] for frame in frames)
        latest_at = max(frame["captured_at"] for frame in frames)
        session.update(response, observed_at=latest_at, packet_id=packet_id, mission_id=mission["id"])
        decision = session.decide(rollout_mode=self.mode)
        course = recommend_course(response, status, mission_valid=mission.get("state") == "running")
        result = {"decision": decision.to_dict(), "course": course, "dispatch_performed": False,
                  "state": "shadow", "reason": "rollout_not_physically_armed"}
        name = decision.skill_call.name
        safety_course = course["recommendation"] in {
            "stop_and_reassess", "hold_and_reobserve", "replan_around_dynamic_hazard",
            "look_left", "look_right"}
        if name not in {"explore", "observe", "stop"} and not safety_course:
            result.update(state="rejected", reason="unsupported_semantic_skill:" + name)
            self.save()
            return result
        if decision.rationale == "execution_feedback_requires_reobservation" and not safety_course:
            result.update(state="rejected", reason=decision.rationale)
            self.save()
            return result
        # A request to look is not authority for a translating arc. Stationary
        # orientation has no adapter yet, so yield instead of inventing one.
        kind = {"forward_candidate": "continue", "look_left": "yield",
                "look_right": "yield", "stop_and_reassess": "stop",
                "hold_and_reobserve": "yield", "replan_around_dynamic_hazard": "yield"}.get(course["recommendation"])
        if name == "stop":
            kind = "stop"
        if kind is None:
            result.update(state="rejected", reason="unsupported_course:" + course["recommendation"])
            self.save()
            return result
        lease_id = mission.get("lease_id")
        intent = {"mission_id": session.mission_id, "lease_id": lease_id,
                  "session_id": session.session_id, "decision_id": "decision-" + uuid.uuid4().hex,
                  "frame_id": frames[-1]["frame_id"], "observed_at": observed_at,
                  "latest_frame_at": latest_at, "expires_at": observed_at + MAX_EVIDENCE_AGE_S,
                  "kind": kind}
        if self.fence is not None:
            intent.update(self.fence.to_dict())
        try:
            validate_intent(intent, now)
            if latest_at <= session.execution["latest_frame_at"]:
                raise ValueError("duplicate_or_superseded_window")
            if (mission.get("state") != "running" or status.get("motion_lease_id") != lease_id
                    or any((frame.get("motion") or {}).get("motion_lease_id") != lease_id for frame in frames)):
                raise ValueError("mission_or_frame_lease_mismatch")
            if kind not in {"yield", "stop"} and not sensor_gate(status):
                raise ValueError("local_sensors_invalid")
            if len(session.execution["issued"]) >= MAX_INTENTS:
                raise ValueError("intent_capacity_exhausted")
            if kind not in {"stop", "yield"} and any(record.get("state") in {"sending", "unknown", "accepted", "applied"}
                   for record in session.execution["issued"].values()):
                raise ValueError("execution_outcome_pending")
        except ValueError as exc:
            result.update(state="rejected", reason=str(exc))
            self.save()
            return result
        result["intent"] = intent
        if self.mode == "shadow":
            self.save()
            return result
        if mission.get("semantic_intents_enabled") is not True:
            result.update(state="rejected", reason="onboard_not_armed")
            self.save()
            return result
        session.execution["lease_id"] = lease_id
        session.execution["latest_frame_at"] = latest_at
        session.execution["issued"][intent["decision_id"]] = {"intent": intent, "state": "sending"}
        self.save()  # A crash now must not create a second command on resume.
        outcome = self._submit(intent)
        result.update(state=outcome["state"], reason=outcome.get("reason"),
                      dispatch_performed=True, outcome=outcome)
        return result

    def _submit(self, intent, *, retry=False):
        if self.fence is None or not self.fence.matches(intent):
            raise SessionIdentityError("stale executor cannot submit intent")
        try:
            outcome = self.body.submit_intent(dict(intent))
            if (not isinstance(outcome, dict) or outcome.get("state") not in
                    {"accepted", "applied", "completed", "rejected", "unknown"}):
                raise BodyDispatchError("invalid intent acknowledgment")
            if outcome.get("intent", intent) != intent:
                raise BodyDispatchError("intent acknowledgment identity mismatch")
            previous = self.session.execution["issued"][intent["decision_id"]]
            if outcome.get("intent") is None and (outcome["state"] != "rejected"
                    or retry or previous["state"] == "unknown"):
                raise BodyDispatchError("acknowledgment does not establish delivery outcome")
            if outcome["state"] in {"accepted", "applied", "completed"} and outcome.get("ok") is not True:
                raise BodyDispatchError("inconsistent intent acknowledgment")
            outcome = dict(outcome, intent=intent)
        except (BodyDispatchError, OSError, TimeoutError) as exc:
            outcome = {"ok": False, "intent": intent, "state": "unknown", "reason": str(exc)}
        self.session.record_execution(outcome)
        self.save()
        return self.session.execution["issued"][intent["decision_id"]]

    @serialized
    def retry(self, decision_id, *, mission, now=None):
        """Retry only the exact durable wire payload, never a new decision ID."""
        if self.mode != "supervised" or self.fence is None:
            raise SessionIdentityError("executor has no claimed authority")
        now = time.time() if now is None else now
        self.reconcile(mission, now=now)
        record = self.session.execution["issued"][decision_id]
        if record["state"] not in {"sending", "unknown"}:
            return record
        intent = record["intent"]
        if not self.fence.matches(intent):
            raise SessionIdentityError("cannot retry an intent from a previous executor")
        validate_intent(intent, now)
        return self._submit(intent, retry=True)
