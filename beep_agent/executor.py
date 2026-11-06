"""Mission-owned, shadow-default semantic feedback loop.

Only bounded course changes and terminal stop/yield are supported. Never starts
or replaces a mission. Persist before sending, reconcile after uncertain I/O;
retrying a stored wire intent is safe at the onboard mailbox, not a new decision.
"""
from __future__ import annotations

from pathlib import Path
import time
import uuid

from beep_bridge.semantic_intent import MAX_EVIDENCE_AGE_S, MAX_INTENTS, sensor_gate, validate_intent
from beep_eyes.course import recommend_course
from .body import BodyDispatchError
from .session import SessionIdentityError


class ClosedLoopExecutor:
    def __init__(self, session, body, *, mode="shadow", armed=False, state_path: Path | None = None):
        if mode not in {"shadow", "supervised"}:
            raise ValueError("executor mode must be shadow or supervised")
        if mode == "supervised" and (not armed or state_path is None):
            raise ValueError("supervised executor requires explicit arm and durable state_path")
        self.session, self.body = session, body
        self.mode, self.state_path = mode, state_path

    def save(self):
        if self.state_path is not None:
            self.session.save(self.state_path)

    def reconcile(self, mission):
        if not isinstance(mission, dict) or mission.get("id") != self.session.mission_id:
            raise SessionIdentityError("executor mission identity changed")
        saved_lease = self.session.execution["lease_id"]
        if saved_lease is not None and mission.get("lease_id") != saved_lease:
            raise SessionIdentityError("executor lease identity changed")
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
                    and time.time() >= intent.get("expires_at", float("inf"))):
                self.session.record_execution({"ok": False, "state": "expired_unconfirmed",
                                               "reason": "delivery_outcome_unconfirmed_after_expiry",
                                               "intent": intent})
        self.save()

    def step(self, response, *, packet_id, frames, mission, status, now=None):
        now = time.time() if now is None else now
        self.reconcile(mission)
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
        try:
            outcome = self.body.submit_intent(intent)
            if (not isinstance(outcome, dict) or outcome.get("state") not in
                    {"accepted", "applied", "completed", "rejected"}):
                raise BodyDispatchError("invalid intent acknowledgment")
            if outcome.get("intent", intent) != intent:
                raise BodyDispatchError("intent acknowledgment identity mismatch")
            outcome = dict(outcome, intent=intent)
        except BodyDispatchError as exc:
            outcome = {"ok": False, "intent": intent, "state": "unknown", "reason": str(exc)}
        session.record_execution(outcome)
        self.save()
        result.update(state=outcome["state"], reason=outcome.get("reason"),
                      dispatch_performed=True, outcome=outcome)
        return result
