"""Persistent semantic session with optional exact boot/lease binding for BEEP."""
from __future__ import annotations

from dataclasses import asdict
import copy
import json
import os
from pathlib import Path
import time
import uuid
import tempfile
from typing import Any

from .drives import GoalArbiter, GoalSelection
from .executive import EmbodiedExecutive, EmbodiedGoal, ExecutiveDecision
from .world_model import WorldModel
from .skills import skill


class SessionIdentityError(ValueError):
    pass


class AgentSession:
    """Keep world, goal and executive state across explicitly released leases."""

    SCHEMA_VERSION = 2

    def __init__(self, *, robot_id: str, mission_id: str | None,
                 fixed_goal: EmbodiedGoal | None = None, world: WorldModel | None = None,
                 session_id: str | None = None, created_at: float | None = None,
                 boot_id: str | None = None) -> None:
        self.robot_id = str(robot_id)
        self.boot_id = None if boot_id is None else str(boot_id)
        self.mission_id = None if mission_id is None else str(mission_id)
        self.session_id = session_id or f"agent-{uuid.uuid4().hex[:12]}"
        self.created_at = time.time() if created_at is None else float(created_at)
        self.updated_at = self.created_at
        self.world = world or WorldModel()
        self.arbiter = GoalArbiter()
        self.fixed_goal = fixed_goal
        self.selection: GoalSelection | None = None
        self.executive: EmbodiedExecutive | None = None
        self.execution: dict[str, Any] = {"lease_id": None, "latest_frame_at": 0.0,
                                         "issued": {}, "last_outcome": None,
                                         "executor_fence": None,
                                         "reobserve_through_sequence": -1,
                                         "lease_history": []}

    def bind_motion(self, *, mission_id: str, lease_id: str, boot_id: str) -> None:
        """Bind only after exact release of any prior owner; never infer stop."""
        if not all(isinstance(x, str) and x.strip() for x in (mission_id, lease_id, boot_id)):
            raise SessionIdentityError("motion binding needs nonempty IDs")
        if self.boot_id is not None and boot_id != self.boot_id:
            raise SessionIdentityError("motion binding belongs to a different boot")
        existing = self.execution["lease_id"]
        if existing is not None:
            if (self.mission_id, existing) == (mission_id, lease_id):
                return
            raise SessionIdentityError("release exact current lease before rebinding")
        self.boot_id = boot_id
        self.mission_id = mission_id
        self.execution["lease_id"] = lease_id
        self.execution["lease_history"].append({"mission_id": mission_id,
                                                 "lease_id": lease_id, "state": "bound"})

    def release_motion(self, *, mission_id: str | None, lease_id: str | None) -> None:
        if (not mission_id or not lease_id or self.execution["lease_id"] is None
                or (self.mission_id, self.execution["lease_id"]) != (mission_id, lease_id)):
            raise SessionIdentityError("cannot release a different motion owner")
        self.execution["lease_id"] = None
        self.mission_id = None
        self.execution["lease_history"].append({"mission_id": mission_id,
                                                 "lease_id": lease_id, "state": "released"})

    def _adopt(self, candidate: GoalSelection, *, preserve_executive: bool = False) -> None:
        self.selection = candidate
        if preserve_executive and self.executive is not None:
            self.executive.goal = candidate.goal
        else:
            self.executive = EmbodiedExecutive(self.world, candidate.goal)

    def _select_if_needed(self) -> None:
        candidate = (GoalSelection(self.fixed_goal, "operator", "Mission-fixed semantic goal.")
                     if self.fixed_goal is not None else self.arbiter.select(self.world))
        if self.selection is None:
            self._adopt(candidate)
            return
        if self.fixed_goal is not None:
            return
        current = self.selection
        # Social engagement may interrupt autonomous curiosity, but releases the
        # executive when the person is no longer actively engaged.
        if candidate.drive == "social" and current.drive != "social":
            self._adopt(candidate)
            return
        if current.drive == "social" and candidate.drive != "social":
            self._adopt(candidate)
            return
        # Exploration yields when a concrete target appears.
        if current.goal.name == "explore" and candidate.goal.name != "explore":
            self._adopt(candidate)
            return
        if current.drive == "curiosity" and current.goal.name != "explore":
            current_target = self.world.find_object(current.goal.target_query)
            candidate_target = self.world.find_object(candidate.goal.target_query)
            if current_target is None:
                self._adopt(candidate)
                return
            if (candidate_target is not None and candidate_target.track_id == current_target.track_id
                    and candidate.goal.marking_requested and not current.goal.marking_requested):
                # Escalate the same target without discarding inspection history.
                self._adopt(candidate, preserve_executive=True)

    def update(self, perception: dict[str, Any], *, observed_at: float, packet_id: str,
               mission_id: str | None = None, lease_id: str | None = None) -> dict[str, Any]:
        active_lease = self.execution["lease_id"]
        if self.boot_id is not None and active_lease is None:
            if mission_id is not None or lease_id is not None:
                raise SessionIdentityError("unowned perception cannot claim a motion lease")
        else:
            incoming = self.mission_id if mission_id is None else str(mission_id)
            if self.mission_id is not None and incoming != self.mission_id:
                raise SessionIdentityError("perception mission does not match agent session")
            if self.boot_id is not None and lease_id != active_lease:
                raise SessionIdentityError("perception does not belong to exact active lease")
        snapshot = self.world.update(perception, observed_at=observed_at, packet_id=packet_id)
        self._select_if_needed()
        self.updated_at = float(observed_at)
        return snapshot

    def decide(self, *, rollout_mode: str = "shadow") -> ExecutiveDecision:
        self._select_if_needed()
        assert self.executive is not None
        if self.world.sequence <= self.execution["reobserve_through_sequence"]:
            return ExecutiveDecision(self.executive.goal.name, None,
                                     skill("observe", reason="Re-perceive after execution feedback."),
                                     "execution_feedback_requires_reobservation", False)
        return self.executive.decide(rollout_mode=rollout_mode)

    def record_execution(self, outcome: dict[str, Any]) -> None:
        intent = outcome.get("intent") or {}
        if intent.get("mission_id") != self.mission_id or intent.get("session_id") != self.session_id:
            raise SessionIdentityError("execution outcome belongs to a different session or mission")
        if self.boot_id is not None and (self.execution["lease_id"] is None or
                                         intent.get("lease_id") != self.execution["lease_id"]):
            raise SessionIdentityError("execution outcome belongs to a different lease")
        key = intent["decision_id"]
        previous = self.execution["issued"].get(key)
        if previous is not None and previous.get("intent") != intent:
            raise SessionIdentityError("execution decision identity conflict")
        state = outcome.get("state")
        if state not in {"sending", "unknown", "accepted", "applied", "completed", "rejected", "expired_unconfirmed"}:
            raise SessionIdentityError("invalid execution outcome state")
        if previous is not None:
            old_state = previous.get("state")
            if old_state in {"completed", "rejected"}:
                return
            if old_state == "applied" and state in {"sending", "accepted"}:
                return
            if old_state == "expired_unconfirmed" and state in {"sending", "unknown", "accepted", "applied"}:
                return
        if previous == outcome:
            return
        self.execution["issued"][key] = copy.deepcopy(outcome)
        self.execution["last_outcome"] = copy.deepcopy(outcome)
        if outcome.get("state") in {"rejected", "unknown", "expired_unconfirmed"} or intent.get("kind") in {"yield", "stop"}:
            self.execution["reobserve_through_sequence"] = self.world.sequence + 1

    def to_dict(self) -> dict[str, Any]:
        selection = None
        if self.selection is not None:
            selection = {"goal": asdict(self.selection.goal), "drive": self.selection.drive,
                         "rationale": self.selection.rationale}
        return {
            "schema_version": self.SCHEMA_VERSION,
            "session_id": self.session_id,
            "robot_id": self.robot_id,
            "boot_id": self.boot_id,
            "mission_id": self.mission_id,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "fixed_goal": None if self.fixed_goal is None else asdict(self.fixed_goal),
            "selection": selection,
            "inspect_attempts": {} if self.executive is None else dict(self.executive.inspect_attempts),
            "world": self.world.to_dict(),
            "execution": self.execution,
        }

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", dir=path.parent,
                                             prefix=path.name + ".", suffix=".tmp", delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(self.to_dict(), stream, indent=2, sort_keys=True, allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
            directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    @classmethod
    def from_dict(cls, value: dict[str, Any], *, expected_robot_id: str | None = None,
                  expected_mission_id: str | None = None, max_age_s: float | None = None,
                  now: float | None = None,
                  expected_boot_id: str | None = None) -> "AgentSession":
        if int(value.get("schema_version", 0)) not in {1, cls.SCHEMA_VERSION}:
            raise SessionIdentityError("unsupported agent session schema")
        robot_id = str(value["robot_id"])
        boot_id = value.get("boot_id")
        mission_id = value.get("mission_id")
        if expected_robot_id is not None and robot_id != str(expected_robot_id):
            raise SessionIdentityError("agent session belongs to a different robot")
        if expected_boot_id is not None and boot_id != str(expected_boot_id):
            raise SessionIdentityError("agent session belongs to a different boot")
        if expected_mission_id is not None and mission_id != str(expected_mission_id):
            raise SessionIdentityError("agent session belongs to a different mission")
        current = time.time() if now is None else float(now)
        if max_age_s is not None and current - float(value["updated_at"]) > float(max_age_s):
            raise SessionIdentityError("agent session is stale")
        fixed = value.get("fixed_goal")
        session = cls(robot_id=robot_id, mission_id=mission_id,
                      fixed_goal=None if fixed is None else EmbodiedGoal(**fixed),
                      world=WorldModel.from_dict(value["world"]),
                      session_id=value["session_id"], created_at=float(value["created_at"]),
                      boot_id=boot_id)
        selected = value.get("selection")
        if selected is not None:
            session.selection = GoalSelection(EmbodiedGoal(**selected["goal"]), selected["drive"], selected["rationale"])
            session.executive = EmbodiedExecutive(session.world, session.selection.goal)
            session.executive.inspect_attempts = {str(k): int(v) for k, v in value.get("inspect_attempts", {}).items()}
        session.updated_at = float(value["updated_at"])
        if "execution" in value:
            session.execution.update(copy.deepcopy(value["execution"]))
        return session

    @classmethod
    def load(cls, path: Path, **kwargs) -> "AgentSession":
        return cls.from_dict(json.loads(Path(path).read_text()), **kwargs)