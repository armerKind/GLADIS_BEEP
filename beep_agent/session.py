"""Mission-bound persistent semantic session for BEEP."""
from __future__ import annotations

from dataclasses import asdict
import json
import os
from pathlib import Path
import time
import uuid
from typing import Any

from .drives import GoalArbiter, GoalSelection
from .executive import EmbodiedExecutive, EmbodiedGoal, ExecutiveDecision
from .world_model import WorldModel


class SessionIdentityError(ValueError):
    pass


class AgentSession:
    """Keep world, goal and executive state bound to one physical mission."""

    SCHEMA_VERSION = 1

    def __init__(self, *, robot_id: str, mission_id: str | None,
                 fixed_goal: EmbodiedGoal | None = None, world: WorldModel | None = None,
                 session_id: str | None = None, created_at: float | None = None) -> None:
        self.robot_id = str(robot_id)
        self.mission_id = None if mission_id is None else str(mission_id)
        self.session_id = session_id or f"agent-{uuid.uuid4().hex[:12]}"
        self.created_at = time.time() if created_at is None else float(created_at)
        self.updated_at = self.created_at
        self.world = world or WorldModel()
        self.arbiter = GoalArbiter()
        self.fixed_goal = fixed_goal
        self.selection: GoalSelection | None = None
        self.executive: EmbodiedExecutive | None = None

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
               mission_id: str | None = None) -> dict[str, Any]:
        incoming = self.mission_id if mission_id is None else str(mission_id)
        if self.mission_id is not None and incoming != self.mission_id:
            raise SessionIdentityError("perception mission does not match agent session")
        snapshot = self.world.update(perception, observed_at=observed_at, packet_id=packet_id)
        self._select_if_needed()
        self.updated_at = float(observed_at)
        return snapshot

    def decide(self, *, rollout_mode: str = "shadow") -> ExecutiveDecision:
        self._select_if_needed()
        assert self.executive is not None
        return self.executive.decide(rollout_mode=rollout_mode)

    def to_dict(self) -> dict[str, Any]:
        selection = None
        if self.selection is not None:
            selection = {"goal": asdict(self.selection.goal), "drive": self.selection.drive,
                         "rationale": self.selection.rationale}
        return {
            "schema_version": self.SCHEMA_VERSION,
            "session_id": self.session_id,
            "robot_id": self.robot_id,
            "mission_id": self.mission_id,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "fixed_goal": None if self.fixed_goal is None else asdict(self.fixed_goal),
            "selection": selection,
            "inspect_attempts": {} if self.executive is None else dict(self.executive.inspect_attempts),
            "world": self.world.to_dict(),
        }

    def save(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + f".{os.getpid()}.tmp")
        temporary.write_text(json.dumps(self.to_dict(), indent=2, sort_keys=True))
        os.replace(temporary, path)

    @classmethod
    def from_dict(cls, value: dict[str, Any], *, expected_robot_id: str | None = None,
                  expected_mission_id: str | None = None, max_age_s: float | None = None,
                  now: float | None = None) -> "AgentSession":
        if int(value.get("schema_version", 0)) != cls.SCHEMA_VERSION:
            raise SessionIdentityError("unsupported agent session schema")
        robot_id = str(value["robot_id"])
        mission_id = value.get("mission_id")
        if expected_robot_id is not None and robot_id != str(expected_robot_id):
            raise SessionIdentityError("agent session belongs to a different robot")
        if expected_mission_id is not None and mission_id != str(expected_mission_id):
            raise SessionIdentityError("agent session belongs to a different mission")
        current = time.time() if now is None else float(now)
        if max_age_s is not None and current - float(value["updated_at"]) > float(max_age_s):
            raise SessionIdentityError("agent session is stale")
        fixed = value.get("fixed_goal")
        session = cls(robot_id=robot_id, mission_id=mission_id,
                      fixed_goal=None if fixed is None else EmbodiedGoal(**fixed),
                      world=WorldModel.from_dict(value["world"]),
                      session_id=value["session_id"], created_at=float(value["created_at"]))
        selected = value.get("selection")
        if selected is not None:
            session.selection = GoalSelection(EmbodiedGoal(**selected["goal"]), selected["drive"], selected["rationale"])
            session.executive = EmbodiedExecutive(session.world, session.selection.goal)
            session.executive.inspect_attempts = {str(k): int(v) for k, v in value.get("inspect_attempts", {}).items()}
        session.updated_at = float(value["updated_at"])
        return session

    @classmethod
    def load(cls, path: Path, **kwargs) -> "AgentSession":
        return cls.from_dict(json.loads(Path(path).read_text()), **kwargs)