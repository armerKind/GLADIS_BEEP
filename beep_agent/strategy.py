"""Phase B shadow strategic owner: typed goals without motor authority."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from math import isfinite
from typing import Any, Iterable

from .drives import GoalSelection
from .executive import EmbodiedGoal
from .session import AgentSession

LIFETIME_CHARTER = (
    "GLADIS inhabits BEEP to learn the world through honest perception, "
    "remain physically intact, investigate novelty, engage consenting people, "
    "and maintain continuity of memory. No object or demonstration is a lifetime goal. "
    "GLADIS proposes goals; the local deterministic controller alone owns motion."
)


class StrategyError(ValueError):
    pass


def _id(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip()) and len(value) <= 256


@dataclass(frozen=True)
class GoalProposal:
    schema_version: int
    proposal_id: str
    session_id: str
    boot_id: str
    map_epoch: int
    map_revision: int | None
    base_goal_revision: int
    observed_world_sequence: int
    goal_name: str
    target_track_id: str | None
    issued_at_robot: float
    expires_at_robot: float
    source_event_ids: tuple[str, ...]
    reason: str

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "GoalProposal":
        if not isinstance(raw, dict) or set(raw) != set(cls.__dataclass_fields__):
            raise StrategyError("strategic proposal fields differ from v1")
        if type(raw["schema_version"]) is not int or raw["schema_version"] != 1:
            raise StrategyError("unsupported strategic proposal schema")
        if not all(_id(raw[k]) for k in ("proposal_id", "session_id", "boot_id")):
            raise StrategyError("invalid strategic identity")
        if (raw["target_track_id"] is not None and not _id(raw["target_track_id"])):
            raise StrategyError("invalid target track")
        for k in ("map_epoch", "base_goal_revision", "observed_world_sequence"):
            if type(raw[k]) is not int or raw[k] < 0:
                raise StrategyError("invalid strategic version")
        revision = raw["map_revision"]
        if revision is not None and (type(revision) is not int or revision < 0):
            raise StrategyError("invalid map revision")
        if (not isinstance(raw["goal_name"], str) or raw["goal_name"] not in
                {"explore", "investigate", "investigate_and_mark", "acknowledge_person", "observe", "rest"}):
            raise StrategyError("unsupported strategic goal")
        for k in ("issued_at_robot", "expires_at_robot"):
            if (isinstance(raw[k], bool) or not isinstance(raw[k], (int, float))
                    or not isfinite(raw[k]) or raw[k] < 0):
                raise StrategyError("invalid strategic clock")
        if not raw["issued_at_robot"] < raw["expires_at_robot"] <= raw["issued_at_robot"] + 10:
            raise StrategyError("strategic proposal must have bounded lifetime")
        evidence = raw["source_event_ids"]
        if (not isinstance(evidence, list) or len(evidence) > 16
                or not all(_id(x) for x in evidence) or len(set(evidence)) != len(evidence)):
            raise StrategyError("invalid strategic evidence IDs")
        if not isinstance(raw["reason"], str) or not 8 <= len(raw["reason"]) <= 512:
            raise StrategyError("strategic rationale is missing or excessive")
        return cls(**{**raw, "issued_at_robot": float(raw["issued_at_robot"]),
                      "expires_at_robot": float(raw["expires_at_robot"]),
                      "source_event_ids": tuple(evidence)})

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["source_event_ids"] = list(self.source_event_ids)
        return result


class StrategicOwner:
    """Admit one GLADIS goal revision from current world evidence; shadow only."""

    def __init__(self, session: AgentSession):
        self.session = session

    def build_context(self, *, episodes: Iterable[dict] = (), places: Iterable[dict] = ()) -> dict:
        world = self.session.world.snapshot()
        world["tracks"] = sorted(world["tracks"], key=lambda t: t["last_seen_at"],
                                 reverse=True)[:16]
        world["hazards"] = world["hazards"][:8]
        world["changes"] = world["changes"][:8]
        world["scene_summary"] = world["scene_summary"][:512]
        return {"charter": LIFETIME_CHARTER, "session_id": self.session.session_id,
                "boot_id": self.session.boot_id,
                "goal_revision": self.session.strategy["goal_revision"],
                "current_goal": None if self.session.selection is None
                                else asdict(self.session.selection.goal),
                "world": world, "episodes": list(episodes)[-8:], "places": list(places)[-8:]}

    def submit(self, proposal: GoalProposal, *, now_robot: float, map_epoch: int,
               map_revision: int | None, known_event_ids: frozenset[str] = frozenset()) -> GoalSelection:
        with self.session._strategy_lock:
            return self._submit_locked(proposal, now_robot=now_robot, map_epoch=map_epoch,
                                       map_revision=map_revision, known_event_ids=known_event_ids)

    def _submit_locked(self, proposal: GoalProposal, *, now_robot: float, map_epoch: int,
                       map_revision: int | None, known_event_ids: frozenset[str]) -> GoalSelection:
        if not isinstance(proposal, GoalProposal):
            raise StrategyError("expected validated strategic proposal")
        proposal = GoalProposal.from_dict(proposal.to_dict())
        if not isinstance(now_robot, (int, float)) or not isfinite(now_robot):
            raise StrategyError("invalid robot time")
        if self.session.fixed_goal is not None:
            raise StrategyError("operator-fixed goal cannot be replaced by strategy")
        if (proposal.session_id, proposal.boot_id) != (self.session.session_id, self.session.boot_id):
            raise StrategyError("strategic owner belongs to another session or boot")
        receipts = self.session.strategy["proposal_ids"]
        prior = receipts.get(proposal.proposal_id)
        if prior is not None:
            if prior["proposal"] != proposal.to_dict():
                raise StrategyError("conflicting strategic proposal ID")
            if prior["applied_revision"] != self.session.strategy["goal_revision"]:
                raise StrategyError("old strategic proposal has been superseded")
            assert self.session.selection is not None
            return self.session.selection
        if ((proposal.map_epoch, proposal.map_revision) != (map_epoch, map_revision)
                or proposal.observed_world_sequence != self.session.world.sequence
                or proposal.base_goal_revision != self.session.strategy["goal_revision"]
                or not proposal.issued_at_robot <= now_robot <= proposal.expires_at_robot):
            raise StrategyError("stale strategic evidence or goal revision")
        if not set(proposal.source_event_ids) <= known_event_ids:
            raise StrategyError("unverified tactical evidence")
        if proposal.goal_name in {"investigate", "investigate_and_mark", "acknowledge_person"}:
            target = self.session.world.tracks.get(proposal.target_track_id or "")
            expected_kind = "person" if proposal.goal_name == "acknowledge_person" else "object"
            required_confidence = .7 if expected_kind == "person" else .65
            if (target is None or target.kind != expected_kind or target.missed_packets
                    or target.confidence < required_confidence):
                raise StrategyError("strategic target lacks current corroboration")
            if proposal.goal_name == "investigate_and_mark" and (
                    target.proximity != "near" or not set(target.visible_features) &
                    {"left_outward_corner", "right_outward_corner"}):
                raise StrategyError("marking needs current close outward corner")
            query = target.description
        elif proposal.target_track_id is not None:
            raise StrategyError("untargeted strategic goal cannot claim a track")
        else:
            query = "novel safe space" if proposal.goal_name == "explore" else ""
        drive = ("social" if proposal.goal_name == "acknowledge_person" else
                 "embodiment" if proposal.goal_name in {"observe", "rest"} else "curiosity")
        goal = EmbodiedGoal(proposal.goal_name, query,
                            marking_requested=proposal.goal_name == "investigate_and_mark")
        selection = GoalSelection(goal, drive, proposal.reason)
        self.session._adopt(selection)
        self.session.strategy["goal_revision"] += 1
        receipts[proposal.proposal_id] = {"proposal": proposal.to_dict(),
                                          "applied_revision": self.session.strategy["goal_revision"]}
        if len(receipts) > 128:
            del receipts[next(iter(receipts))]
        return selection
