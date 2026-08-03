"""Self-directed semantic executive between perception and hardware policy."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from .skills import SkillCall, skill
from .world_model import EntityTrack, WorldModel


@dataclass(frozen=True)
class EmbodiedGoal:
    name: str
    target_query: str
    marking_requested: bool = False


@dataclass(frozen=True)
class ExecutiveDecision:
    goal: str
    target_id: str | None
    skill_call: SkillCall
    rationale: str
    requires_physical_authority: bool

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["skill_call"] = self.skill_call.to_dict()
        return value


class EmbodiedExecutive:
    """Choose one semantic next action from persistent world state.

    The executive owns goals and sequencing, but not motors. Its output must pass
    a separate sensor/lease policy and bounded executor before physical effect.
    """

    def __init__(self, world: WorldModel, goal: EmbodiedGoal) -> None:
        self.world = world
        self.goal = goal
        self.inspect_attempts: dict[str, int] = {}

    def _decision(self, target: EntityTrack | None, call: SkillCall, rationale: str) -> ExecutiveDecision:
        physical = call.name in {"orient", "approach_target", "inspect_target", "mark_target", "explore", "gesture"}
        return ExecutiveDecision(self.goal.name, target.track_id if target else None, call, rationale, physical)

    def decide(self, *, rollout_mode: str = "shadow") -> ExecutiveDecision:
        if rollout_mode not in {"shadow", "stationary", "supervised"}:
            raise ValueError("rollout_mode must be shadow, stationary, or supervised")
        if not self.world.sequence:
            return self._decision(None, skill("observe", reason="No world evidence exists yet."), "acquire_initial_world_state")

        central_living_hazards = {
            hazard["type"] for hazard in self.world.hazards
            if hazard["position"] == "center" and float(hazard["confidence"]) >= 0.5 and hazard["type"] in {"person", "animal", "vehicle"}
        }
        if central_living_hazards:
            return self._decision(None, skill("observe", reason="A living or moving hazard occupies the intended path."), "yield_to_living_hazard")

        if self.goal.name == "acknowledge_person":
            person = self.world.find_person(self.goal.target_query)
            if person is None or person.confidence < 0.7:
                return self._decision(None, skill("observe", reason="The social target is no longer reliable."), "social_target_uncertain")
            return self._decision(
                person,
                skill("speak", target_id=person.track_id, text="I see you. I am occupied, but not impolite.", reason="Acknowledge active engagement while stationary."),
                "social_acknowledgement",
            )
        if self.goal.name == "explore":
            return self._decision(None, skill("explore", duration_s=8, reason="Seek novel safe observations, then stop and re-perceive."), "bounded_curiosity")

        target = self.world.find_object(self.goal.target_query)
        if target is None:
            return self._decision(None, skill("observe", reason=f"Search for an object matching {self.goal.target_query}."), "target_not_found")
        if target.confidence < 0.65:
            return self._decision(target, skill("observe", target_id=target.track_id, reason="Target confidence is insufficient for physical intent."), "target_uncertain")
        if target.position in {"left", "right"}:
            return self._decision(
                target,
                skill("orient", target_id=target.track_id, direction=target.position, degrees=10, reason="Center the tracked target before approach."),
                "center_target",
            )
        if target.position == "unknown":
            return self._decision(target, skill("observe", target_id=target.track_id, reason="Target bearing is unknown."), "bearing_unknown")

        if target.proximity in {"far", "mid"}:
            return self._decision(
                target,
                skill("approach_target", target_id=target.track_id, max_step_m=0.15, stop_distance_m=0.40, reason="Approach one bounded segment, then stop and re-perceive."),
                "bounded_approach",
            )
        if target.proximity == "unknown":
            return self._decision(target, skill("observe", target_id=target.track_id, reason="Estimate target proximity before movement."), "proximity_unknown")

        corners = [feature for feature in target.visible_features if feature in {"left_outward_corner", "right_outward_corner"}]
        if not corners:
            attempt = self.inspect_attempts.get(target.track_id, 0)
            strategy = "left_view" if attempt % 2 == 0 else "right_view"
            self.inspect_attempts[target.track_id] = attempt + 1
            return self._decision(
                target,
                skill("inspect_target", target_id=target.track_id, strategy=strategy, max_turn_deg=12, reason="Reveal a genuine outward corner before marking."),
                "inspect_geometry",
            )
        if self.goal.marking_requested:
            feature = corners[0]
            return self._decision(
                target,
                skill("mark_target", target_id=target.track_id, feature=feature, dry_run=rollout_mode != "supervised", reason="A near target has a visible outward corner suitable for marking."),
                "mark_visible_corner",
            )
        return self._decision(target, skill("observe", target_id=target.track_id, reason="Target examination is complete."), "goal_observed")
