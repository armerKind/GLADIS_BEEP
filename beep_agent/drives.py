"""Intrinsic drives and personality-shaped autonomous goal selection."""
from __future__ import annotations

from dataclasses import dataclass

from .executive import EmbodiedGoal
from .world_model import WorldModel


@dataclass(frozen=True)
class PersonalityProfile:
    identity: str = "GLADIS"
    curiosity: float = 0.90
    sociability: float = 0.55
    marking_drive: float = 0.45
    caution: float = 0.85
    style: str = "precise, dry, observant"


@dataclass(frozen=True)
class GoalSelection:
    goal: EmbodiedGoal
    drive: str
    rationale: str


class GoalArbiter:
    """Choose what to pursue from current world state, not operator scripting."""

    def __init__(self, personality: PersonalityProfile | None = None) -> None:
        self.personality = personality or PersonalityProfile()

    def select(self, world: WorldModel) -> GoalSelection:
        if not world.sequence:
            return GoalSelection(EmbodiedGoal("perceive", "scene"), "embodiment", "No current world state exists.")
        people = [track for track in world.tracks.values() if track.kind == "person" and not track.missed_packets]
        engaged = [track for track in people if track.engagement in {"looking", "speaking", "approaching"} and track.confidence >= 0.7]
        if engaged:
            target = max(engaged, key=lambda track: (track.confidence, track.last_seen_at))
            return GoalSelection(
                EmbodiedGoal("acknowledge_person", target.description),
                "social",
                "A person is actively engaging with this body.",
            )
        objects = [track for track in world.tracks.values() if track.kind == "object" and not track.missed_packets and track.confidence >= 0.7]
        if objects:
            target = max(objects, key=lambda track: (1.0 / track.seen_count, track.confidence, track.last_seen_at))
            has_visible_corner = any(feature in {"left_outward_corner", "right_outward_corner"} for feature in target.visible_features)
            marking_requested = self.personality.marking_drive >= 0.4 and target.proximity == "near" and has_visible_corner
            return GoalSelection(
                EmbodiedGoal("investigate_and_mark" if marking_requested else "investigate", target.description, marking_requested),
                "curiosity",
                "A confident object remains insufficiently examined.",
            )
        return GoalSelection(
            EmbodiedGoal("explore", "novel safe space"),
            "curiosity",
            "No social or object target currently deserves attention.",
        )
