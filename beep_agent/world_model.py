"""Persistent packet-to-packet world state for an embodied GLADIS instance."""
from __future__ import annotations

import re
from dataclasses import asdict, dataclass, field
from typing import Any

from beep_eyes.schema import validate_perception_response

_WORD = re.compile(r"[a-z0-9]+")
_STOP_WORDS = {"a", "an", "the", "is", "on", "in", "at", "with", "and", "of", "to"}


def _tokens(text: str) -> set[str]:
    return {token for token in _WORD.findall(text.lower()) if token not in _STOP_WORDS}


def _similarity(left: str, right: str) -> float:
    a, b = _tokens(left), _tokens(right)
    return len(a & b) / len(a | b) if a and b else 0.0


@dataclass
class EntityTrack:
    track_id: str
    kind: str
    description: str
    position: str
    confidence: float
    first_seen_at: float
    last_seen_at: float
    seen_count: int = 1
    missed_packets: int = 0
    proximity: str = "unknown"
    visible_features: list[str] = field(default_factory=list)
    engagement: str = "unknown"
    source_ids: list[str] = field(default_factory=list)


class WorldModel:
    """Maintain stable local identities across model inference packets.

    Packet-local IDs are aliases, never trusted as durable identity. Tracks are
    matched first by known alias and then conservatively by kind/description.
    Ambiguous observations become new tracks rather than being silently merged.
    """

    def __init__(self, *, max_missed_packets: int = 6) -> None:
        self.max_missed_packets = max_missed_packets
        self.sequence = 0
        self._next_id = 1
        self.tracks: dict[str, EntityTrack] = {}
        self.aliases: dict[tuple[str, str], str] = {}
        self.hazards: list[dict[str, Any]] = []
        self.attention_track_id: str | None = None
        self.last_scene_summary = ""
        self.last_changes: list[str] = []
        self.last_packet_id: str | None = None

    def _new_track(self, kind: str, item: dict[str, Any], observed_at: float) -> EntityTrack:
        track_id = f"{kind}-{self._next_id:04d}"
        self._next_id += 1
        track = EntityTrack(
            track_id=track_id,
            kind=kind,
            description=item["description"],
            position=item["position"],
            confidence=float(item["confidence"]),
            first_seen_at=observed_at,
            last_seen_at=observed_at,
            proximity=item.get("proximity", "unknown"),
            visible_features=list(item.get("visible_features", [])),
            engagement=item.get("engagement", "unknown"),
            source_ids=[item["id"]],
        )
        self.tracks[track_id] = track
        return track

    def _match(self, kind: str, item: dict[str, Any]) -> EntityTrack | None:
        alias = self.aliases.get((kind, item["id"]))
        if alias in self.tracks:
            return self.tracks[alias]
        candidates = []
        for track in self.tracks.values():
            if track.kind != kind or track.missed_packets > self.max_missed_packets:
                continue
            score = _similarity(track.description, item["description"])
            if track.position == item["position"]:
                score += 0.15
            candidates.append((score, track.track_id, track))
        candidates.sort(reverse=True)
        if not candidates or candidates[0][0] < 0.55:
            return None
        if len(candidates) > 1 and candidates[0][0] - candidates[1][0] < 0.12:
            return None
        return candidates[0][2]

    def update(self, perception: dict[str, Any], *, observed_at: float, packet_id: str) -> dict[str, Any]:
        perception = validate_perception_response(perception)
        if packet_id == self.last_packet_id:
            raise ValueError("duplicate perception packet")
        self.sequence += 1
        observed_tracks: set[str] = set()
        source_to_track: dict[tuple[str, str], str] = {}
        for kind, collection in (("person", perception["scene"]["people"]), ("object", perception["scene"]["objects"])):
            for item in collection:
                track = self._match(kind, item) or self._new_track(kind, item, observed_at)
                track.description = item["description"]
                track.position = item["position"]
                track.confidence = float(item["confidence"])
                track.last_seen_at = observed_at
                track.missed_packets = 0
                track.proximity = item.get("proximity", track.proximity)
                track.visible_features = list(item.get("visible_features", track.visible_features))
                track.engagement = item.get("engagement", track.engagement)
                if item["id"] not in track.source_ids:
                    track.source_ids.append(item["id"])
                if track.first_seen_at != observed_at:
                    track.seen_count += 1
                self.aliases[(kind, item["id"])] = track.track_id
                source_to_track[(kind, item["id"])] = track.track_id
                observed_tracks.add(track.track_id)
        for track in self.tracks.values():
            if track.track_id not in observed_tracks:
                track.missed_packets += 1
        self.tracks = {key: track for key, track in self.tracks.items() if track.missed_packets <= self.max_missed_packets}
        attention = perception["attention"]
        kind = attention["target_type"]
        self.attention_track_id = source_to_track.get((kind, attention["target_id"])) if kind in {"person", "object"} else None
        self.hazards = [dict(item) for item in perception["scene"]["hazards"]]
        self.last_scene_summary = perception["scene"]["summary"]
        self.last_changes = list(perception["scene"]["changes"])
        self.last_packet_id = packet_id
        return self.snapshot()

    def _find_kind(self, kind: str, query: str) -> EntityTrack | None:
        ranked = []
        for track in self.tracks.values():
            if track.kind != kind or track.missed_packets:
                continue
            score = _similarity(query, track.description) + 0.1 * track.confidence
            ranked.append((score, track.track_id, track))
        ranked.sort(reverse=True)
        return ranked[0][2] if ranked and ranked[0][0] >= 0.2 else None

    def find_object(self, query: str) -> EntityTrack | None:
        return self._find_kind("object", query)

    def find_person(self, query: str) -> EntityTrack | None:
        return self._find_kind("person", query)

    def snapshot(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "last_packet_id": self.last_packet_id,
            "scene_summary": self.last_scene_summary,
            "changes": list(self.last_changes),
            "attention_track_id": self.attention_track_id,
            "hazards": [dict(item) for item in self.hazards],
            "tracks": [asdict(self.tracks[key]) for key in sorted(self.tracks)],
        }

    def to_dict(self) -> dict[str, Any]:
        value = self.snapshot()
        value.update({
            "max_missed_packets": self.max_missed_packets,
            "next_id": self._next_id,
            "aliases": [{"kind": kind, "source_id": source_id, "track_id": track_id}
                        for (kind, source_id), track_id in sorted(self.aliases.items())],
        })
        return value

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "WorldModel":
        world = cls(max_missed_packets=int(value.get("max_missed_packets", 6)))
        world.sequence = int(value.get("sequence", 0))
        world._next_id = int(value.get("next_id", 1))
        world.tracks = {item["track_id"]: EntityTrack(**item) for item in value.get("tracks", [])}
        world.aliases = {(item["kind"], item["source_id"]): item["track_id"]
                         for item in value.get("aliases", [])}
        world.hazards = [dict(item) for item in value.get("hazards", [])]
        world.attention_track_id = value.get("attention_track_id")
        world.last_scene_summary = str(value.get("scene_summary", ""))
        world.last_changes = list(value.get("changes", []))
        world.last_packet_id = value.get("last_packet_id")
        if world.tracks:
            highest = max(int(track_id.rsplit("-", 1)[-1]) for track_id in world.tracks)
            world._next_id = max(world._next_id, highest + 1)
        return world
