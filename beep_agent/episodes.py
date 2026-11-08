"""Bounded, provenance-linked summaries; never a substitute for raw journal."""
from __future__ import annotations

from copy import deepcopy

from .journal import EventJournal


class EpisodeStore:
    def __init__(self, *, max_episodes: int = 64):
        if max_episodes < 1:
            raise ValueError("max_episodes must be positive")
        self.max_episodes = max_episodes
        self.episodes: list[dict] = []

    def add(self, summary: str, *, evidence_ids: list[str], journal: EventJournal,
            outcome: str) -> dict:
        if not isinstance(summary, str) or not summary.strip() or len(summary) > 1000:
            raise ValueError("invalid episode summary")
        if outcome not in {"observed", "inferred", "disproved"}:
            raise ValueError("episode must distinguish observation, inference, disproof")
        available = {event.event_id for event in journal.replay()}
        if not evidence_ids or any(not isinstance(e, str) or e not in available for e in evidence_ids):
            raise ValueError("episode evidence missing from journal")
        episode = {"summary": summary, "evidence_ids": list(dict.fromkeys(evidence_ids)),
                   "outcome": outcome}
        self.episodes.append(episode)
        if len(self.episodes) > self.max_episodes:
            self.episodes = self.episodes[-self.max_episodes:]
        return deepcopy(episode)

    def to_dict(self) -> dict:
        return {"schema_version": 1, "max_episodes": self.max_episodes,
                "episodes": deepcopy(self.episodes)}

    @classmethod
    def from_dict(cls, value: dict) -> "EpisodeStore":
        if value.get("schema_version") != 1:
            raise ValueError("unknown episode schema")
        store = cls(max_episodes=int(value["max_episodes"]))
        episodes = deepcopy(value["episodes"])
        if not isinstance(episodes, list) or len(episodes) > store.max_episodes:
            raise ValueError("invalid episode count")
        for item in episodes:
            if (not isinstance(item, dict) or set(item) != {"summary", "evidence_ids", "outcome"}
                    or item["outcome"] not in {"observed", "inferred", "disproved"}
                    or not isinstance(item["evidence_ids"], list)):
                raise ValueError("invalid episode")
        store.episodes = episodes
        return store
