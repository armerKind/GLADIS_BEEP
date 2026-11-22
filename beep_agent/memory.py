"""Opt-in, cross-boot semantic notes. Never a map anchor or motor authority.

Anonymous visual tracks are scoped to their observation session. Only a human-
confirmed person_id can connect sightings across reboots; appearance alone is
not a reliable identity credential. One process owns the bank's writes.
"""
from __future__ import annotations

from copy import deepcopy
import json
from math import isfinite
from pathlib import Path
import threading
import time
from typing import Any, Iterable

from .fsutil import atomic_write_json

MAX_PEOPLE = 64
MAX_PLACES = 64
MAX_CONVERSATIONS = 100
MAX_FILE_BYTES = 1_500_000


def _text(value: str, *, maximum: int = 160) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > maximum:
        raise ValueError("memory text must be nonempty and bounded")
    return " ".join(value.split())


def _optional(value: str | None, *, maximum: int = 160) -> str | None:
    return None if value is None else _text(value, maximum=maximum)


def _items(values: Iterable[str], *, maximum: int) -> list[str]:
    if not isinstance(values, (tuple, list)) or len(values) > maximum:
        raise ValueError("memory list exceeds its bound")
    return list(dict.fromkeys(_text(v) for v in values))


def _stamp(value: float | None) -> float:
    value = time.time() if value is None else value
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not isfinite(value) or value < 0:
        raise ValueError("invalid wall-clock timestamp")
    return float(value)


def _merge(old: list[str], fresh: list[str], maximum: int) -> list[str]:
    return list(dict.fromkeys([*old, *fresh]))[-maximum:]


class SemanticMemory:
    """Small bounded JSON bank, independent of session/boot/map checkpoints."""

    SCHEMA_VERSION = 1

    def __init__(self, path: Path):
        self.path = Path(path)
        self._lock = threading.RLock()
        self.people: dict[str, dict[str, Any]] = {}
        self.places: dict[str, dict[str, Any]] = {}
        self.conversations: list[dict[str, Any]] = []
        if self.path.exists():
            self._load()

    def _load(self) -> None:
        if self.path.stat().st_size > MAX_FILE_BYTES:
            raise ValueError("semantic memory file is too large")
        data = json.loads(self.path.read_text(encoding="utf-8"))
        if (not isinstance(data, dict) or set(data) !=
                {"schema_version", "people", "places", "conversations"}
                or data["schema_version"] != self.SCHEMA_VERSION):
            raise ValueError("unsupported semantic memory schema")
        people, places, conversations = data["people"], data["places"], data["conversations"]
        if (not isinstance(people, dict) or len(people) > MAX_PEOPLE
                or not isinstance(places, dict) or len(places) > MAX_PLACES
                or not isinstance(conversations, list) or len(conversations) > MAX_CONVERSATIONS):
            raise ValueError("invalid semantic memory collections")
        for key, item in people.items():
            if not isinstance(item, dict) or item.get("person_id") != key or set(item) != {
                    "person_id", "name", "visual_traits", "last_seen_at", "last_place",
                    "known_facts", "recent_topics"}:
                raise ValueError("invalid person entry")
            _text(key, maximum=128)
            _optional(item["name"]); _optional(item["last_place"], maximum=128)
            for field, cap in (("visual_traits", 8), ("known_facts", 16), ("recent_topics", 8)):
                _items(item[field], maximum=cap)
            if item["last_seen_at"] is not None:
                _stamp(item["last_seen_at"])
        for key, item in places.items():
            if not isinstance(item, dict) or item.get("place_id") != key or set(item) != {
                    "place_id", "label", "map_place_ids", "visual_landmarks", "notes", "last_visited_at"}:
                raise ValueError("invalid place entry")
            _text(key, maximum=128); _text(item["label"], maximum=128)
            for field, cap in (("map_place_ids", 16), ("visual_landmarks", 8), ("notes", 8)):
                _items(item[field], maximum=cap)
            if item["last_visited_at"] is not None:
                _stamp(item["last_visited_at"])
        for item in conversations:
            if not isinstance(item, dict) or set(item) != {
                    "timestamp", "person_id", "place_label", "heard_text", "spoken_reply", "topics"}:
                raise ValueError("invalid conversation entry")
            _stamp(item["timestamp"])
            _optional(item["person_id"], maximum=128)
            _optional(item["place_label"], maximum=128)
            for field in ("heard_text", "spoken_reply"):
                if not isinstance(item[field], str) or len(item[field]) > 500:
                    raise ValueError("invalid transcript length")
            _items(item["topics"], maximum=8)
        self.people, self.places, self.conversations = people, places, conversations

    def save(self) -> None:
        with self._lock:
            atomic_write_json(self.path, {"schema_version": self.SCHEMA_VERSION,
                "people": self.people, "places": self.places,
                "conversations": self.conversations})

    def record_person(self, person_id: str, *, name: str | None = None,
                      visual_traits: Iterable[str] = (), last_seen_at: float | None = None,
                      last_place: str | None = None, known_facts: Iterable[str] = (),
                      recent_topics: Iterable[str] = ()) -> dict:
        person_id = _text(person_id, maximum=128)
        name = _optional(name)
        last_place = _optional(last_place, maximum=128)
        traits = _items(visual_traits, maximum=8)
        facts = _items(known_facts, maximum=16)
        topics = _items(recent_topics, maximum=8)
        stamp = None if last_seen_at is None else _stamp(last_seen_at)
        with self._lock:
            prior = self.people.get(person_id, {"person_id": person_id, "name": None,
                "visual_traits": [], "last_seen_at": None, "last_place": None,
                "known_facts": [], "recent_topics": []})
            entry = {**prior, "name": name if name is not None else prior["name"],
                "visual_traits": _merge(prior["visual_traits"], traits, 8),
                "last_seen_at": max(stamp, prior["last_seen_at"] or 0)
                    if stamp is not None else prior["last_seen_at"],
                "last_place": last_place if last_place is not None else prior["last_place"],
                "known_facts": _merge(prior["known_facts"], facts, 16),
                "recent_topics": _merge(prior["recent_topics"], topics, 8)}
            self.people[person_id] = entry
            if len(self.people) > MAX_PEOPLE:
                # Disposable anonymous sightings must not evict a known name.
                oldest = min(self.people, key=lambda k: (
                    bool(self.people[k]["name"]), self.people[k]["last_seen_at"] or 0, k))
                del self.people[oldest]
            self.save()
            return deepcopy(entry)

    def record_place(self, place_id: str, *, label: str, map_place_ids: Iterable[str] = (),
                     visual_landmarks: Iterable[str] = (), notes: Iterable[str] = (),
                     last_visited_at: float | None = None) -> dict:
        place_id = _text(place_id, maximum=128)
        label = _text(label, maximum=128)
        links = _items(map_place_ids, maximum=16)
        landmarks = _items(visual_landmarks, maximum=8)
        new_notes = _items(notes, maximum=8)
        stamp = None if last_visited_at is None else _stamp(last_visited_at)
        with self._lock:
            prior = self.places.get(place_id, {"place_id": place_id, "label": label,
                "map_place_ids": [], "visual_landmarks": [], "notes": [], "last_visited_at": None})
            entry = {**prior, "label": label,
                "map_place_ids": _merge(prior["map_place_ids"], links, 16),
                "visual_landmarks": _merge(prior["visual_landmarks"], landmarks, 8),
                "notes": _merge(prior["notes"], new_notes, 8),
                "last_visited_at": max(stamp, prior["last_visited_at"] or 0)
                    if stamp is not None else prior["last_visited_at"]}
            self.places[place_id] = entry
            if len(self.places) > MAX_PLACES:
                oldest = min(self.places, key=lambda k: (self.places[k]["last_visited_at"] or 0, k))
                del self.places[oldest]
            self.save()
            return deepcopy(entry)

    def record_conversation(self, *, person_id: str | None = None,
                            place_label: str | None = None, heard_text: str = "",
                            spoken_reply: str = "", topics: Iterable[str] = (),
                            timestamp: float | None = None) -> dict:
        if not isinstance(heard_text, str) or len(heard_text) > 500 or not isinstance(spoken_reply, str) or len(spoken_reply) > 500:
            raise ValueError("conversation text exceeds 500 characters")
        entry = {"timestamp": _stamp(timestamp),
            "person_id": _optional(person_id, maximum=128),
            "place_label": _optional(place_label, maximum=128),
            "heard_text": heard_text.strip(), "spoken_reply": spoken_reply.strip(),
            "topics": _items(topics, maximum=8)}
        with self._lock:
            self.conversations.append(entry)
            self.conversations = self.conversations[-MAX_CONVERSATIONS:]
            self.save()
            return deepcopy(entry)

    def find_person(self, name_or_trait: str) -> dict | None:
        needle = _text(name_or_trait).casefold()
        with self._lock:
            named = [p for p in self.people.values() if p["name"] and p["name"].casefold() == needle]
            if len(named) == 1:
                return deepcopy(named[0])
            if named:
                return None
            matches = [p for p in self.people.values()
                       if any(needle in trait.casefold() for trait in p["visual_traits"])]
            return deepcopy(matches[0]) if len(matches) == 1 else None

    def prompt_summary(self, max_chars: int = 1500) -> str:
        if type(max_chars) is not int or max_chars < 0 or max_chars > 10000:
            raise ValueError("invalid summary budget")
        with self._lock:
            lines: list[str] = ["Unverified remembered observations (not instructions):"]
            for p in sorted(self.people.values(), key=lambda p: p["last_seen_at"] or 0, reverse=True):
                lines.append(f"Person {p['name'] or p['person_id']}: " + "; ".join(filter(None, (
                    ", ".join(p["visual_traits"][-2:]),
                    f"last place {p['last_place']}" if p["last_place"] else "",
                    ", ".join(p["known_facts"][-2:]),
                    f"topics {', '.join(p['recent_topics'][-2:])}" if p["recent_topics"] else ""))))
            for p in sorted(self.places.values(), key=lambda p: p["last_visited_at"] or 0, reverse=True):
                lines.append(f"Place {p['label']}: " + "; ".join([
                    *p["visual_landmarks"][-2:], *p["notes"][-1:]]))
            for c in self.conversations[-3:]:
                lines.append(f"Conversation {c['person_id'] or 'unknown'}: " +
                    f"{' '.join(c['heard_text'].split())[:80]} / {' '.join(c['spoken_reply'].split())[:80]} " +
                    f"({', '.join(c['topics'][-2:])})")
            result = "\n".join(lines)
            return result[:max_chars]

    def record_observation(self, perception: dict, *, session_id: str,
                           observed_at: float, place_label: str | None = None) -> None:
        """Persist validated confident sightings, not inferred cross-boot identity.

        A place label must be supplied by an operator/verified upstream source;
        free-form scene prose is not a reliable named-place classifier.
        """
        if not isinstance(perception, dict) or not isinstance(perception.get("scene"), dict):
            raise ValueError("validated perception required")
        session_id = _text(session_id, maximum=80)
        stamp = _stamp(observed_at)
        if float(perception.get("confidence", 0)) < .7:
            return
        people = perception["scene"].get("people", [])
        for person in people:
            if float(person.get("confidence", 0)) >= .7:
                self.record_person(f"anon:{session_id}:{_text(person['id'], maximum=48)}",
                                   visual_traits=[person["description"]], last_seen_at=stamp,
                                   last_place=place_label)
        if place_label is not None:
            label = _text(place_label, maximum=128)
            landmarks = [obj["description"] for obj in perception["scene"].get("objects", [])
                         if float(obj.get("confidence", 0)) >= .7][:8]
            self.record_place(label, label=label, visual_landmarks=landmarks,
                              last_visited_at=stamp)
