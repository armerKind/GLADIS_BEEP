"""Durable append-only event journal; corruption never gets silently repaired."""
from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path

from .events import EventEnvelope, EventValidationError


class JournalError(ValueError):
    pass


class EventJournal:
    def __init__(self, path: Path):
        self.path = Path(path)

    @staticmethod
    def _parse(text: str) -> list[EventEnvelope]:
        if text and not text.endswith("\n"):
            raise JournalError("partial journal tail; preserve file for recovery")
        events = []
        seen = {}
        for line in text.splitlines():
            try:
                event = EventEnvelope.from_dict(json.loads(line))
            except (ValueError, TypeError, EventValidationError) as exc:
                raise JournalError("invalid journal event; preserve file for recovery") from exc
            if event.event_id in seen:
                raise JournalError("duplicate event ID in journal")
            seen[event.event_id] = event
            events.append(event)
        return events

    def replay(self) -> list[EventEnvelope]:
        if not self.path.exists():
            return []
        with self.path.open("r", encoding="utf-8") as stream:
            fcntl.flock(stream, fcntl.LOCK_SH)
            return self._parse(stream.read())

    def append(self, event: EventEnvelope) -> None:
        if not isinstance(event, EventEnvelope):
            raise JournalError("journal accepts only validated EventEnvelope")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        new_file = not self.path.exists()
        # Lock the whole read/append transaction; O_APPEND keeps different
        # writers from overwriting each other. fsync is required for evidence.
        with self.path.open("a+", encoding="utf-8") as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            stream.seek(0)
            previous = self._parse(stream.read())
            for old in previous:
                if old.event_id == event.event_id:
                    if old == event:
                        return  # Idempotent replay of the same event.
                    raise JournalError("conflicting event ID")
            payload = json.dumps(event.to_dict(), sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n"
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
            if new_file:
                directory = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
