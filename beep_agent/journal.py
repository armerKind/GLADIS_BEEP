"""Durable append-only event journal; corruption never gets silently repaired."""
from __future__ import annotations

from copy import deepcopy
import fcntl
import json
import os
import threading
from pathlib import Path

from .events import EventEnvelope, EventValidationError


class JournalError(ValueError):
    pass


class EventJournal:
    def __init__(self, path: Path):
        self.path = Path(path)
        self._index: dict[str, EventEnvelope] | None = None
        self._identity: tuple[int, int] | None = None
        self._offset = 0
        self._ctime_ns: int | None = None
        self._cache_lock = threading.RLock()

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
        with self._cache_lock:
            if not self.path.exists():
                if self._index is not None:
                    raise JournalError("journal disappeared after validation")
                return []
            with self.path.open("rb") as stream:
                fcntl.flock(stream, fcntl.LOCK_SH)
                self._sync_locked(stream)
                assert self._index is not None
                return deepcopy(list(self._index.values()))

    def lookup(self, event_ids: tuple[str, ...]) -> dict[str, EventEnvelope]:
        """Read only requested validated records; parse an external append tail once."""
        if (not isinstance(event_ids, tuple) or len(event_ids) > 16
                or any(not isinstance(i, str) or not i.strip() for i in event_ids)):
            raise JournalError("invalid bounded journal lookup")
        with self._cache_lock:
            if not self.path.exists():
                if self._index is not None:
                    raise JournalError("journal disappeared after validation")
                return {}
            with self.path.open("rb") as stream:
                fcntl.flock(stream, fcntl.LOCK_SH)
                self._sync_locked(stream)
                assert self._index is not None
                return deepcopy({i: self._index[i] for i in event_ids if i in self._index})

    def _sync_locked(self, stream) -> None:
        """Validate identity and append-only tail under file and process locks."""
        stat = os.fstat(stream.fileno())
        path_stat = self.path.stat()
        identity = (stat.st_dev, stat.st_ino)
        if identity != (path_stat.st_dev, path_stat.st_ino):
            raise JournalError("journal path changed during access")
        if self._index is None:
            stream.seek(0)
            try:
                previous = self._parse(stream.read().decode("utf-8"))
            except UnicodeDecodeError as exc:
                raise JournalError("invalid journal encoding") from exc
            self._index = {old.event_id: old for old in previous}
        else:
            if identity != self._identity or stat.st_size < self._offset:
                raise JournalError("journal replaced or truncated; preserve evidence")
            if stat.st_size == self._offset and stat.st_ctime_ns != self._ctime_ns:
                raise JournalError("journal overwritten in place; preserve evidence")
            if stat.st_size > self._offset:
                stream.seek(self._offset)
                try:
                    tail = self._parse(stream.read().decode("utf-8"))
                except UnicodeDecodeError as exc:
                    raise JournalError("invalid journal encoding") from exc
                if any(old.event_id in self._index for old in tail):
                    raise JournalError("duplicate event ID in journal tail")
                self._index.update((old.event_id, old) for old in tail)
        self._identity = identity
        self._offset = stat.st_size
        self._ctime_ns = stat.st_ctime_ns

    def append(self, event: EventEnvelope) -> None:
        if not isinstance(event, EventEnvelope):
            raise JournalError("journal accepts only validated EventEnvelope")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # The in-process lock protects cache state; flock protects cooperating
        # processes. Neither silently repairs an invalid external journal.
        with self._cache_lock, self.path.open("a+b") as stream:
            fcntl.flock(stream, fcntl.LOCK_EX)
            self._sync_locked(stream)
            assert self._index is not None
            previous = self._index.get(event.event_id)
            if previous is not None:
                if previous == event:
                    return  # Idempotent replay of the same event.
                raise JournalError("conflicting event ID")
            payload = (json.dumps(event.to_dict(), sort_keys=True, separators=(",", ":"),
                                  allow_nan=False) + "\n").encode("utf-8")
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
            final = os.fstat(stream.fileno())
            if (final.st_dev, final.st_ino) != (self.path.stat().st_dev, self.path.stat().st_ino):
                raise JournalError("journal path changed during append")
            self._index[event.event_id] = deepcopy(event)
            self._offset = final.st_size
            self._ctime_ns = final.st_ctime_ns
            if self._offset == len(payload):
                directory = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
