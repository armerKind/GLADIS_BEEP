"""Offline atomic AgentSession + GoalCoordinator checkpoint; no bridge I/O.

One replaceable JSON document prevents a torn pair. EventJournal is a separate
append-only file and is NOT atomically committed with this checkpoint.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile

from .coordinator import CoordinatorError, GoalCoordinator
from .fsutil import fsync_directory, lock_exclusive
from .session import AgentSession, SessionIdentityError


class CheckpointError(ValueError):
    pass


class CombinedCheckpoint:
    """Single-document persistence with fsync and generation-fenced writers."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self.generation = 0

    def _read(self, *, expected_robot_id=None, expected_boot_id=None) -> tuple[int, AgentSession, GoalCoordinator]:
        try:
            raw = json.loads(self.path.read_text(encoding='utf-8'))
            if (not isinstance(raw, dict) or set(raw) !=
                    {'schema_version', 'generation', 'session', 'coordinator'}
                    or type(raw['schema_version']) is not int or raw['schema_version'] != 1
                    or type(raw['generation']) is not int or raw['generation'] < 1):
                raise CheckpointError('invalid combined checkpoint schema')
            session = AgentSession.from_dict(raw['session'],
                expected_robot_id=expected_robot_id, expected_boot_id=expected_boot_id)
            coordinator = GoalCoordinator.from_dict(session, raw['coordinator'])
            return raw['generation'], session, coordinator
        except (OSError, ValueError, TypeError, KeyError, AttributeError,
                SessionIdentityError, CoordinatorError) as exc:
            raise CheckpointError('invalid, missing, or mismatched checkpoint') from exc

    def _lock(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        stream = (self.path.parent / (self.path.name + '.lock')).open('a+b')
        lock_exclusive(stream)
        return stream

    def load(self, *, expected_robot_id: str | None = None,
             expected_boot_id: str | None = None) -> tuple[AgentSession, GoalCoordinator]:
        with self._lock():
            generation, session, coordinator = self._read(
                expected_robot_id=expected_robot_id, expected_boot_id=expected_boot_id)
            self.generation = generation
            return session, coordinator

    def save(self, session: AgentSession, coordinator: GoalCoordinator) -> int:
        if (not isinstance(session, AgentSession) or not isinstance(coordinator, GoalCoordinator)
                or coordinator.session is not session or session.boot_id is None):
            raise CheckpointError('checkpoint needs one boot-bound session and its coordinator')
        # The coordinator always locks in this order; do not snapshot two
        # independently mutable objects in two separate file replacements.
        with coordinator._lock, session._strategy_lock:
            pair = dict(schema_version=1, generation=self.generation + 1,
                        session=session._to_dict_locked(), coordinator=coordinator.to_dict())
        try:
            payload = json.dumps(pair, sort_keys=True, allow_nan=False).encode('utf-8')
            parsed = json.loads(payload)
            check_session = AgentSession.from_dict(parsed['session'],
                expected_robot_id=session.robot_id, expected_boot_id=session.boot_id)
            GoalCoordinator.from_dict(check_session, parsed['coordinator'])
        except (ValueError, TypeError, KeyError, SessionIdentityError,
                CoordinatorError) as exc:
            raise CheckpointError('inconsistent combined checkpoint snapshot') from exc
        with self._lock():
            if self.path.exists():
                disk_generation, disk_session, _ = self._read()
                if ((disk_session.robot_id, disk_session.session_id, disk_session.boot_id) !=
                        (session.robot_id, session.session_id, session.boot_id)):
                    raise CheckpointError('checkpoint identity cannot be replaced')
            else:
                disk_generation = 0
            if disk_generation != self.generation:
                raise CheckpointError('stale checkpoint writer generation')
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(mode='wb', dir=self.path.parent,
                        prefix=self.path.name + '.', suffix='.tmp', delete=False) as stream:
                    temporary = Path(stream.name)
                    stream.write(payload)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, self.path)
                fsync_directory(self.path.parent)
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
            self.generation = pair['generation']
            return self.generation
