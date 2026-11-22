"""Cross-platform file locking and directory fsync for agent persistence.

On POSIX (the Pi) these use ``fcntl.flock`` and an ``fsync`` of the parent
directory so renames and first appends are durable. On Windows (development
hosts only) neither primitive exists: locking falls back to a no-op, so only
the callers' in-process locks apply, and directory fsync is skipped.
"""
from __future__ import annotations

import os
import json
from pathlib import Path
import tempfile

try:
    import fcntl
except ImportError:  # Windows
    fcntl = None

_O_DIRECTORY = getattr(os, "O_DIRECTORY", None)


def lock_shared(stream) -> None:
    """Take a shared advisory lock on an open file, released when it closes."""
    if fcntl is not None:
        fcntl.flock(stream, fcntl.LOCK_SH)


def lock_exclusive(stream) -> None:
    """Take an exclusive advisory lock on an open file, released when it closes."""
    if fcntl is not None:
        fcntl.flock(stream, fcntl.LOCK_EX)


def fsync_directory(path: Path) -> None:
    """Persist directory entries (renames, new files) where the OS supports it."""
    if os.name == "nt" or _O_DIRECTORY is None:
        return
    directory = os.open(path, os.O_RDONLY | _O_DIRECTORY)
    try:
        os.fsync(directory)
    finally:
        os.close(directory)


def atomic_write_json(path: Path, value: object) -> None:
    """Write one complete JSON snapshot; never leave a partial target file."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=path.name + ".", suffix=".tmp",
                                         delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        fsync_directory(path.parent)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
