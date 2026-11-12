"""Cross-platform file locking and directory fsync for agent persistence.

On POSIX (the Pi) these use ``fcntl.flock`` and an ``fsync`` of the parent
directory so renames and first appends are durable. On Windows (development
hosts only) neither primitive exists: locking falls back to a no-op, so only
the callers' in-process locks apply, and directory fsync is skipped.
"""
from __future__ import annotations

import os
from pathlib import Path

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
