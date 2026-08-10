from __future__ import annotations

import json
from pathlib import Path
import time

from .core import Frame


_IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".webp"}


class DirectoryFrameSource:
    """Deterministic offline source using optional `<image>.json` sidecars."""

    def __init__(self, root: str | Path, source_name="directory"):
        self.root = Path(root)
        self.source_name = source_name
        self._paths = sorted(path for path in self.root.iterdir() if path.suffix.lower() in _IMAGE_SUFFIXES)
        self._index = 0

    def read(self) -> Frame | None:
        if self._index >= len(self._paths):
            return None
        path = self._paths[self._index]
        self._index += 1
        sidecar = path.with_suffix(path.suffix + ".json")
        metadata = {}
        if sidecar.exists():
            metadata = json.loads(sidecar.read_text(encoding="utf-8"))
            if not isinstance(metadata, dict):
                raise ValueError(f"sidecar must contain an object: {sidecar}")
        return Frame(
            source=self.source_name,
            sequence=self._index,
            captured_at=float(metadata.pop("captured_at", path.stat().st_mtime or time.time())),
            payload=path.read_bytes(),
            suffix=path.suffix.lower(),
            metadata=metadata,
        )
