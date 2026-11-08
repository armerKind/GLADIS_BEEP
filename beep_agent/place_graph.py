"""Conservative topological graph of observed safe viewpoints (not routes)."""
from __future__ import annotations

from math import hypot, isfinite
from typing import Iterable

from .events import EventEnvelope
from .pose_history import ProjectedPose


class PlaceGraph:
    def __init__(self, *, overlap_threshold: float = 0.5, min_separation_m: float = 0.4):
        if not (0 < overlap_threshold < 1) or min_separation_m <= 0:
            raise ValueError("invalid place-graph thresholds")
        self.overlap_threshold = overlap_threshold
        self.min_separation_m = min_separation_m
        self.nodes: dict[str, dict] = {}
        self.edges: list[dict] = []
        self._active_id: str | None = None
        self._identity: tuple[str, int, int] | None = None
        self._pending: tuple[str, int] | None = None
        self._next_id = 1

    def invalidate_revision(self, revision: int) -> None:
        for node in self.nodes.values():
            node["metric_valid"] = False
        for edge in self.edges:
            edge["valid"] = False
        self._active_id = None
        self._pending = None
        if self._identity is not None:
            self._identity = (self._identity[0], self._identity[1], revision)

    def observe(self, event: EventEnvelope, free_cells: Iterable[tuple[int, int]], *,
                projected_pose: ProjectedPose | None,
                clearance_m: float, visibility_radius_m: float,
                footprint_radius_m: float,
                bottleneck: bool = False, branch: bool = False,
                edge_clearance_verified: bool = False) -> str | None:
        if ((event.event_kind, event.source) not in {
                ("frame_acquired", "camera"), ("lidar_observation", "lidar")}
                or not event.spatial_ready or projected_pose is None
                or (projected_pose.event_id, projected_pose.boot_id, projected_pose.map_epoch,
                    projected_pose.map_revision, projected_pose.map_snapshot_id)
                != (event.event_id, event.boot_id, event.map_epoch,
                    event.map_revision, event.map_snapshot_id)):
            return None
        if not all(isfinite(v) for v in (projected_pose.x_m, projected_pose.y_m)):
            raise ValueError("verified projected map pose is required")
        if (not all(isfinite(v) for v in (clearance_m, visibility_radius_m, footprint_radius_m))
                or clearance_m <= 0 or visibility_radius_m <= 0 or footprint_radius_m <= 0):
            raise ValueError("invalid free-space geometry")
        if clearance_m < footprint_radius_m:
            return None
        cells = frozenset(free_cells)
        if not cells or len(cells) > 512 or any(not isinstance(c, tuple) or len(c) != 2
                            or not all(isinstance(v, int) and not isinstance(v, bool) for v in c)
                            for c in cells):
            raise ValueError("free cells must be nonempty aligned integer map cells")
        assert event.map_revision is not None
        identity = (event.boot_id, event.map_epoch, event.map_revision)
        if self._identity != identity:
            self.invalidate_revision(event.map_revision)
            self._identity = identity
        # This API cannot silently mistake a local odom pose for a map pose.
        # The caller provides a map-projected pose under the exact revision.
        x, y = projected_pose.x_m, projected_pose.y_m
        active = self.nodes.get(self._active_id) if self._active_id is not None else None
        if active is None:
            for prior in self.nodes.values():
                if (prior["metric_valid"] or prior["boot_id"] != event.boot_id
                        or prior["map_epoch"] != event.map_epoch
                        or hypot(x - prior["x_m"], y - prior["y_m"]) >= self.min_separation_m):
                    continue
                overlap = len(cells & prior["free_cells"]) / len(cells | prior["free_cells"])
                if overlap < self.overlap_threshold:
                    continue
                prior.update(map_revision=event.map_revision,
                             map_snapshot_id=event.map_snapshot_id, x_m=x, y_m=y,
                             free_cells=cells, clearance_m=clearance_m, metric_valid=True)
                prior["evidence_ids"].append(event.event_id)
                self._active_id = prior["node_id"]
                self._pending = None
                return prior["node_id"]
        if active is not None:
            distance = hypot(x - active["x_m"], y - active["y_m"])
            overlap = len(cells & active["free_cells"]) / len(cells | active["free_cells"])
            low_overlap = overlap < self.overlap_threshold and distance >= self.min_separation_m
            corridor_retrace = distance >= max(1.5, 2 * visibility_radius_m)
            reason = ("bottleneck" if bottleneck and distance >= self.min_separation_m
                      else "branch" if branch and distance >= self.min_separation_m
                      else "new_visibility" if low_overlap else "corridor_retrace" if corridor_retrace
                      else None)
            if reason is None:
                self._pending = None
                return None
            if reason != "corridor_retrace":
                signature = (reason, active["node_id"])
                count = self._pending[1] + 1 if self._pending and self._pending[0] == str(signature) else 1
                self._pending = (str(signature), count)
                if count < 2:
                    return None
        else:
            reason = "first_viewpoint"
        node_id = f"place-{self._next_id:04d}"
        self._next_id += 1
        self.nodes[node_id] = {
            "node_id": node_id, "boot_id": event.boot_id, "map_epoch": event.map_epoch,
            "map_revision": event.map_revision, "map_snapshot_id": event.map_snapshot_id,
            "x_m": x, "y_m": y, "free_cells": cells, "clearance_m": clearance_m,
            "metric_valid": True, "reason": reason, "evidence_ids": [event.event_id],
        }
        if active is not None and active["metric_valid"] and active["map_revision"] == event.map_revision:
            self.edges.append({"from": active["node_id"], "to": node_id,
                               "valid": edge_clearance_verified, "evidence_id": event.event_id,
                               "map_revision": event.map_revision})
        self._active_id = node_id
        self._pending = None
        return node_id

    def to_dict(self) -> dict:
        return {"next_id": self._next_id, "identity": self._identity,
                "overlap_threshold": self.overlap_threshold,
                "min_separation_m": self.min_separation_m,
                "active_id": self._active_id, "edges": [dict(edge) for edge in self.edges],
                "nodes": [{**node, "free_cells": sorted(map(list, node["free_cells"]))}
                          for node in self.nodes.values()]}

    @classmethod
    def from_dict(cls, value: dict, *, verified_identity: tuple[str, int, int] | None = None) -> "PlaceGraph":
        graph = cls(overlap_threshold=float(value["overlap_threshold"]),
                    min_separation_m=float(value["min_separation_m"]))
        graph._next_id = int(value["next_id"])
        graph._identity = None if value["identity"] is None else tuple(value["identity"])
        graph._active_id = value["active_id"]
        graph.nodes = {n["node_id"]: {**n, "free_cells": frozenset(map(tuple, n["free_cells"]))}
                       for n in value["nodes"]}
        graph.edges = [dict(e) for e in value["edges"]]
        # Pending evidence never survives a restart or map revalidation.
        if verified_identity is None or verified_identity != graph._identity:
            graph.invalidate_revision(verified_identity[2] if verified_identity else 0)
        return graph
