"""Conservative topological graph of observed safe viewpoints (not routes)."""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, floor, hypot, isfinite, sin
from typing import Iterable

from .events import EventEnvelope
from .pose_history import ProjectedPose


CANONICAL_CELL_M = 0.1


@dataclass(frozen=True)
class GridGeometry:
    """Origin and resolution of raw occupancy indices in the map frame."""
    origin_x_m: float
    origin_y_m: float
    resolution_m: float
    frame_id: str
    origin_yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        if (self.frame_id != "map" or not all(isinstance(v, (int, float))
                and not isinstance(v, bool) and isfinite(v) for v in
                (self.origin_x_m, self.origin_y_m, self.resolution_m, self.origin_yaw_rad))
                or self.resolution_m <= 0):
            raise ValueError("grid needs finite map-frame origin and positive resolution")

    def canonical_cells(self, cells: Iterable[tuple[int, int]]) -> frozenset[tuple[int, int]]:
        raw = frozenset(cells)
        if not raw or len(raw) > 512 or any(not isinstance(c, tuple) or len(c) != 2
                or not all(isinstance(v, int) and not isinstance(v, bool) for v in c)
                for c in raw):
            raise ValueError("free cells must be nonempty bounded integer grid indices")
        c, s = cos(self.origin_yaw_rad), sin(self.origin_yaw_rad)
        aligned = set()
        for gx, gy in raw:
            dx, dy = (gx + 0.5) * self.resolution_m, (gy + 0.5) * self.resolution_m
            x = self.origin_x_m + c * dx - s * dy
            y = self.origin_y_m + s * dx + c * dy
            if not isfinite(x) or not isfinite(y):
                raise ValueError("grid indices overflow metric map frame")
            aligned.add((floor(x / CANONICAL_CELL_M), floor(y / CANONICAL_CELL_M)))
        return frozenset(aligned)


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
                grid_geometry: GridGeometry | None = None,
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
        if not isinstance(grid_geometry, GridGeometry):
            raise ValueError("grid provenance required for spatial place observation")
        cells = grid_geometry.canonical_cells(free_cells)
        assert event.map_revision is not None
        identity = (event.boot_id, event.map_epoch, event.map_revision)
        if self._identity != identity:
            self.invalidate_revision(event.map_revision)
            self._identity = identity
        # This API cannot silently mistake a local odom pose for a map pose.
        # The caller provides a map-projected pose under the exact revision.
        x, y = projected_pose.x_m, projected_pose.y_m
        active = self.nodes.get(self._active_id) if self._active_id is not None else None
        # A revisited room or the second revalidated node must be matched
        # even while another node is active. Never compare raw grid indices.
        matches = []
        for prior in self.nodes.values():
            if (prior["boot_id"] != event.boot_id or prior["map_epoch"] != event.map_epoch
                    or hypot(x - prior["x_m"], y - prior["y_m"]) >= self.min_separation_m
                    or not prior["free_cells"]):
                continue
            overlap = len(cells & prior["free_cells"]) / len(cells | prior["free_cells"])
            if overlap >= self.overlap_threshold:
                matches.append((overlap, -hypot(x - prior["x_m"], y - prior["y_m"]),
                                prior["node_id"], prior))
        if matches:
            prior = max(matches)[3]
            if not prior["metric_valid"] or prior["map_revision"] != event.map_revision:
                # The old map pose is stale after optimization: reproject it.
                prior.update(map_revision=event.map_revision, map_snapshot_id=event.map_snapshot_id,
                             x_m=x, y_m=y, free_cells=cells, clearance_m=clearance_m,
                             cell_frame="map_quantized_0.1m", metric_valid=True)
            else:
                # Repeated small steps must not drag a place anchor along with
                # the robot and suppress corridor breadcrumb spawning.
                prior["clearance_m"] = min(prior["clearance_m"], clearance_m)
            if event.event_id not in prior["evidence_ids"]:
                prior["evidence_ids"].append(event.event_id)
            if active is not None and active is not prior and active["metric_valid"]:
                self._connect(active["node_id"], prior["node_id"], event,
                              edge_clearance_verified)
            was_other = active is not prior
            self._active_id = prior["node_id"]
            self._pending = None
            return prior["node_id"] if was_other else None
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
                signature = (reason, active["node_id"], x >= active["x_m"], y >= active["y_m"])
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
            "cell_frame": "map_quantized_0.1m",
            "metric_valid": True, "reason": reason, "evidence_ids": [event.event_id],
        }
        if active is not None and active["metric_valid"] and active["map_revision"] == event.map_revision:
            self._connect(active["node_id"], node_id, event, edge_clearance_verified)
        self._active_id = node_id
        self._pending = None
        return node_id

    def _connect(self, origin: str, destination: str, event: EventEnvelope,
                 edge_clearance_verified: bool) -> None:
        for edge in self.edges:
            if ({edge["from"], edge["to"]} == {origin, destination}
                    and edge["map_revision"] == event.map_revision):
                if edge_clearance_verified:
                    edge.update(valid=True, evidence_id=event.event_id)
                return
        self.edges.append({"from": origin, "to": destination,
                           "valid": edge_clearance_verified, "evidence_id": event.event_id,
                           "map_revision": event.map_revision})

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
        graph.nodes = {n["node_id"]: {**n,
                       "free_cells": (frozenset(map(tuple, n["free_cells"]))
                                      if n.get("cell_frame") == "map_quantized_0.1m" else frozenset()),
                       "metric_valid": n["metric_valid"] and n.get("cell_frame") == "map_quantized_0.1m"}
                       for n in value["nodes"]}
        if graph._active_id in graph.nodes and not graph.nodes[graph._active_id]["metric_valid"]:
            graph._active_id = None
        graph.edges = [dict(e) for e in value["edges"]]
        for edge in graph.edges:
            if (edge["from"] not in graph.nodes or edge["to"] not in graph.nodes
                    or not graph.nodes[edge["from"]]["metric_valid"]
                    or not graph.nodes[edge["to"]]["metric_valid"]):
                edge["valid"] = False
        # Pending evidence never survives a restart or map revalidation.
        if verified_identity is None or verified_identity != graph._identity:
            graph.invalidate_revision(verified_identity[2] if verified_identity else 0)
        return graph
