"""Offline Phase A event recorder; no bridge motor path or ROS dependency.

Sensor adapters must prove clock/TF/submap provenance before submitting events.
This component journals decisions before mutating its pose ring/place graph and
replays records deterministically after an interrupted process.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict

from .events import EventEnvelope
from .journal import EventJournal, JournalError
from .map_revision import MapRevisionTracker
from .place_graph import GridGeometry, PlaceGraph
from .pose_history import MapTransform, PoseRing, ProjectedPose, project_capture
from .session import AgentSession, SessionIdentityError


class PhaseARecorder:
    def __init__(self, session: AgentSession, journal: EventJournal,
                 revisions: MapRevisionTracker):
        self.session = session
        self.journal = journal
        self.revisions = revisions
        self.pose_ring = PoseRing()
        self.graph = PlaceGraph()
        self._seen: dict[str, EventEnvelope] = {}
        for event in journal.replay():
            if (event.robot_id, event.session_id, event.boot_id) != (
                    session.robot_id, session.session_id, session.boot_id):
                raise SessionIdentityError("journal identity differs from embodied session")
            self._seen[event.event_id] = event
            if (event.event_kind, event.source) in {("frame_acquired", "camera"),
                                                     ("lidar_observation", "lidar")}:
                self.pose_ring.append(event)
            elif event.event_kind == "place_observation":
                payload = event.payload
                capture = self._seen.get(payload["capture_event_id"])
                if capture is None:
                    raise JournalError("place observation precedes its acquisition")
                if "grid_geometry" not in payload:
                    # Legacy raw indices have no map-frame alignment proof.
                    # Retain the event, never resurrect its metric place node.
                    continue
                pose = ProjectedPose(**payload["projected_pose"])
                self.graph.observe(capture, map(tuple, payload["free_cells"]),
                                   projected_pose=pose,
                                   grid_geometry=GridGeometry(**payload["grid_geometry"]),
                                   clearance_m=payload["clearance_m"],
                                   visibility_radius_m=payload["visibility_radius_m"],
                                   footprint_radius_m=payload["footprint_radius_m"],
                                   bottleneck=payload["bottleneck"], branch=payload["branch"],
                                   edge_clearance_verified=payload["edge_clearance_verified"])
        current = revisions.status()
        if current["status"] != "verified":
            self.pose_ring.clear()
        if (self.graph._identity is not None and
                (current["status"] != "verified" or self.graph._identity !=
                 (current["boot_id"], current["map_epoch"], current["map_revision"]))):
            self.graph.invalidate_revision(current["map_revision"] or 0)
            self.pose_ring.clear()

    def observe_submaps(self, submaps: dict[str, tuple[float, float, float, int]], *,
                        boot_id: str, map_epoch: int,
                        healthy: bool = False) -> tuple[int | None, str]:
        """Advance revision and invalidate all metric edges on uncertainty."""
        previous = self.revisions.status()
        result = self.revisions.observe(submaps, boot_id=boot_id,
                                        map_epoch=map_epoch, healthy=healthy)
        current = self.revisions.status()
        if (current["status"] != "verified" or
                (previous["boot_id"], previous["map_epoch"], previous["map_revision"])
                != (current["boot_id"], current["map_epoch"], current["map_revision"])):
            self.graph.invalidate_revision(current["map_revision"] or 0)
            self.pose_ring.clear()
        return result

    def _owner(self, event: EventEnvelope) -> None:
        if (event.robot_id, event.session_id, event.boot_id) != (
                self.session.robot_id, self.session.session_id, self.session.boot_id):
            raise SessionIdentityError("event belongs to a different embodied session/boot")
        active_lease = self.session.execution["lease_id"]
        if active_lease is not None:
            if (event.mission_id, event.lease_id) != (self.session.mission_id, active_lease):
                raise SessionIdentityError("event belongs to another motion lease")
        elif event.mission_id is not None or event.lease_id is not None:
            raise SessionIdentityError("unowned event cannot claim a motion lease")

    def _revision_matches(self, event: EventEnvelope) -> bool:
        current = self.revisions.status()
        return (current["boot_id"] == event.boot_id and
                current["map_epoch"] == event.map_epoch and
                current["map_revision"] == event.map_revision and
                current["status"] == event.map_revision_status)

    def record_acquisition(self, event: EventEnvelope) -> bool:
        self._owner(event)
        if (event.event_kind, event.source) not in {("frame_acquired", "camera"),
                                                    ("lidar_observation", "lidar")}:
            raise ValueError("expected camera or LiDAR acquisition")
        if not self._revision_matches(event):
            raise SessionIdentityError("event map revision is stale/unproven")
        if event.event_id in self._seen:
            if self._seen[event.event_id] != event:
                raise JournalError("conflicting event ID")
            return False
        self.journal.append(event)
        self._seen[event.event_id] = event
        self.pose_ring.append(event)
        return True

    def record_place(self, event: EventEnvelope, *, free_cells: set[tuple[int, int]],
                     grid_geometry: GridGeometry,
                     optimized: MapTransform, clearance_m: float,
                     visibility_radius_m: float, footprint_radius_m: float,
                     computed_at_robot: float,
                     bottleneck: bool = False,
                     branch: bool = False,
                     edge_clearance_verified: bool = False) -> str | None:
        self._owner(event)
        if not self._revision_matches(event):
            self.graph.invalidate_revision(self.revisions.status()["map_revision"] or 0)
            return None
        if self._seen.get(event.event_id) != event:
            raise JournalError("place evidence must be journaled acquisition")
        if len(free_cells) > 512:
            raise ValueError("place observation exceeds bounded local cells")
        projected = project_capture(event, optimized)
        if projected is None:
            return None
        place_id = "place-obs:" + event.event_id
        if place_id in self._seen:
            return None
        candidate = deepcopy(self.graph)
        node = candidate.observe(event, free_cells, projected_pose=projected,
                                 grid_geometry=grid_geometry,
                                 clearance_m=clearance_m,
                                 visibility_radius_m=visibility_radius_m,
                                 footprint_radius_m=footprint_radius_m,
                                 bottleneck=bottleneck, branch=branch,
                                 edge_clearance_verified=edge_clearance_verified)
        value = event.to_dict()
        value.update(event_id=place_id, event_kind="place_observation", source="map",
                     received_at_robot=computed_at_robot,
                     payload={"capture_event_id": event.event_id,
                              "projected_pose": asdict(projected),
                              "free_cells": sorted(map(list, free_cells)),
                              "grid_geometry": asdict(grid_geometry),
                              "clearance_m": clearance_m,
                              "visibility_radius_m": visibility_radius_m,
                              "footprint_radius_m": footprint_radius_m,
                              "bottleneck": bottleneck, "branch": branch,
                              "edge_clearance_verified": edge_clearance_verified})
        record = EventEnvelope.from_dict(value)
        self.journal.append(record)
        self._seen[place_id] = record
        self.graph = candidate
        return node
