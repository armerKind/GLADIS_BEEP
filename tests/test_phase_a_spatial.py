"""Phase A offline contract/replay tests; no hardware or motor dispatch."""
import json
import math
import tempfile
import unittest
from pathlib import Path

from beep_agent.events import EventEnvelope, EventValidationError
from beep_agent.pose_history import PoseRing, MapTransform, project_capture
from beep_agent.map_revision import MapRevisionTracker
from beep_agent.place_graph import PlaceGraph
from beep_agent.journal import EventJournal, JournalError
from beep_agent.episodes import EpisodeStore
from beep_agent.phase_a import PhaseARecorder
from beep_agent.session import AgentSession, SessionIdentityError


def frame(name="f1", *, at=1.0, epoch=1, revision=0, status="verified", x=0.0,
          y=0.0, yaw=0.0, snapshot="grid-1", transform=True, boot="boot-1"):
    return EventEnvelope.from_dict({
        "schema_version": 1, "event_id": name, "event_kind": "frame_acquired",
        "source": "camera", "received_at_robot": at,
        "robot_id": "beep", "session_id": "session-1", "boot_id": boot,
        "mission_id": "mission-1", "lease_id": "lease-1",
        "acquired_at_robot": at, "robot_clock": "boot_monotonic",
        "acquired_at_ros_ns": int(at * 1e9),
        "map_epoch": epoch, "map_revision": revision if status == "verified" else None,
        "map_revision_status": status, "map_snapshot_id": snapshot,
        "capture_local_transform": ({"parent_frame": "odom", "child_frame": "base_link",
             "stamp_ros_ns": int(at * 1e9), "x_m": x, "y_m": y, "yaw_rad": yaw,
             "source": "cartographer_local_slam", "valid": True,
             "uncertainty": None} if transform else None),
        "evidence_id": "img-" + name, "payload": {},
    })


def graph_observe(graph, event, cells, **kwargs):
    """Synthetic identity map-to-odom TF; production must supply real map pose."""
    kwargs.setdefault("footprint_radius_m", 0.4)  # Synthetic validated footprint.
    projection = project_capture(event, MapTransform(event.boot_id, event.map_epoch,
        event.map_revision if event.map_revision is not None else 0,
        event.map_snapshot_id or "grid-1", event.acquired_at_ros_ns or 0,
        0, 0, 0))
    return graph.observe(event, cells, projected_pose=projection, **kwargs)


class EnvelopeTests(unittest.TestCase):
    def test_roundtrip_and_no_metric_effect_when_revision_unknown_or_tf_missing(self):
        e = frame()
        self.assertTrue(e.spatial_ready)
        self.assertEqual(EventEnvelope.from_dict(json.loads(json.dumps(e.to_dict()))), e)
        self.assertFalse(frame(status="unknown").spatial_ready)
        self.assertFalse(frame(transform=False).spatial_ready)

    def test_rejects_invalid_clock_nan_and_ambiguous_revision(self):
        base = frame().to_dict()
        for change in ({"acquired_at_robot": float("nan")},
                       {"robot_clock": "wall"},
                       {"map_revision": -1},
                       {"map_revision": None},
                       {"map_revision_status": "unknown"},
                       {"acquired_at_ros_ns": -1}):
            with self.subTest(change=change), self.assertRaises(EventValidationError):
                EventEnvelope.from_dict(dict(base, **change))
        with self.assertRaises(EventValidationError):
            EventEnvelope.from_dict(dict(base, capture_local_transform=dict(
                base["capture_local_transform"], yaw_rad=float("inf"))))

    def test_nonspatial_event_keeps_envelope_fields(self):
        value = frame().to_dict()
        value.update(event_id="speech-1", event_kind="speech", capture_local_transform=None,
                     map_revision=None, map_revision_status="unknown", map_snapshot_id=None)
        self.assertFalse(EventEnvelope.from_dict(value).spatial_ready)


class PoseRingTests(unittest.TestCase):
    def test_interpolated_transform_cannot_masquerade_as_acquired_frame(self):
        ring = PoseRing(max_gap_s=0.05)
        ring.append(frame("a", at=1.0, x=0))
        ring.append(frame("b", at=1.02, x=1))
        middle = ring.at(1.01, boot_id="boot-1", map_epoch=1, map_revision=0)
        self.assertIsNotNone(middle)
        assert middle is not None and middle.capture_local_transform is not None
        self.assertEqual(middle.event_kind, "interpolated_transform")
        self.assertAlmostEqual(middle.capture_local_transform["x_m"], 0.5)
        graph = PlaceGraph()
        self.assertIsNone(graph_observe(graph, middle, {(0, 0)},
                                        clearance_m=1, visibility_radius_m=1))

    def test_local_projection_and_same_epoch_revision_rejection(self):
        event = frame(x=1, y=0, yaw=0)
        identity = MapTransform("boot-1", 1, 0, "grid-1", 1_000_000_000, 0, 0, 0)
        before = project_capture(event, identity)
        self.assertIsNotNone(before)
        assert before is not None
        self.assertEqual((before.x_m, before.y_m), (1, 0))
        shifted = MapTransform("boot-1", 1, 0, "grid-1", 1_100_000_000, 2, 3, math.pi / 2)
        after = project_capture(event, shifted)
        self.assertIsNotNone(after)
        assert after is not None
        self.assertAlmostEqual(after.x_m, 2)
        self.assertAlmostEqual(after.y_m, 4)
        self.assertIsNone(project_capture(event, MapTransform("boot-1", 1, 1, "grid-1",
                                                         1_100_000_000, 2, 3, 0)))
        self.assertIsNone(project_capture(event, MapTransform("boot-1", 1, 0, "grid-2",
                                                         1_100_000_000, 2, 3, 0)))
        self.assertIsNone(project_capture(event, MapTransform("boot-1", 1, 0, "grid-1",
                                                         3_100_000_000, 2, 3, 0)))

    def test_capture_time_lookup_and_bounded_history(self):
        ring = PoseRing(capacity=2, max_gap_s=0.05)
        for index, t in enumerate((1.0, 1.1, 1.2)):
            ring.append(frame(f"f{index}", at=t, x=index))
        pose = ring.at(1.2, boot_id="boot-1", map_epoch=1, map_revision=0)
        self.assertIsNotNone(pose)
        assert pose is not None and pose.capture_local_transform is not None
        self.assertEqual(pose.capture_local_transform["x_m"], 2)
        self.assertIsNone(ring.at(1.0, boot_id="boot-1", map_epoch=1, map_revision=0))
        self.assertIsNone(ring.at(1.125, boot_id="boot-1", map_epoch=1, map_revision=0))
        self.assertIsNone(ring.at(1.2, boot_id="other", map_epoch=1, map_revision=0))
        self.assertIsNone(ring.at(1.2, boot_id="boot-1", map_epoch=2, map_revision=0))
        self.assertIsNone(ring.at(1.2, boot_id="boot-1", map_epoch=1, map_revision=1))
        ring.append(frame("stale", at=1.3, status="unknown"))
        self.assertIsNone(ring.at(1.3, boot_id="boot-1", map_epoch=1, map_revision=0))


class RevisionTests(unittest.TestCase):
    def test_same_epoch_optimization_increments_but_new_scan_does_not(self):
        tracker = MapRevisionTracker(position_epsilon_m=0.01)
        self.assertEqual(tracker.status()["status"], "unknown")
        self.assertEqual(tracker.observe({"1": (0.0, 0.0, 0.0, 1)}, boot_id="b", map_epoch=1, healthy=True), (0, "verified"))
        self.assertEqual(tracker.observe({"1": (0.0, 0.0, 0.0, 2), "2": (1.0, 0.0, 0.0, 1)}, boot_id="b", map_epoch=1, healthy=True), (0, "verified"))
        self.assertEqual(tracker.observe({"1": (0.2, 0.0, 0.0, 2), "2": (1.0, 0.0, 0.0, 1)}, boot_id="b", map_epoch=1, healthy=True), (1, "verified"))
        self.assertEqual(tracker.observe({"1": (0.2, 0.0, 0.0, 2)}, boot_id="b", map_epoch=1, healthy=True), (None, "unknown"))
        self.assertEqual(tracker.observe({"1": (0.0, 0.0, 0.0, 1)}, boot_id="b", map_epoch=2, healthy=True), (0, "verified"))

    def test_no_revision_proof_fails_closed(self):
        tracker = MapRevisionTracker()
        self.assertEqual(tracker.observe({}, boot_id="b", map_epoch=1), (None, "unknown"))
        self.assertEqual(tracker.observe({"1": (0.0, 0.0, 0.0, 1)}, boot_id="b", map_epoch=1, healthy=False), (None, "unknown"))

    def test_revision_feed_expiry_requires_new_epoch_after_gap(self):
        ticks = [10.0]
        tracker = MapRevisionTracker(clock=lambda: ticks[0], max_age_s=1)
        sample = {"1": (0.0, 0.0, 0.0, 1)}
        self.assertEqual(tracker.observe(sample, boot_id="b", map_epoch=1, healthy=True), (0, "verified"))
        ticks[0] = 11.1
        self.assertEqual(tracker.status()["status"], "unknown")
        self.assertEqual(tracker.observe(sample, boot_id="b", map_epoch=1, healthy=True), (None, "unknown"))
        self.assertEqual(tracker.observe(sample, boot_id="b", map_epoch=2, healthy=True), (0, "verified"))


class GraphTests(unittest.TestCase):
    def test_lidar_free_space_can_spawn_place_without_camera(self):
        graph = PlaceGraph()
        data = frame("scan-1").to_dict()
        data.update(event_kind="lidar_observation", source="lidar", evidence_id="scan-1")
        event = EventEnvelope.from_dict(data)
        self.assertEqual(graph_observe(graph, event, {(0, 0), (1, 0)},
                         clearance_m=1, visibility_radius_m=1), "place-0001")

    def test_no_node_inside_footprint_or_from_unprojected_local_pose(self):
        graph = PlaceGraph()
        cells = {(0, 0), (1, 0)}
        event = frame()
        self.assertIsNone(graph.observe(event, cells, projected_pose=None,
                                        clearance_m=1, visibility_radius_m=1,
                                        footprint_radius_m=0.4))
        self.assertIsNone(graph_observe(graph, event, cells,
                          clearance_m=0.3, visibility_radius_m=1,
                          footprint_radius_m=0.4))
        self.assertEqual(graph.nodes, {})

    def test_room_dedupe_doorway_corridor_and_revision_invalidation(self):
        graph = PlaceGraph(overlap_threshold=0.5, min_separation_m=0.4)
        room = {(x, y) for x in range(6) for y in range(6)}
        self.assertEqual(graph_observe(graph, frame("r", x=0), room, clearance_m=1.0, visibility_radius_m=2), "place-0001")
        for x in (0.1, 0.2, 0.3):
            self.assertIsNone(graph_observe(graph, frame(f"repeat-{x}", x=x), room, clearance_m=1.0, visibility_radius_m=2))
        self.assertEqual(len(graph.nodes), 1)
        persisted = json.loads(json.dumps(graph.to_dict()))
        self.assertFalse(PlaceGraph.from_dict(persisted).nodes["place-0001"]["metric_valid"])
        self.assertTrue(PlaceGraph.from_dict(persisted, verified_identity=("boot-1", 1, 0))
                        .nodes["place-0001"]["metric_valid"])
        door = {(x, y) for x in range(5, 9) for y in range(2, 4)}
        self.assertIsNone(graph_observe(graph, frame("door-a", x=1), door, clearance_m=0.5,
                                        visibility_radius_m=1, bottleneck=True))
        self.assertEqual(graph_observe(graph, frame("door-b", x=1.1), door, clearance_m=0.5,
                                       visibility_radius_m=1, bottleneck=True), "place-0002")
        self.assertEqual(len(graph.edges), 1)
        self.assertFalse(graph.edges[0]["valid"])  # No verified swept-footprint edge.
        graph.invalidate_revision(1)
        self.assertFalse(graph.edges[0]["valid"])
        self.assertFalse(graph.nodes["place-0001"]["metric_valid"])
        self.assertFalse(graph.nodes["place-0002"]["metric_valid"])
        self.assertEqual(graph_observe(graph, frame("revalidated", revision=1, x=0.1), room,
                        clearance_m=1, visibility_radius_m=2), "place-0001")
        self.assertEqual(len(graph.nodes), 2)
        self.assertTrue(graph.nodes["place-0001"]["metric_valid"])
        self.assertFalse(graph.edges[0]["valid"])
        self.assertIsNone(graph_observe(graph, frame("unknown", revision=1, status="unknown"), room,
                                        clearance_m=1, visibility_radius_m=2))

    def test_corridor_breadcrumb_requires_new_retrace_geometry(self):
        graph = PlaceGraph()
        corridor = {(x, y) for x in range(20) for y in (0, 1)}
        graph_observe(graph, frame("start", x=0), corridor, clearance_m=0.65, visibility_radius_m=0.7)
        self.assertIsNone(graph_observe(graph, frame("near", x=0.5), corridor, clearance_m=0.65, visibility_radius_m=0.7))
        self.assertEqual(graph_observe(graph, frame("far", x=2.0), corridor, clearance_m=0.65,
                                       visibility_radius_m=0.7), "place-0002")
        self.assertEqual(len(graph.edges), 1)


class JournalEpisodeTests(unittest.TestCase):
    def test_append_only_roundtrip_conflict_and_episode_evidence(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "journal.jsonl"
            journal = EventJournal(path)
            e = frame()
            journal.append(e)
            journal.append(e)
            self.assertEqual(len(journal.replay()), 1)
            with self.assertRaises(JournalError):
                journal.append(frame("f1", x=3))
            episodes = EpisodeStore(max_episodes=2)
            episodes.add("Inspected the room", evidence_ids=["f1"], journal=journal,
                         outcome="observed")
            with self.assertRaises(ValueError):
                episodes.add("Invented encounter", evidence_ids=["absent"], journal=journal,
                             outcome="observed")
            self.assertEqual(episodes.to_dict()["episodes"][0]["evidence_ids"], ["f1"])
            self.assertEqual(len(journal.replay()), 1)

    def test_corrupt_journal_fails_closed_without_rewriting(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "journal.jsonl"
            EventJournal(path).append(frame())
            with path.open("a") as stream:
                stream.write('{"partial":')
            with self.assertRaises(JournalError):
                EventJournal(path).replay()
            self.assertIn('{"partial":', path.read_text())


class EmbodiedSessionTests(unittest.TestCase):
    def test_legacy_snapshot_loads_only_without_boot_identity_claim(self):
        old = AgentSession(robot_id="beep", mission_id="legacy").to_dict()
        old["schema_version"] = 1
        del old["boot_id"]
        old["execution"].pop("lease_history", None)
        resumed = AgentSession.from_dict(old, expected_robot_id="beep")
        self.assertIsNone(resumed.boot_id)
        with self.assertRaises(SessionIdentityError):
            AgentSession.from_dict(old, expected_boot_id="boot-1")

    def test_exact_owner_rebind_keeps_world_and_session_across_lease(self):
        session = AgentSession(robot_id="beep", mission_id="m1", boot_id="boot-1")
        session.bind_motion(mission_id="m1", lease_id="l1", boot_id="boot-1")
        sample = json.loads((Path(__file__).parent / "fixtures" / "eyes_response.json").read_text())
        session.update(sample, observed_at=1, packet_id="p1", mission_id="m1", lease_id="l1")
        before = session.world
        sid = session.session_id
        with self.assertRaises(SessionIdentityError):
            session.bind_motion(mission_id="m2", lease_id="l2", boot_id="boot-1")
        session.release_motion(mission_id="m1", lease_id="l1")
        session.bind_motion(mission_id="m2", lease_id="l2", boot_id="boot-1")
        session.update(sample, observed_at=2, packet_id="p2", mission_id="m2", lease_id="l2")
        self.assertIs(session.world, before)
        self.assertEqual(session.world.sequence, 2)
        self.assertEqual(session.session_id, sid)
        with self.assertRaises(SessionIdentityError):
            session.update({}, observed_at=1, packet_id="old", mission_id="m1", lease_id="l1")
        with self.assertRaises(SessionIdentityError):
            session.bind_motion(mission_id="m3", lease_id="l3", boot_id="another-boot")
        self.assertEqual(session.mission_id, "m2")
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "session.json"
            session.save(path)
            restored = AgentSession.load(path, expected_boot_id="boot-1")
        self.assertEqual(restored.execution["lease_id"], "l2")
        self.assertEqual(restored.session_id, sid)


class RecorderTests(unittest.TestCase):
    def test_saved_cap_frame_cannot_invent_acquisition_tf_from_global_pose(self):
        saved = json.loads((Path(__file__).parent / "fixtures" /
                            "cap_confirmation_window0.json").read_text())
        source = saved["packet"]["frames"][0]
        self.assertEqual(source["pose"]["source"], "guarded_cartographer_slam")
        with tempfile.TemporaryDirectory() as root:
            session = AgentSession(robot_id="beep", mission_id="mission-1",
                                   boot_id="boot-1", session_id="session-1")
            session.bind_motion(mission_id="mission-1", lease_id="lease-1", boot_id="boot-1")
            tracker = MapRevisionTracker()
            tracker.observe({}, boot_id="boot-1", map_epoch=1)
            recorder = PhaseARecorder(session, EventJournal(Path(root) / "events.jsonl"), tracker)
            value = frame(status="unknown", transform=False).to_dict()
            value.update(event_id=source["frame_id"], evidence_id=source["frame_id"],
                         map_snapshot_id=None,
                         payload={"legacy_captured_at_wall": source["captured_at"],
                                  "legacy_global_pose": source["pose"]})
            event = EventEnvelope.from_dict(value)
            self.assertTrue(recorder.record_acquisition(event))
            self.assertFalse(event.spatial_ready)
            self.assertEqual(recorder.graph.nodes, {})
            self.assertIsNone(recorder.pose_ring.at(1, boot_id="boot-1", map_epoch=1,
                                                    map_revision=0))

    def test_append_before_state_and_exact_owner_and_revision_replay(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root) / "events.jsonl"
            session = AgentSession(robot_id="beep", mission_id="mission-1",
                                   boot_id="boot-1", session_id="session-1")
            session.bind_motion(mission_id="mission-1", lease_id="lease-1", boot_id="boot-1")
            tracker = MapRevisionTracker(position_epsilon_m=0.01)
            tracker.observe({"1": (0., 0., 0., 1)}, boot_id="boot-1", map_epoch=1, healthy=True)
            recorder = PhaseARecorder(session, EventJournal(path), tracker)
            e = frame()
            self.assertTrue(recorder.record_acquisition(e))
            self.assertFalse(recorder.record_acquisition(e))
            self.assertEqual(len(EventJournal(path).replay()), 1)
            self.assertIsNotNone(recorder.pose_ring.at(1, boot_id="boot-1", map_epoch=1,
                                                        map_revision=0))
            old_owner = e.to_dict()
            old_owner.update(event_id="old", lease_id="old-lease")
            with self.assertRaises(SessionIdentityError):
                recorder.record_acquisition(EventEnvelope.from_dict(old_owner))
            self.assertEqual(len(EventJournal(path).replay()), 1)
            room = {(x, y) for x in range(4) for y in range(4)}
            tf = MapTransform("boot-1", 1, 0, "grid-1", 1_000_000_000, 0, 0, 0)
            self.assertEqual(recorder.record_place(e, free_cells=room, optimized=tf,
                                clearance_m=0.9, visibility_radius_m=1,
                                footprint_radius_m=0.4, computed_at_robot=1.1), "place-0001")
            self.assertEqual(len(EventJournal(path).replay()), 2)
            resumed = PhaseARecorder(session, EventJournal(path), tracker)
            self.assertEqual(resumed.graph.to_dict(), recorder.graph.to_dict())
            recorder.observe_submaps({"1": (0.5, 0., 0., 1)}, boot_id="boot-1", map_epoch=1, healthy=True)
            self.assertFalse(recorder.graph.nodes["place-0001"]["metric_valid"])
            self.assertIsNone(recorder.pose_ring.at(1, boot_id="boot-1", map_epoch=1,
                                                    map_revision=0))
            self.assertIsNone(recorder.record_place(e, free_cells=room, optimized=tf,
                              clearance_m=0.9, visibility_radius_m=1,
                              footprint_radius_m=0.4, computed_at_robot=1.1))
            unproven_restart = PhaseARecorder(session, EventJournal(path), MapRevisionTracker())
            self.assertFalse(unproven_restart.graph.nodes["place-0001"]["metric_valid"])
            self.assertIsNone(unproven_restart.pose_ring.at(1, boot_id="boot-1",
                                                       map_epoch=1, map_revision=0))
