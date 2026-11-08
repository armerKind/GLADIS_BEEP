# Phase A offline gate — cross-model review packet

**Status:** Offline checkpoint for review before Phase B. No bridge deployment, ROS/camera producer integration, or motor activation. The original plan's event-driven observer service is not yet a persistent embodied runtime; this gate checks the Phase A contracts and replay primitives, not a complete live autonomy stack.

## Review scope

- `beep_agent/events.py`: strict `EventEnvelope` v1, boot/session/mission/lease IDs, two clock fields, map epoch + revision/status + grid snapshot, capture-time `odom → base_link`, finite JSON, 128 KiB cap. Unknown revision/TF cannot become spatial-ready.
- `beep_agent/map_revision.py`: healthy full-submap-feed observer; growing submap versions/new scans do not increment optimization revision, existing submap placement changes do. Lost/expired feed or disappearing submaps fail closed. Cartographer has no exposed optimization counter: the `healthy=True` assertion must come from a future verified producer, not assumed from `/map` updates.
- `beep_agent/pose_history.py`: bounded exact/short-gap local transform history; typed map projection with explicit same boot/epoch/revision/grid and ROS stamp checks. It cannot fix nonrigid pose-graph corrections or infer exposure time from MJPEG bytes.
- `beep_agent/place_graph.py`: conservative, synthetic map-aligned free-cell overlap, persistent bottleneck and corridor breadcrumb spawning, same-revision validation, measured footprint clearance argument, no valid edge by default; reobserved old nodes can be revalidated without resurrecting old edges. Not a navigator.
- `beep_agent/journal.py`, `episodes.py`, `phase_a.py`: fsynced append-only JSONL with duplicate-ID idempotency and corrupt-tail fail-closed; evidence-linked bounded episodes; recorder journals acquisitions/place-observation input before mutating ring/graph and replays graph deterministically. Raw media is referenced, not embedded.
- `beep_agent/session.py`: new snapshot schema v2; legacy v1 snapshots may load without boot-scoped ownership, while boot-bound sessions require explicit exact-lease release before rebinding. World/goal continuity across lease handoff is tested; `release_motion()` is bookkeeping **only**, not proof the physical robot stopped.

## Tests to inspect

`python -m unittest discover -s tests -p 'test_phase_a_spatial.py' -v`

Covers: same-epoch optimization, new scan/no revision, loss of submap feed, wrong boot/epoch/revision, stale/unknown TF, projection composition, duplicate and partial journal, crash/replay, old-lease rejection, open-room dedupe, persistent doorway, corridor breadcrumb, revision invalidation/reobservation, default-invalid edges, and insufficient clearance. A saved cap-search frame with only a legacy global pose is replayed as **non-spatial** evidence, not promoted to a fabricated capture-time local transform.

Run full suite with `python -m unittest discover -s tests -q` and `python -m compileall -q beep_agent beep_eyes beep_bridge scripts tests` before approving Phase B. Record the actual test total and commit SHA in the handoff message; this document intentionally does not fabricate a static count.

## Questions for external gate audit

1. Can `SubmapList` continuity and previously known submap placement shifts establish a sufficiently conservative optimization revision, or must the bridge expose a stronger producer signal? Does the bounded feed-expiry rule appropriately fail closed?
2. Are the proposed dual-clock acquisition and transform stamps sufficient when the current MJPEG frame has **no sensor exposure timestamp**? The current implementation must not claim metric localization of a captured frame until this is measured or redesigned.
3. Are graph free-space cell alignment and `map_snapshot_id` adequate, or should origin/resolution/grid-frame provenance be explicit in v1 before any live graph data is journaled?
4. Are journal replay and session snapshot recovery safe across an unclean process restart, an exact lease handoff, or a map revision change? In particular, does an accepted event ever become a physical effect without independent current-owner validation?
5. Are the graph overlap/bottleneck hysteresis and edge-validation defaults conservative under partial occlusion and 2D LiDAR blind spots? No synthetic test can certify a physical swept path.

**Gate:** Do not start Phase B from passing unit tests alone. Review and close any schema/provenance defects first; BEEP hardware motion stays gated regardless.
