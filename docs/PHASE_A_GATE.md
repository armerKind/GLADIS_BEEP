# Phase A offline gate — cross-model review packet

**Status:** External audit defects 1A–1E corrected in the offline contract and regression-tested. Phase B may proceed **offline**, but this does not certify a live Phase A producer or authorize motion. No bridge deployment, ROS/camera producer integration, or motor activation. The original plan's event-driven observer service is not yet a persistent embodied runtime.

## Review scope

- `beep_agent/events.py`: `EventEnvelope` v1 now records `timestamp_quality`; camera exposure time must be independently proven before metric projection. Old records lacking quality replay as `unknown`, **not** spatial-ready. Boot/session/mission/lease IDs, separate clocks, map epoch/revision/status/snapshot, capture-time local TF, finite JSON and 128 KiB cap remain.
- `beep_agent/map_revision.py`: healthy full-submap-feed observer; growing submap versions/new scans do not increment optimization revision, existing submap placement changes do. Lost/expired feed fails closed, then recovers with a revision increment on a verified superset; a missing known submap needs a new epoch. Cartographer has no exposed optimization counter: the `healthy=True` assertion must come from a future verified producer, not assumed from `/map` updates.
- `beep_agent/pose_history.py`: bounded exact/short-gap local transform history; typed map projection with explicit same boot/epoch/revision/grid and ROS stamp checks. It cannot fix nonrigid pose-graph corrections or infer exposure time from MJPEG bytes.
- `beep_agent/place_graph.py`: explicit grid origin/resolution/yaw/map-frame provenance; raw indices are converted to fixed 0.10 m map-frame buckets before IoU. All existing nodes can be matched on revisit or individually revalidated after revision changes. Direction/quadrant is included in two-tick spawn hysteresis. Footprint clearance remains an input; edges default invalid. Not a navigator.
- `beep_agent/journal.py`, `episodes.py`, `phase_a.py`: fsynced append-only JSONL with cached event-ID index and inode/size/offset tracking; external writers' new tails are parsed under lock, corrupt/partial tails and replacement fail closed. Place records include grid geometry; legacy raw-index place records retain evidence but cannot resurrect metric nodes. Raw media is referenced, not embedded.
- `beep_agent/session.py`: boot-bound sessions accept unowned READY/between-lease perception only with null mission and lease IDs; exact release clears the mission ID. World/goal continuity across leases is tested; `release_motion()` is bookkeeping **only**, not proof the physical robot stopped.

## Tests to inspect

`python -m unittest discover -s tests -p 'test_phase_a_spatial.py' -v`

Covers: same-epoch optimization, transient-gap revision recovery and removed-submap epoch reset, new scan/no revision, wrong boot/epoch/revision, stale/unknown TF, ingest-time camera rejection, projection composition, incremental journal read/partial-tail/replacement, READY/between-lease perception, old-lease rejection, shifted grid origin, same-revision revisit, multi-node revalidation, quadrant hysteresis, default-invalid edges and insufficient clearance. A saved cap-search frame with only a legacy global pose is replayed as **non-spatial** evidence, not promoted to a fabricated capture-time local transform.

Run full suite with `python -m unittest discover -s tests -q` and `python -m compileall -q beep_agent beep_eyes beep_bridge scripts tests` before approving Phase B. Record the actual test total and commit SHA in the handoff message; this document intentionally does not fabricate a static count.

## External audit disposition and remaining integration gates

1. `SubmapList` placement shifts remain only a conservative proxy. A real producer must prove a continuous, healthy full feed and boot/map epochs; a transient gap recovers at an advanced revision, missing known IDs require a new epoch.
2. Dual-clock agreement cannot establish MJPEG exposure time. Only `timestamp_quality: sensor_exposure` permits camera spatial readiness; **no current live producer proves it**.
3. Place events now journal grid-frame origin, yaw and resolution. Indexed cells are converted to fixed map-frame buckets before cross-snapshot overlap; old unaligned records are nonmetric on replay.
4. Unowned perception and exact lease handoff are now covered. Neither event contracts nor graph dispatch motors; physical cancellation/stop races are still a Phase C gate.
5. Two-tick spawn hysteresis now includes candidate quadrant. Synthetic edge/footprint checks cannot certify traversability under 2D LiDAR blind spots.

**Gate:** Corrected Phase A contracts may feed **offline Phase B** work. Before a live observer or metric route, integrate and verify real producer clocks/TF/submaps/grid provenance and independently validate safety. BEEP hardware motion stays gated.
