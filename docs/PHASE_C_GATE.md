# Phase C offline checkpoint: viewpoint cones and transport witness

**Status:** First shadow-only Phase C slice. Phase A offline contract is approved; Phase B offline tactical/strategy contracts were reviewed and accepted. This checkpoint neither implements a live camera/TF producer nor enables semantic motor dispatch. No bridge code was deployed or run on BEEP.

## Implemented and replayed

- `beep_agent/visual_grounding.py` consumes Phase A `EventEnvelope` and `project_capture()` without migrating the journal schema. A verified sensor-exposure stamp, capture-time valid local TF, matching boot/epoch/revision/snapshot in a current optimized map transform, matching frame/evidence IDs, sufficient detection confidence, a normalized bounding box, and a matching camera-posture calibration are required. Output is a **map-frame uncertain viewpoint cone** from the calibrated camera mount, with no target distance, point coordinate, or motor authority. Stationary observation needs no motion lease.
- `ViewpointRequest` admission is pure and advisory, requiring *current* boot/epoch/revision/snapshot, exact mission and lease, matching source evidence and frame time, a bearing inside the cone, and a short robot-clock expiration. It does **not** validate a path, obstacle clearance, physical camera calibration, map->odom producer, or collision envelope. It cannot move the robot.
- The historical cap packet only records a qualitative `right` position and a frame receipt/capture field whose sensor-exposure provenance is unverified. It cannot yield a metric target position or a valid cone; synthetic bbox/calibration tests exercise the contract without presenting fixture metadata as measured hardware truth.
- An event-controlled `test_transport_fencing.py` witness demonstrates that `/stop` can latch cancellation while `app_transaction()` still holds `app_io_lock` during a blocked greeting. The stop transaction waits for that lock. Existing tests also cover blocked connect, partial writes, lease replacement, and in-flight bytes. **This is an observed host-side limitation, not a stop-latency pass.** No vendor wire behavior or physical stopping distance was measured.

## Remaining Phase C work

1. Provide an actual exposure-timestamped, calibrated camera with posture and bbox producer; verify capture-time ROS TF and current optimized map->odom transform/revision/snapshot against the live Cartographer feed. If any provenance is absent, retain qualitative semantics without metric routing.
2. Implement/validate map-space viewpoint, free-cell, footprint, and discrepancy policies under loop closures and changing sensor quality; do not turn the cone into an object coordinate or a steering command.
3. Refactor/measure app stop priority so a blocked connect/greeting/write cannot starve a stop; characterize worst-case end-to-end stop delay and residual distance with the actual vendor server before considering semantic motor authority. Host-side byte/lock tests do not prove firmware consumption or servo stopping.
4. Phase B live model session, tactical producers, audio mixer, and Pi inference benchmarks also remain separate integration gates. Shadow contracts being approved does not make those processes exist.

## Offline verification

Run `python -m unittest discover -s tests -p 'test_phase_c_grounding.py' -v`, `python -m unittest discover -s tests -p 'test_transport_fencing.py' -v`, and `python -m unittest discover -s tests -q`. Do not reinterpret passing synthetic tests as hardware commissioning.
