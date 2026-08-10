# External room-camera observer

`beep_observer` is an independent supervisory and evidence channel for BEEP. It is **not** a navigator and exposes no locomotion command. Its only optional control authority is an authenticated, rate-limited request to stop and cancel the exact currently active mission.

## Present implementation status

Implemented and offline-tested:

- generic `FrameSource`, `RiskEvaluator`, and `StopSink` protocols;
- deterministic directory/sidecar replay source;
- normalized `clear`, `watch`, and `stop` assessment schema;
- local JSONL evidence with SHA-256 frame correlation;
- image retention only for `watch` and `stop` events;
- record-only default mode;
- explicit stop mode requiring a bearer token;
- client-side stop cooldown;
- bridge-side authentication, exact mission-ID matching, and rate limiting;
- stale mission IDs cannot cancel a newer mission.

Not implemented or claimed:

- no physical camera has been selected;
- no USB/RTSP/ONVIF adapter has been selected or calibrated;
- no body detector, pose tracker, floor homography, zone model, ArUco/AprilTag tracker, or contact classifier has been validated;
- no observer stop has been tested against powered BEEP;
- observer evidence does not prove collision avoidance until synchronized physical validation is completed.

## Placement and calibration requirements

For eventual hardware installation:

1. Mount the camera high in a room corner, looking downward enough to see BEEP's complete body and the surrounding floor.
2. Keep closet faces, doorways, and likely turning zones visible. A camera hidden behind the same occlusion as the robot is merely decorative surveillance.
3. Calibrate floor coordinates and obstacle polygons from measured reference points.
4. Measure end-to-end frame age and reject stale observations.
5. Track the whole body or a conservative body envelope, not a center pixel.
6. Synchronize observer and bridge clocks or record an explicit offset.
7. Begin in record-only mode and compare events against physical video before granting stop authority.

Tags may improve tracking, but the architecture does not assume them. Marker loss must become `watch`/unknown evidence, never an arbitrary movement command.

## Privacy and retention

- Processing and storage are local by default.
- Evidence directories are private (`0700`); event files and retained images are `0600`.
- Clear-frame bytes are not retained. Their hashes may be logged for correlation.
- Only `watch` and `stop` images are retained by default.
- Do not point the camera at private household areas beyond the minimum required floor region.
- Do not upload continuous room video to a remote model. Export only bounded, deliberate evidence when Max explicitly requests it.

## Offline replay

Each image may have a `<filename>.<suffix>.json` sidecar:

```json
{
  "captured_at": 1786380000.25,
  "risk": "stop",
  "reasons": ["body_clearance_below_limit", "commanded_motion_without_displacement"],
  "metrics": {
    "clearance_m": 0.08,
    "displacement_m": 0.0,
    "window_s": 1.0
  }
}
```

Record-only replay:

```bash
python3 scripts/run_external_observer.py \
  --source-dir fixtures/observer \
  --evidence-dir observer_evidence/replay-001
```

Stop-capable replay is intentionally awkward:

```bash
export BEEP_OBSERVER_STOP_TOKEN='use-a-random-secret-not-committed-to-git'
python3 scripts/run_external_observer.py \
  --source-dir fixtures/observer \
  --evidence-dir observer_evidence/replay-002 \
  --mode stop \
  --bridge-url http://192.168.8.88:8766
```

The bridge service must receive the same secret through `BEEP_OBSERVER_STOP_TOKEN`. If the bridge token is unset, `POST /observer/stop` returns `503 observer_stop_disabled`. Because the bearer token is sent in an HTTP header, use this only on the isolated robot LAN or authenticated Tailscale path; do not expose port 8766 to an untrusted network.

## Stop contract

The observer first reads `GET /mission`, then sends:

```http
POST /observer/stop
Authorization: Bearer <token>
Content-Type: application/json
```

```json
{
  "mission_id": "mission-7",
  "event_id": "8a9a558d0aa19e65cc81",
  "reason": "body_clearance_below_limit",
  "observed_at": 1786380001.1
}
```

The bridge rejects missing/invalid authentication, missing mission IDs, stale mission IDs, no-active-mission requests, and requests inside the server cooldown. It delegates accepted requests to exact-ID mission cancellation, which performs the established repeated stop sequence.

## Real camera integration boundary

A future source must implement `FrameSource.read() -> Frame | None`. A future local vision component must implement `RiskEvaluator.evaluate(frame) -> Assessment`. Keep those components outside bridge locomotion. They may report evidence and request stop; they must never choose forward/back/turn commands.
