# Architecture Notes

## Goal

BEEP is an embodied sibling-agent platform for GLADIS: a robot body that can perceive, speak, move, map, and execute bounded autonomous behaviors. The bridge is its current local nervous system.

## Where computation runs

Normal robot computation runs on BEEP's Raspberry Pi:

- ROS 2 Foxy and the LiDAR driver
- Cartographer and occupancy-grid publication
- HTTP bridge and sensor monitoring
- local collision checks and motor commands
- camera access; analog speaker playback is provided separately by a Jupyter maintenance helper

A browser connected to JupyterLab on port `8888` controls a Python kernel running on the Pi. It does not move SLAM computation to the browser's PC unless notebook code explicitly calls a remote service. Jupyter is used for deployment, diagnostics, maintenance, and experiments; it is not the production autonomy loop.

## Runtime topology

```text
GLADIS / operator
   |-- continuous moving-frame ring (4/6/9 panels)
   |-- Gemini semantic perception
   |-- persistent world model, drives, executive, typed body adapter
   |
   | high-level HTTP request
   v
BEEP bridge :8766 on Pi
   |-- DOGZILLALib SDK -> gait registers and preset actions
   |-- ROS 2 /scan -> local LiDAR sectors and safety
   |-- Cartographer TF -> raw global pose
   |-- ROS 2 /map -> occupancy grid and coverage metrics
   |-- pose guard -> accepted navigation pose
   |-- asynchronous mission worker + immutable lease watchdog
   |-- Yahboom :6500 -> camera MJPEG
   `-- Yahboom :6000 -> optional app-standard stop path

JupyterLab :8888 on Pi
   `-- deploy / reset SLAM / save map / play audio / diagnostics
```

The remote HTTP request may cross Tailscale, but autonomous movement supervision stays on BEEP. Generic `/move` and vendor preset `/action` bypass the autonomous LiDAR/map guards and must be treated as supervised low-level interfaces.

## Perception and mapping layers

### 1. Local LiDAR safety

The latest `/scan` is reduced to front, front-left, front-right, left, right, and rear clearances. This is the authoritative layer for immediate movement safety inside LiDAR-aware approach and autonomy controllers.

```text
GET /scan
GET /local_map
GET /local_map.svg
```

The local map is robot-frame diagnostic geometry. It does not require trustworthy global localization.

### 2. Raw Cartographer SLAM

Cartographer consumes the ROS 2D LiDAR `/scan` stream and publishes TF plus a ROS occupancy grid. The current configuration:

- does not use wheel odometry,
- does not use IMU data,
- keeps online correlative scan matching for quadruped turns,
- strongly penalizes invented translation and rotation,
- filters stationary jitter,
- reduces pose-graph optimization load.

Raw TF is retained under `slam.raw_pose` for diagnosis. Because BEEP is a swaying, slipping quadruped, raw scan matching can still produce fictional movement.

### 3. Guarded navigation pose

The bridge exposes an accepted pose with source:

```text
guarded_cartographer_slam
```

The guard:

- anchors stationary updates inside a 0.12 m / 0.18 rad envelope,
- rejects implausible moving translation or yaw jumps,
- allows a short post-motion scan-matching window,
- records the raw pose, rejection reason, and rejection count,
- blocks frontier or coverage claims when localization becomes invalid.

Dead reckoning remains available only as a legacy/fallback estimate and must not overwrite a fresh guarded Cartographer pose.

### 4. Legacy bridge map

`GET /map` and `/map.svg` expose the bridge's older sparse diagnostic map. The authoritative global occupancy grid for navigation is ROS `/map`, summarized under the `slam` object returned by `/health` and `/status`.

### 5. Moving temporal vision

`ContinuousMovingCapture` runs on GLADIS independently of motor execution. It samples bridge JPEGs at approximately 3 FPS into a bounded ring, renders the newest chronological 4-, 6-, or 9-frame window, and sends one panel plus the latest full-resolution frame to Gemini. Capture continues during inference; completed requests use the newest available window rather than queueing stale evidence. During any active lease the bridge camera path bypasses vendor app-control preparation, preventing frame acquisition from injecting an app-level stop.

Gemini provides semantic scene evidence, not collision reflexes or raw motor commands. LiDAR remains authoritative for immediate safety. Individual blurred frames are tolerated through temporal redundancy and recorded quality metadata.

### 6. Persistent semantic world

`beep_agent` associates packet-local people/objects with stable local tracks, arbitrates personality-shaped drives, selects goals, emits strict typed semantic skills, and maps the implemented `explore`, `observe`, and `stop` skills through `BridgeBodyAdapter`. The moving-eyes supervisor updates this world model and prepares the next skill while navigation continues; deterministic semantic course updates during an active mission remain a future adapter.

## Autonomy modes

### Asynchronous autonomous mission

```text
POST /mission/start
GET  /mission
POST /mission/cancel
```

The start request validates fresh LiDAR and, for coverage mode, usable guarded SLAM, then acquires one exclusive immutable lease and returns HTTP 202. A dedicated onboard worker maintains fluent local navigation while `/status`, `/mission`, and exact-ID cancellation remain responsive. The independent lease-deadline watchdog latches cancellation and attempts repeated stops even if the owner or remote HTTP client stalls. Conflicting motion returns HTTP 409 and is never queued.

### Legacy bridge exploration

```text
GET /explore_room
GET /explore
```

This compatibility mode checks fresh LiDAR and `slam.active` before starting, then drives the legacy bridge mapper. Unlike frontier and coverage modes, it does not continuously enforce guarded-pose validity or map/pose freshness during the loop. Prefer the newer modes for map-directed claims.

### Local LiDAR walk

```text
GET /lidar_walk
GET /demo_walk
```

- no global-localization claim,
- one fluent forward gait across compatible windows,
- forward-plus-yaw arcs when geometry permits,
- in-place turns and lateral escapes when blocked,
- approximately 80 ms local safety polling,
- hard timeout and final SDK stop attempts.

### Frontier exploration

```text
GET /frontier_explore
```

- clusters known-free/unknown boundaries,
- inflates obstacles for leg sweep,
- runs A* through known free space,
- replans after supervised gait windows,
- aborts on stale or rejected SLAM,
- watches repeated turn progress.

Frontiers can disappear as the live map changes during a scan or turn. A return reason such as `coverage_complete_or_no_reachable_frontiers` must be interpreted together with the trace; a turns-only trace is not proof of physical traversal.

### Coverage exploration

```text
GET /coverage_explore
```

Coverage mode combines local fluent LiDAR motion with guarded-SLAM map-growth measurement. It aborts when `pose_valid` is false, pose age exceeds 6.0 s, or map age exceeds 2.5 s. Plateau requires samples spanning at least 90% of the configured window and `max(known_cells) - min(known_cells) < min_growth_cells`. A maximum duration is only a safety ceiling.

This is a more honest room-completion heuristic than elapsed time, but it is not mathematical proof that every occluded corner or inaccessible area was visited.

### Marking behavior

`pee` is only vendor preset action 11: a right-leg lift.

`mark_object` means:

1. approach a frontal target under LiDAR supervision,
2. stop at marking distance,
3. turn sideways,
4. perform `pee`.

The current target is generic frontal LiDAR geometry. Human-leg detection, person selection, and socially targeted marking remain future work. An oddly specific roadmap, but a roadmap nonetheless.

## Stop semantics

Autonomy paths execute repeated SDK stop attempts during cleanup. Every lease also has an independent onboard deadline watchdog tied to the exact immutable owner. `/stop` and `/mission/cancel` latch the active owner's cancellation token and issue repeated stops without waiting for a long navigation handler. `stop_burst` also attempts the Yahboom app-socket stop, but at least one successful SDK attempt is the authoritative reported motor-safety result. All SDK calls can still fail, so software cleanup is not a physical-stop guarantee. An app-socket timeout is recorded separately and must not convert a successful SDK stop into a reported motor failure.

## Known limitations

- No trusted wheel odometry.
- No integrated ROS IMU input for Cartographer.
- Raw localization can drift under gait vibration or repetitive geometry.
- Pose guarding rejects bad localization; it cannot manufacture missing odometry.
- Saved `.pbstream` state exists, but production pure-localization startup/relocalization is not yet implemented.
- Coverage saturation can miss occluded or physically unreachable space.
- Camera quality depends heavily on lighting.
- No battery telemetry.
- The external Wi-Fi adapter and Tailscale path can disappear while local robot services continue.
- Human-leg detection and socially targeted marking are not implemented.
- Persistent visual object/person tracks exist, but metric visual localization and deterministic semantic course adjustment during an active mission remain incomplete.
- Asynchronous missions, exact cancellation, and four-frame moving temporal vision are physically validated on bridge `0.17.3-moving-camera-handshake`; six- and nine-frame gait panels remain unvalidated.

## Next useful upgrades

1. Add pure-localization startup against a selected good `.pbstream`.
2. Integrate a trustworthy odometry or IMU source if hardware permits.
3. Physically validate asynchronous cancellation and moving 4/6/9-frame perception, then add deterministic semantic course updates.
4. Persist and compare coverage history across runs.
5. Add authenticated movement control before exposing the bridge beyond trusted networks.
6. Add battery/voltage telemetry if the hardware provides a reliable source.
7. Add human-leg detection and socially appropriate target policies as a separate perception layer.
