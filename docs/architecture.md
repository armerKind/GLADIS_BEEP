# Architecture Notes

## Goal

BEEP is an embodied sibling-agent platform for GLADIS: a robot body that can perceive, speak, move, map, and execute bounded autonomous behaviors. The bridge is its current local nervous system.

## Where computation runs

Normal robot computation runs on BEEP's Raspberry Pi:

- ROS 2 Foxy and the LiDAR driver
- Cartographer and occupancy-grid publication
- HTTP bridge and sensor monitoring
- local collision checks and motor commands
- camera and analog speaker access

A browser connected to JupyterLab on port `8888` controls a Python kernel running on the Pi. It does not move SLAM computation to the browser's PC unless notebook code explicitly calls a remote service. Jupyter is used for deployment, diagnostics, maintenance, and experiments; it is not the production autonomy loop.

## Runtime topology

```text
GLADIS / operator
   |
   | high-level HTTP request
   v
BEEP bridge :8766 on Pi
   |-- DOGZILLALib SDK -> gait registers and preset actions
   |-- ROS 2 /scan -> local LiDAR sectors and safety
   |-- Cartographer TF -> raw global pose
   |-- ROS 2 /map -> occupancy grid and coverage metrics
   |-- pose guard -> accepted navigation pose
   |-- Yahboom :6500 -> camera MJPEG
   `-- Yahboom :6000 -> optional app-standard stop path

JupyterLab :8888 on Pi
   `-- deploy / reset SLAM / save map / play audio / diagnostics
```

The remote HTTP request may cross Tailscale, but the movement supervision loop stays on BEEP. Network latency is therefore not the primary collision-avoidance mechanism.

## Perception and mapping layers

### 1. Local LiDAR safety

The latest `/scan` is reduced to front, front-left, front-right, left, right, and rear clearances. This is the authoritative layer for immediate movement safety.

```text
GET /scan
GET /local_map
GET /local_map.svg
```

The local map is robot-frame diagnostic geometry. It does not require trustworthy global localization.

### 2. Raw Cartographer SLAM

Cartographer consumes the MS200 LiDAR and publishes TF plus a ROS occupancy grid. The current configuration:

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

## Autonomy modes

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
- hard timeout and unconditional final SDK stop.

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

Coverage mode combines local fluent LiDAR motion with guarded-SLAM map-growth measurement. It stops when known occupancy-grid cells fail to grow by the configured threshold for a full time window. A maximum duration is only a safety ceiling.

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

All autonomy paths execute unconditional SDK stop cleanup. `stop_burst` also attempts the Yahboom app-socket stop, but SDK stop success is the authoritative motor-safety result. An app-socket timeout is recorded separately and must not convert a successful SDK stop into a reported motor failure.

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
- Human-leg detection and semantic object memory are not implemented.

## Next useful upgrades

1. Add pure-localization startup against a selected good `.pbstream`.
2. Integrate a trustworthy odometry or IMU source if hardware permits.
3. Add semantic targets and human-leg detection as a separate perception layer.
4. Persist and compare coverage history across runs.
5. Add authenticated movement control before exposing the bridge beyond trusted networks.
6. Add battery/voltage telemetry if the hardware provides a reliable source.
