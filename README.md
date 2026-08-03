# GLADIS_BEEP

Software, deployment helpers, and operational notes for **BEEP** — a Yahboom DogZilla S2 quadruped used as an embodied sibling-agent experiment for GLADIS.

BEEP combines the vendor motor SDK, 2D LiDAR, ROS 2, Cartographer, camera, and a local HTTP bridge. Jupyter maintenance helpers can also drive the Pi's analog speaker. Autonomous sensor polling and stop cleanup run on BEEP's Raspberry Pi; GLADIS sends high-level requests over the network.

## Current state

Current bridge source line (the last physically verified deployment was `0.14.1-coverage-pose-cadence`):

```text
0.14.2-bounded-manual-move
```

Current source implements the following. The main locomotion, mapping, coverage, camera, and speaker paths have also been exercised on BEEP; unit tests alone are not presented as physical verification.

- SDK-backed locomotion through `DOGZILLALib`
- fresh-LiDAR gates and final SDK stop attempts in autonomous LiDAR-aware controllers
- fluent forward gait, forward-plus-yaw walking arcs, bounded turns, and lateral escapes
- camera frame capture from the Yahboom MJPEG service
- named vendor actions such as stretch, pray, and the right-leg `pee` preset
- continuous LiDAR-supervised approach and marking sequence
- persistent ROS 2 Cartographer and occupancy-grid services
- raw-versus-guarded Cartographer pose with stationary-drift and impossible-jump rejection
- systematic frontier planning over the ROS occupancy grid
- local presentation-safe LiDAR walking when global localization is not trustworthy
- coverage-driven room exploration that stops when mapped known-space growth plateaus
- Jupyter-based deployment, SLAM reset/export, diagnostics, and speaker playback helpers
- MAC-bound dual Wi-Fi boot configu...[truncated]

## What “SLAM” means here

BEEP runs Cartographer on its Raspberry Pi. The vendor's Jupyter examples demonstrate those components; Jupyter is a browser interface to code executing on the Pi, not a remote SLAM computer.

The current robot does **not** expose trusted wheel odometry or a dependable ROS IMU input to Cartographer. LiDAR scan matching is therefore vulnerable to gait vibration, foot slip, scan distortion, and repetitive room geometry. The bridge keeps Cartographer's raw TF pose for diagnostics and exposes a guarded pose for navigation. Fresh local LiDAR remains authoritative for immediate collision safety.

This is genuine online mapping with pose-graph SLAM, but it is not a finished consumer-robot localization appliance. Marketing survived. Precision required engineering.

## Runtime architecture

```text
GLADIS / operator
      |
      | HTTP over DOGZILLA_WIFI or Tailscale
      v
BEEP bridge :8766 (Raspberry Pi)
      |-- DOGZILLALib motor SDK
      |-- ROS 2 /scan + fresh-LiDAR safety
      |-- Cartographer TF + /map monitoring
      |-- guarded pose and exploration controllers
      |-- Yahboom camera/app services

JupyterLab :8888 (Raspberry Pi)
      `-- deployment, diagnostics, SLAM maintenance, audio playback
```

Normal autonomy does not depend on an open Jupyter notebook or a responsive remote connection.

## Repository layout

```text
beep_bridge/       HTTP bridge and frontier planner
config/            Cartographer configuration
systemd/           bridge, Cartographer, and occupancy-grid units
scripts/           deployment, synchronization, SLAM, capture, and audio helpers
tests/             planner, bridge integration, pose-guard, and behavior tests
docs/              architecture, API, deployment, and network notes
assets/ros_maps/   selected historical Cartographer artifacts
maps/              placeholder for ignored generated bridge maps
```

## Primary endpoints

```text
GET /health                     liveness plus nested runtime status snapshot
GET /stop                       repeated SDK stop attempts plus app-stop telemetry
GET /lidar_walk                 local LiDAR-reactive fluent navigation
GET /frontier_explore           guarded-SLAM frontier planning/navigation
GET /coverage_explore           explore until guarded map growth plateaus
GET /action?name=stretch        named vendor preset
GET /mark_object?dry_run=1      preview approach → turn sideways → pee sequence
GET /frame.jpg                  camera frame
```

See [`docs/bridge_api.md`](docs/bridge_api.md) for parameters and safety semantics.

## Useful scripts

```bash
scripts/check_bridge.sh                 # Check live bridge health
scripts/capture_bridge_state.sh         # Capture status/maps/frame locally
scripts/sync_from_pi.py                 # Pull bridge and selected Pi artifacts
scripts/jupyter_deploy_bridge.py        # Upload bridge/planner/config and restart
scripts/jupyter_reset_slam.py           # Start a clean Cartographer trajectory
scripts/jupyter_save_slam.py            # Finish and export pbstream/PGM/YAML
scripts/jupyter_play_audio.py FILE.mp3   # Upload and play through Pi analog audio
scripts/probe_bridge_perception.py       # Probe camera, microphone, API latency, and Pi resources
scripts/run_beep_eyes.py                 # Temporal camera/replay → Gemini → typed shadow policy
scripts/run_beep_mission.py              # Persistent world → autonomous goal → semantic skill trace
scripts/install_systemd_service.py      # Install the bridge systemd service
scripts/save_slam_map.sh                # Pi-side Cartographer exporter
```

Jupyter helpers default to `192.168.8.88`; set `BEEP_HOST=beep.tailb08b32.ts.net` for the Tailscale path. The source retains Yahboom's vendor-default Jupyter password for the stock image; use `BEEP_JUPYTER_PASSWORD` for any changed credential and never commit private alternatives or tokens.

## Networking

BEEP's intended dual-network arrangement is:

```text
wlan0  original DOGZILLA_WIFI access point / robot network
wlan1  external 2.4 GHz Wi-Fi uplink
```

Addresses last physically verified on 2026-07-19:

```text
192.168.8.88                    local DogZilla address
beep.tailb08b32.ts.net          Tailscale MagicDNS name
100.100.141.33                  last assigned Tailscale address
```

Useful ports:

```text
8766  BEEP bridge HTTP API
8888  JupyterLab on Pi
6000  Yahboom app-control server
6500  Yahboom MJPEG camera stream
22    SSH when authorized/reachable
```

Tailscale can use a high-latency DERP relay or disappear with the external Wi-Fi uplink. Motor safety must therefore remain local.

## Safety model

- Autonomous navigation and approach routines require a fresh LiDAR scan; generic `/move` and vendor `/action` are lower-level interfaces and are not scan-gated.
- Local LiDAR sectors govern immediate obstacle stops inside LiDAR-aware controllers.
- Guarded SLAM is required for map-directed frontier/coverage claims.
- Reject stale map/pose, impossible relocation, repeated turn non-progress, and unsafe clearances.
- Autonomous movement routines attempt final SDK stops on completion, timeout, and exception; a hardware/SDK failure can still prevent a physical stop.
- SDK stop success is authoritative; the optional Yahboom app-socket stop is best-effort telemetry.
- No battery telemetry is available; use charged batteries and supervised bounded runs.
- Do not infer “whole room” solely from elapsed time. Coverage mode uses map-growth saturation and still cannot prove access beneath or behind every obstacle.

## Development

Run validation from the repository root:

```bash
python3 -m py_compile beep_bridge/beep_bridge.py beep_bridge/frontier_planner.py tests/test_*.py
python3 -m unittest discover -s tests -v
git diff --check
```

The eyes subsystem supports fully offline saved-frame replay. BEEP is required only at explicit hardware-validation gates; see the continuous-development procedure in `docs/EYES_ARCHITECTURE.md`.

## Further documentation

- [`docs/architecture.md`](docs/architecture.md) — onboard components, mapping layers, autonomy modes, and limits
- [`docs/bridge_api.md`](docs/bridge_api.md) — HTTP API and safety behavior
- [`docs/deployment.md`](docs/deployment.md) — deployment, systemd, Jupyter, SLAM reset/export, and verification
- [`docs/fair_remote_control_status.md`](docs/fair_remote_control_status.md) — current network topology and historical adapter notes
- [`docs/perception_baseline.md`](docs/perception_baseline.md) — measured camera, microphone, bridge-latency, and Pi-resource baseline
- [`docs/EYES_ARCHITECTURE.md`](docs/EYES_ARCHITECTURE.md) — multipanel Gemini eyes, typed proposals, offline workflow, and rollout gates
- [`docs/EMBODIED_MIDDLE_LAYER.md`](docs/EMBODIED_MIDDLE_LAYER.md) — persistent world, personality/drives, autonomous goals, and callable body skills
- [`assets/ros_maps/README.md`](assets/ros_maps/README.md) — retained Cartographer artifacts

## Version-control policy

Commit source, tests, stable documentation, configuration, systemd units, and selected useful map artifacts.

Do **not** commit credentials, `.hermes/`, logs, transient captures, generated runtime maps, notebook checkpoints, or large raw media. Scratch notebooks are not archaeological treasures.
