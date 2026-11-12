# GLADIS_BEEP

Software, deployment helpers, and operational notes for **BEEP** — a Yahboom DogZilla S2 quadruped used as an embodied sibling-agent experiment for GLADIS.

BEEP combines the vendor motor SDK, 2D LiDAR, ROS 2, Cartographer, camera, and a local HTTP bridge. Jupyter maintenance helpers can also drive the Pi's analog speaker. Autonomous sensor polling and stop cleanup run on BEEP's Raspberry Pi; GLADIS sends high-level requests over the network.

## Current state

**New session entry point: [docs/HANDOFF.md](docs/HANDOFF.md).** It covers what is verified on hardware, what is ready for live wiring, and the next three engineering tasks.

`main` holds the **Stack 2** continuous autonomous embodiment architecture plus the hardware, ROS, vision and audio infrastructure. The earlier Stack 1 prototype (cap-search stop-think-act policy, `ClosedLoopExecutor`, bridge `IntentMailbox`, external room-camera observer, fair/presentation scripts and per-phase gate docs) is archived on branch `archive/stack1-cap-prototype`.

```text
bridge source    0.22.0-stack2-cutover   (not yet deployed)
bridge deployed  0.21.1-phase-c-audit    (stationary readback verified 2026-11-11)
```

Verified on hardware (stationary, November 11, 2026):

- onboard bridge with stop-priority lock split, ~10 Hz six-sector LiDAR guard, Cartographer 2D SLAM with healthy `SubmapList`/`/map`/TF cadence, and A* frontier planning
- direct MJPEG camera (port 6500, 640x480), moving-frame diversity ring, contact sheets, and live `gemini-3.5-flash-lite` structured scene perception
- USB microphone and analog speaker streams through PortAudio/ALSA (bind by card name; card numbers swap across boots)

Implemented and tested offline, not yet joined to live producers:

- Stack 2 spatial memory and attention in `beep_agent/`: event envelope and journal, map revision, pose history, place graph, episodes, tactical gate, strategic owner, visual grounding, viewpoint scoring, goal coordinator, atomic checkpoint, model session and observe-only supervisor
- audio contracts in `beep_audio/`: PCM capture, VAD, echo reference and barge-in, clock alignment, social interruption

Semantic perception never drives motors. `scripts/run_moving_eyes.py` updates a persistent `AgentSession` in shadow mode while the onboard bridge owns gait, collision safety, leases and stop. Older bridge releases and physical motion trials are recorded in git history.

## What “SLAM” means here

BEEP runs Cartographer on its Raspberry Pi. The vendor's Jupyter examples demonstrate those components; Jupyter is a browser interface to code executing on the Pi, not a remote SLAM computer.

The current robot does **not** expose trusted wheel odometry or a dependable ROS IMU input to Cartographer. LiDAR scan matching is therefore vulnerable to gait vibration, foot slip, scan distortion, and repetitive room geometry. The bridge keeps Cartographer's raw TF pose for diagnostics and exposes a guarded pose for navigation. Fresh local LiDAR remains authoritative for immediate collision safety.

This is genuine online mapping with pose-graph SLAM, but it is not a finished consumer-robot localization appliance. Marketing survived. Precision required engineering.

## Runtime architecture

```text
GLADIS / operator
      |
      |-- moving-eyes ring + Gemini scene perception (shadow)
      |-- Stack 2 agent: journal, spatial memory, tactical/strategic owners
      |-- typed skill adapter (explore / observe / stop)
      |
      | HTTP over DOGZILLA_WIFI or Tailscale
      v
BEEP bridge :8766 (Raspberry Pi)
      |-- DOGZILLALib motor SDK
      |-- ROS 2 /scan + fresh-LiDAR safety
      |-- Cartographer TF + /map monitoring
      |-- guarded pose and exploration controllers
      |-- asynchronous mission worker + immutable lease watchdog
      |-- Yahboom camera/app services

JupyterLab :8888 (Raspberry Pi)
      `-- deployment, diagnostics, SLAM maintenance, audio playback
```

Normal autonomy does not depend on an open Jupyter notebook or a responsive remote connection.

## Repository layout

```text
beep_bridge/       onboard HTTP bridge, frontier A* planner, progress monitor
beep_agent/        Stack 2 embodiment: events/journal, spatial memory, tactical and
                   strategic owners, grounding, coordinator, checkpoint, session
beep_eyes/         moving-frame capture ring, contact sheets, Gemini client, scene schema
beep_audio/        offline PCM capture, VAD, echo/barge-in, clock alignment, social turn-taking
config/            Cartographer configuration
systemd/           bridge, Cartographer, occupancy-grid and vendor-app units
scripts/           deployment, synchronization, SLAM, capture, perception and audio helpers
tests/             unit and offline integration tests for all of the above
docs/              handoff, architecture, bridge API and deployment
assets/ros_maps/   selected historical Cartographer artifacts
maps/              placeholder for ignored generated bridge maps
```

## Primary endpoints

```text
GET /health                     liveness plus nested runtime status snapshot
GET /stop                       repeated SDK stop attempts plus app-stop telemetry
GET /motion/profile             current gait, pace, body-height, shoulder and IMU profile
POST /motion/profile            stationary-only posture commissioning
POST /mission/start             begin asynchronous fluent autonomous navigation
GET /mission                    inspect active or latest mission without blocking
POST /mission/cancel            cancel the exact active autonomous mission
GET /lidar_walk                 local LiDAR-reactive fluent navigation
GET /frontier_explore           guarded-SLAM frontier planning/navigation
GET /coverage_explore           explore until guarded map growth plateaus
GET /action?name=stretch        named vendor preset
GET /mark_object?dry_run=1      preview approach → turn sideways → pee sequence
GET /frame.jpg                  camera frame
GET /map                        legacy bridge-map summary; `?full=1` returns sparse bridge trace
GET /slam_map                   authoritative Cartographer grid, origin, resolution, age, occupancy
GET /local_map                  robot-frame local LiDAR safety map
GET /map.svg                    legacy bridge-map rendering
```

See [`docs/bridge_api.md`](docs/bridge_api.md) for parameters and safety semantics.

## Useful scripts

```bash
scripts/check_bridge.sh                  # Check live bridge health
scripts/capture_bridge_state.sh          # Capture status/maps/frame locally
scripts/sync_from_pi.py                  # Pull bridge and selected Pi artifacts
scripts/jupyter_deploy_bridge.py         # Upload bridge/planner/config and restart
scripts/jupyter_reset_slam.py            # Start a clean Cartographer trajectory
scripts/jupyter_save_slam.py             # Finish and export pbstream/PGM/YAML
scripts/jupyter_play_audio.py FILE.mp3   # Upload and play through Pi analog audio
scripts/probe_bridge_perception.py       # Probe camera, microphone, API latency, and Pi resources
scripts/run_moving_eyes.py               # Continuous 4/6/9-frame moving vision → shadow AgentSession
scripts/run_beep_explore.py              # One bounded coverage mission + parallel moving eyes
scripts/run_beep_autonomy.py             # Offline observe-only supervisor replay
scripts/replay_goal_coordinator.py       # Offline goal-coordinator replay over recorded packets
scripts/install_systemd_service.py       # Install the bridge systemd service
scripts/save_slam_map.sh                 # Pi-side Cartographer exporter
```

Jupyter helpers default to `192.168.8.88`; set `BEEP_HOST=beep.tailb08b32.ts.net` for the Tailscale path. The source retains Yahboom's vendor-default Jupyter password for the stock image; use `BEEP_JUPYTER_PASSWORD` for any changed credential and never commit private alternatives or tokens.

## Networking

BEEP's intended dual-network arrangement is:

```text
wlan0  original DOGZILLA_WIFI access point / robot network
wlan1  external 2.4 GHz Wi-Fi uplink
```

Known addresses and names, exercised during development through 2026-08-03:

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
- Autonomous controllers require all six local LiDAR sectors and apply conservative full-body thresholds: 0.55 m front, 0.42 m front diagonals/turn sweep, 0.35 m sides, and 0.45 m rear. These are provisional safety floors pending measured-footprint calibration.
- A blocked front may trigger a short, continuously supervised reverse escape only when the full side envelope and rear clearance remain available. Reverse also aborts if average frontal clearance worsens by more than 0.02 m or heading drifts by more than 5 degrees. Lateral escape is disabled.
- Reverse/turn escapes must produce measured clearance or heading progress; ineffective escapes and more than two repeated identical escapes fail closed.
- Guarded SLAM is required for map-directed frontier/coverage claims.
- Occupancy-grid planning inflates obstacles by a conservative 0.35 m robot radius and rejects stale map/pose, impossible relocation, turn non-progress, and unsafe sweep clearance.
- Autonomous movement routines attempt final SDK stops on completion, timeout, and exception; a hardware/SDK failure can still prevent a physical stop.
- Every motion lease has an independent onboard deadline watchdog; it attempts repeated stops even if the owning worker or remote HTTP client stalls.
- Asynchronous missions keep `/status`, `/mission`, and cancellation responsive while the local controller maintains fluent gait.
- Moving vision captures into a bounded ring without touching app motor control; 4/6/9-frame panels tolerate individual blurred frames while locomotion continues.
- SDK stop success is authoritative; the optional Yahboom app-socket stop is best-effort telemetry.
- No battery telemetry is available; use charged batteries and supervised bounded runs.
- Do not infer “whole room” solely from elapsed time. Coverage mode uses map-growth saturation and still cannot prove access beneath or behind every obstacle.

## Development

Run validation from the repository root:

```bash
PYTHONPATH="$PWD" python3 -m unittest discover -s tests -p 'test_*.py'
python3 -m compileall -q beep_agent beep_audio beep_bridge beep_eyes scripts tests
git diff --check
```

Behind an HTTP proxy, set `NO_PROXY=127.0.0.1,localhost` so the loopback bridge tests reach their local servers. Vision and the Stack 2 modules run fully offline against saved frames and fixtures; BEEP is needed only for hardware validation.

## Further documentation

- [`docs/HANDOFF.md`](docs/HANDOFF.md) — one-page onboarding: cutover, verified hardware, Stack 2 modules, next tasks
- [`docs/AUTONOMOUS_EMBODIMENT_ARCHITECTURE.md`](docs/AUTONOMOUS_EMBODIMENT_ARCHITECTURE.md) — canonical system design: tactical attention, visual grounding, memory, motor transport, audio, and hardware gates
- [`docs/bridge_api.md`](docs/bridge_api.md) — HTTP API and safety behavior
- [`docs/deployment.md`](docs/deployment.md) — deployment, systemd, Jupyter, SLAM reset/export, and verification
- [`assets/ros_maps/README.md`](assets/ros_maps/README.md) — retained Cartographer artifacts

## Version-control policy

Commit source, tests, stable documentation, configuration, systemd units, and selected useful map artifacts.

Do **not** commit credentials, `.hermes/`, logs, transient captures, generated runtime maps, notebook checkpoints, or large raw media. Scratch notebooks are not archaeological treasures.
