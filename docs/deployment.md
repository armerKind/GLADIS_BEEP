# BEEP Deployment and Operations

## Pi paths

```text
/home/pi/beep_bridge/beep_bridge.py
/home/pi/beep_bridge/frontier_planner.py
/home/pi/cartographer_ws2/install/cartographer_ros/share/cartographer_ros/configuration_files/beep_2d.lua
/home/pi/gladis_maps/
```

## Runtime services

BEEP normally runs:

```text
beep-bridge.service
beep-cartographer.service
beep-occupancy-grid.service
YahboomStart.service
tailscaled.service
```

The repository units are in `systemd/`. The bridge service sources ROS 2 Foxy and the Cartographer overlay, uses `ROS_DOMAIN_ID=16`, selects the SDK motor backend, and restarts automatically.

`YahboomStart.service` is the authoritative vendor launch for the MS200 LiDAR on `/dev/ttyAMA1`. Keep the redundant `XGO_Start.service` disabled: both vendor units launch `oradar_scan`, and two publishers competing for the same serial port can leave `/scan` advertised but silent.

## Jupyter is an onboard maintenance path

JupyterLab listens on port `8888` on BEEP. The browser may run on GLADIS, but notebook kernels and commands execute on BEEP's Raspberry Pi. The helpers use Jupyter because it has sometimes remained easier to authorize than Tailscale SSH.

Host-side Jupyter helpers require Python packages `requests` and `websockets`.

Set the destination explicitly when using Tailscale:

```bash
export BEEP_HOST=beep.tailb08b32.ts.net
```

Local DogZilla access uses the default `192.168.8.88`.

The scripts retain Yahboom's stock-image password as their vendor default. Override changed credentials with `BEEP_JUPYTER_PASSWORD`; do not commit private alternatives, tokens, or generated login material.

Implemented source overrides:

```text
BEEP_BRIDGE_SRC
BEEP_PLANNER_SRC
BEEP_SLAM_CONFIG_SRC
BEEP_SAVE_SLAM_SRC
```

## Deploy bridge, planner, Cartographer config, and map exporter

From the repository root:

```bash
python3 scripts/jupyter_deploy_bridge.py
```

The deployment helper:

1. logs into BEEP's Jupyter server,
2. backs up the deployed bridge,
3. uploads `beep_bridge.py` and `frontier_planner.py`,
4. uploads `config/beep_2d.lua`,
5. uploads and makes executable `scripts/save_slam_map.sh`,
6. compiles the Python source on BEEP,
7. restarts `beep-bridge.service`,
8. verifies that the service is active.

Verify afterward:

```bash
curl -s http://beep.tailb08b32.ts.net:8766/config | python3 -m json.tool
curl -s http://beep.tailb08b32.ts.net:8766/health | python3 -m json.tool
curl -s http://beep.tailb08b32.ts.net:8766/stop | python3 -m json.tool
```

## Install the bridge systemd unit

```bash
python3 scripts/install_systemd_service.py
```

This installer currently targets `beep-bridge.service`. Cartographer and occupancy-grid units are prerequisites and must be installed separately from:

```text
systemd/beep-cartographer.service
systemd/beep-occupancy-grid.service
```

With working SSH, one reproducible installation is:

```bash
scp systemd/beep-cartographer.service systemd/beep-occupancy-grid.service pi@beep:/tmp/
ssh pi@beep 'sudo cp /tmp/beep-cartographer.service /tmp/beep-occupancy-grid.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl enable --now beep-cartographer beep-occupancy-grid'
```

Verify on BEEP:

```bash
systemctl is-active beep-bridge beep-cartographer beep-occupancy-grid
```

## Cartographer lifecycle

The active configuration is `config/beep_2d.lua`:

- one 2D LiDAR scan stream,
- no wheel odometry,
- no IMU data,
- 3.5 m maximum range,
- online correlative scan matching,
- stronger translation/rotation penalties,
- 5 s stationary motion-filter interval,
- sparse pose-graph optimization and constraints.

### Start a clean trajectory

Use this after moving BEEP to a different room or after raw localization/map corruption:

```bash
python3 scripts/jupyter_reset_slam.py
```

The helper stops the duplicate `XGO_Start` launch, restarts the authoritative `YahboomStart` LiDAR publisher, then restarts Cartographer, occupancy-grid publication, and the bridge. It discards the active unsaved trajectory. Verify numeric LiDAR sectors, a fresh guarded pose near the new origin, and a fresh non-empty map before autonomous movement.

### Pose freshness nuance

While stationary, Cartographer's tuned motion filter may publish accepted pose updates only about every five seconds. The generic `slam.active` field can therefore flicker false between updates even when the map remains fresh. Map-directed controllers use mode-appropriate freshness gates:

- frontier exploration expects tightly fresh pose updates,
- coverage exploration accepts the intentional stationary cadence but still requires a valid guarded pose and fresh map,
- movement always requires fresh local LiDAR.

### Save an active trajectory

```bash
BEEP_MAP_BASE=/home/pi/gladis_maps/living_room \
python3 scripts/jupyter_save_slam.py
```

This calls `/home/pi/beep_bridge/save_slam_map.sh` (uploaded by `jupyter_deploy_bridge.py`), finishes trajectory `0`, and exports:

```text
<base>.pbstream
<base>.pgm
<base>.yaml
```

Finishing the trajectory stops live mapping. Restart/reset Cartographer before a later mapping run. Loading a saved state in production pure-localization mode is not yet automated.

## Exploration workflow

### 1. Preflight

```bash
curl -s http://beep.tailb08b32.ts.net:8766/stop
curl -s http://beep.tailb08b32.ts.net:8766/health | python3 -m json.tool
```

Require:

- `moving: false`,
- fresh `scan_age_s`,
- safe initial sectors,
- fresh map where required,
- `slam.pose_valid: true` for map-directed modes.

### 2. Dry-run a frontier plan

```bash
curl 'http://beep.tailb08b32.ts.net:8766/frontier_explore?name=room&max_duration=90&chaos=0.35&dry_run=1'
```

A dry run must not move the robot. Frontier completion is credible only when the trace shows physical route progress; turns-only traces can lose frontiers as the live map changes.

### 3. Coverage-driven room exploration

```bash
curl 'http://beep.tailb08b32.ts.net:8766/coverage_explore?max_duration=600&min_duration=60&coverage_window=45&min_growth_cells=150'
```

Coverage mode uses fluent local LiDAR motion while monitoring guarded-SLAM known-cell growth. `coverage_plateau` means map growth stayed below the configured threshold for the complete window. The maximum duration is a safety ceiling, not the completion objective.

### 4. Local fallback

```bash
curl 'http://beep.tailb08b32.ts.net:8766/lidar_walk?max_duration=90'
```

Use this when global SLAM is rejected. It is explicitly local reactive navigation and must not be described as globally mapped coverage.

## Audio playback

Generate or provide an audio file, then play it through the Pi analog output:

```bash
python3 scripts/jupyter_play_audio.py message.mp3 --gain 3
```

The Pi requires `ffmpeg`, `aplay`, and a working `plughw:1,0` device. The helper creates `/home/pi/beep_bridge/audio/` through Jupyter before uploading the file.

Working device:

```text
plughw:1,0
```

The helper converts to 24 kHz mono WAV and applies 3× software gain. Higher gain can distort. Playback requires Jupyter/network reachability; successful local TTS generation alone does not prove speaker playback.

## Sync and captures

```bash
python3 scripts/sync_from_pi.py
scripts/capture_bridge_state.sh
```

- Runtime captures go under ignored `captures/`.
- Selected `.pbstream`, `.pgm`, and `.yaml` files may be retained under `assets/ros_maps/`.
- Do not overwrite newer local source with a stale Pi copy without reviewing the diff.

## Movement calibration

The app-port `forward` primitive is not dependable for gait locomotion; use `DOGZILLALib`.

Current controller values:

```text
primitive default step       10
fluent forward VX            20
fluent arc yaw VYAW          30
SDK move_x maximum           20
SDK move_y maximum           18
SDK nonzero turn minimum     30
```

Frontier in-place turns use step 30 and calibrated longer durations. A controlled 1.3 s right turn produced meaningful physical rotation. Higher arc yaw values are not yet validated.

## Networking and boot recovery

See [`fair_remote_control_status.md`](fair_remote_control_status.md) for the last verified Tailscale addresses, adapter history, and enrollment state. Initial enrollment uses:

```bash
sudo tailscale up --ssh --hostname beep
```

Expected topology:

```text
wlan0  DOGZILLA_WIFI / 192.168.8.88
wlan1  RTL8188EUS external 2.4 GHz uplink
```

If a cold boot fails with the nano adapter inserted:

1. power off,
2. remove the nano adapter,
3. wait about ten seconds,
4. boot and wait for the normal stretch plus `DOGZILLA_WIFI`,
5. insert the nano adapter,
6. wait for uplink and Tailscale.

## Failure handling

- If LiDAR is stale, do not move.
- If Cartographer jumps, reset or use explicitly local navigation; do not relabel dead reckoning as SLAM.
- If Jupyter and Tailscale disappear, try the direct DogZilla network only when physically in range.
- If all paths fail, report the action as not executed.
- Every autonomous request should be followed by verification of `moving=false`; redundant `/stop` is intentional.
- No battery telemetry exists.
