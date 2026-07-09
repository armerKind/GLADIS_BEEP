# GLADIS_BEEP

Code, notes, and deployment helpers for **BEEP** — the DogZilla S2 quadruped used as GLADIS's embodied robot-dog sibling experiment.

## Current focus

The first version-controlled component is the BEEP bridge:

```text
beep_bridge/beep_bridge.py
```

The bridge provides a lightweight HTTP nervous system over the Yahboom DogZilla stack:

- SDK-backed locomotion via `DOGZILLALib`
- LiDAR ingestion from ROS2 `/scan`
- camera frame capture from Yahboom MJPEG port `6500`
- local robot-frame obstacle map
- scan-matched global occupancy-map experiment
- bounded movement endpoints with stop bursts

## Current bridge version

```text
0.7.1-scanmatch-localmap
```

Important: this is not full SLAM yet. It uses command dead reckoning corrected by local LiDAR scan matching. The local robot-frame map is considered more trustworthy for immediate safety decisions than the global map.

## Repository layout

```text
beep_bridge/      Bridge source code for the Pi
scripts/          Deployment/sync/helper scripts
docs/             API and architecture notes
notebooks/        Optional exploratory notebooks; keep only useful ones
assets/           Example generated maps/screenshots and selected ROS map artifacts
maps/             Placeholder only; generated maps are ignored by git
```

## Useful scripts

```bash
scripts/check_bridge.sh              # Check live bridge status
scripts/capture_bridge_state.sh      # Capture status/maps/frame into captures/
scripts/sync_from_pi.py              # Pull bridge + selected Pi artifacts into repo
scripts/jupyter_deploy_bridge.py     # Upload/restart bridge through Jupyter
```

## Robot network assumptions

When powered and reachable, BEEP is expected at:

```text
192.168.8.88
```

Useful ports:

```text
8766  BEEP bridge HTTP API
8888  JupyterLab on Pi
6000  Yahboom app-control server
6500  Yahboom MJPEG camera stream
22    SSH if available
```

## Version-control policy

Commit source and stable documentation.

Do **not** commit:

- bridge logs
- generated map dumps
- notebook checkpoints
- credentials / tokens
- large raw video/audio captures

Notebook policy: keep notebooks only if they contain reusable experiments or documentation. Throw away scratch notebooks. Humanity has suffered enough.
