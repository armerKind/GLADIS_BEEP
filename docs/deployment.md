# BEEP Bridge Deployment Notes

## Current Pi path

```text
/home/pi/beep_bridge/beep_bridge.py
```

## Runtime environment

The bridge should be launched with ROS2 and the Yahboom/Cartographer workspace sourced:

```bash
source /opt/ros/foxy/setup.bash
source /home/pi/cartographer_ws2/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=16
export ROS_LOCALHOST_ONLY=0
export BEEP_APP_HOST=192.168.8.88
export BEEP_CAMERA_URL=http://192.168.8.88:6500/video_feed
export BEEP_MOTOR_BACKEND=sdk
export BEEP_SDK_STEP=10
export BEEP_SDK_GAIT=walk
export BEEP_SDK_PACE=slow
export BEEP_MAP_DIR=/home/pi/beep_bridge/maps
python3 /home/pi/beep_bridge/beep_bridge.py
```

## Restart pattern on Pi

```bash
cd /home/pi/beep_bridge
if [ -f bridge.pid ]; then
  kill "$(cat bridge.pid)" 2>/dev/null || true
fi
mkdir -p logs maps
nohup bash -lc '
  source /opt/ros/foxy/setup.bash
  source /home/pi/cartographer_ws2/install/setup.bash 2>/dev/null || true
  export ROS_DOMAIN_ID=16 ROS_LOCALHOST_ONLY=0
  export BEEP_APP_HOST=192.168.8.88
  export BEEP_CAMERA_URL=http://192.168.8.88:6500/video_feed
  export BEEP_MOTOR_BACKEND=sdk
  export BEEP_SDK_STEP=10
  export BEEP_SDK_GAIT=walk
  export BEEP_SDK_PACE=slow
  export BEEP_MAP_DIR=/home/pi/beep_bridge/maps
  exec python3 /home/pi/beep_bridge/beep_bridge.py
' > logs/bridge.log 2>&1 & echo $! > bridge.pid
```

## Scripted deployment

From the repository root, if Jupyter is reachable on port `8888`:

```bash
python3 scripts/jupyter_deploy_bridge.py
```

Useful environment overrides:

```bash
BEEP_HOST=192.168.8.88
BEEP_JUPYTER_PASSWORD=yahboom
BEEP_BRIDGE_SRC=beep_bridge/beep_bridge.py
```

## Sync from Pi

Pull live bridge source and useful artifacts:

```bash
python3 scripts/sync_from_pi.py
```

Runtime captures go under `captures/` and are ignored by git. Selected ROS/Cartographer map artifacts go under `assets/ros_maps/` and can be committed when useful.

## Systemd service

The repo contains a Pi systemd unit:

```text
systemd/beep-bridge.service
```

Install/enable/restart it through Jupyter:

```bash
python3 scripts/install_systemd_service.py
```

After install, verify:

```bash
systemctl status beep-bridge.service
curl -s http://192.168.8.88:8766/config | python3 -m json.tool
```

The service is designed to restart automatically after crashes and boot automatically after Pi reboot. A tiny act of civilization.

## Verification

```bash
curl -s http://192.168.8.88:8766/config | python3 -m json.tool
curl -s http://192.168.8.88:8766/status | python3 -m json.tool
curl -s http://192.168.8.88:8766/local_map | python3 -m json.tool
curl -s http://192.168.8.88:8766/stop
```

## Known movement calibration

The app-port `forward` primitive is not reliable for locomotion. Use the SDK backend.

Current healthier baseline:

```text
BEEP_SDK_STEP=10
BEEP_SDK_PACE=slow
BEEP_SDK_GAIT=walk
```

The SDK itself clamps:

```text
move_x max: 20
move_y max: 18
turn min:   30
```

So avoid high step values. They make BEEP look like a filing cabinet discovering knees.
