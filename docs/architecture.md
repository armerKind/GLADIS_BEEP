# Architecture Notes

## Goal

BEEP should become an embodied sibling-agent to GLADIS: a robot body that can perceive, speak, move, map, and act with increasing autonomy.

The bridge is the current nervous system. It deliberately avoids browser/Jupyter control loops.

## Current architecture

```text
GLADIS / host
   |
   | HTTP
   v
BEEP bridge :8766 on Pi
   |-- DOGZILLALib SDK for locomotion
   |-- ROS2 /scan for LiDAR
   |-- Yahboom port :6500 for camera MJPEG
   |-- Yahboom port :6000 for camera/app Standard mode
   |-- local + global mapping experiments
```

## Mapping model

There are two map layers:

### Local map

Robot-frame LiDAR map around BEEP.

This is the trustworthy layer for immediate obstacle avoidance.

```text
GET /local_map
GET /local_map.svg
```

### Global map

Sparse occupancy grid over an 8m x 8m area with 5cm cells.

This now uses local scan matching to correct dead-reckoned pose, but it is still not full SLAM.

```text
GET /map
GET /map.svg
```

## Pose model

Current pose source:

```text
dead_reckoning + local_scan_matching
```

Dead reckoning provides a movement prior. LiDAR scan matching corrects local pose drift where possible.

Reported fields:

```json
{
  "x": 0.0,
  "y": 0.0,
  "yaw": 0.0,
  "source": "dead_reckoning+scan_match",
  "confidence": 0.9,
  "scan_match_score": 0.0124
}
```

## Limitations

- No loop closure yet.
- No persistent semantic object map yet.
- No reliable long-range path planning yet.
- Local map is better than global map for reflex decisions.
- Camera quality depends heavily on lighting.
- Battery loss can drop all ports simultaneously.

## Next useful upgrades

1. Persist bridge as a systemd service.
2. Add a real sync/deploy workflow from this repo to the Pi.
3. Add map-aware escape maneuvers using `/local_map`.
4. Add voice endpoints for speaker/microphone.
5. Add semantic object memory: bin, door, couch, charging spot.
6. Add a simple behavior layer: observe → decide → bounded action → verify.
7. Later: integrate proper SLAM / loop closure if available.
