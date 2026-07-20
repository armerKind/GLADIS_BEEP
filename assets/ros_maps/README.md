# ROS / Cartographer map artifacts

Selected files exported from BEEP's Pi directory:

```text
/home/pi/gladis_maps
```

Formats:

- `.pbstream` — serialized Cartographer trajectory/submap state
- `.pgm` — rendered occupancy-grid image
- `.yaml` — map metadata and resolution
- `inventory.json` — synchronization inventory from `scripts/sync_from_pi.py`

The top-level `barf_*` and `gladis_room_*` artifacts are historical experiments from before the project was renamed BEEP. They are retained for regression/comparison, not as trustworthy navigation maps.

`current_slam_tuned/` contains a selected artifact generated with the later Pi-safe Cartographer configuration. It is still an experimental map: BEEP has no trusted wheel odometry or integrated ROS IMU, so every map must be inspected for scan smearing, false translation, and loop-closure errors before reuse.

Runtime exports are not committed automatically. Save them on BEEP with:

```bash
BEEP_MAP_BASE=/home/pi/gladis_maps/<name> python3 scripts/jupyter_save_slam.py
```

Then selectively pull useful artifacts with `scripts/sync_from_pi.py`. PID files, logs, volatile `current_*` markers, and low-value test maps should remain untracked. Map hoarding is not localization.
