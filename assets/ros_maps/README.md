# ROS / Cartographer map artifacts

These files were pulled from BEEP's Pi directory:

```text
/home/pi/gladis_maps
```

They are historical mapping artifacts from early GLADIS/BEEP experiments. They are kept for comparison against the bridge's newer local-map and scan-matched occupancy-grid approach.

Notes:

- `barf_*` names predate the project rename from BARF to BEEP.
- `.yaml` + `.pgm` are occupancy-grid exports.
- `.pbstream` files are Cartographer state snapshots.
- PID files, logs, and volatile `current_*` markers are intentionally not stored here.
