# Remote Control and Network Status

This file began as fair-preparation notes. It now records the current dual-network setup and preserves the failed adapter experiment only as historical context.

The dual-network topology and cold-boot workaround were established on **2026-07-19**; direct DOGZILLA_WIFI and Tailscale paths were exercised again during development through **2026-08-03**. Reachability is still runtime state and must be checked rather than inferred from this document.

## Current working topology

BEEP preserves the vendor robot network while using a second adapter for internet/Tailscale:

```text
wlan0  DOGZILLA_WIFI access point and 192.168.8.88 robot address
wlan1  Realtek RTL8188EUS 2.4 GHz uplink
```

The uplink has been verified on the `AIDA` Wi-Fi network. The original `DOGZILLA_WIFI` access point remains available and must not be replaced by uplink configuration.

## Current remote path

Tailscale has been authenticated and exercised independently of the DogZilla access point:

```text
BEEP hostname:       beep
BEEP MagicDNS:       beep.tailb08b32.ts.net
BEEP address:        100.100.141.33
GLADIS hostname:     gladis
GLADIS address:      100.79.187.106
```

Verified services have included:

- bridge HTTP on `8766`,
- JupyterLab on `8888`,
- Tailscale SSH when authorized,
- camera/app services on the robot network,
- local LiDAR and SDK motor control.

Tailscale may use DERP relays with variable latency. It is a supervisory/control path, not the safety loop.

## Current bridge and services

Last physically deployed and verified bridge line:

```text
0.16.3-normal-pace
```

Repository source is now `0.17.1-moving-eyes`; its asynchronous mission and moving-camera additions await the next powered deployment/validation gate.

Pi services observed active during that verification:

```text
beep-bridge.service
beep-cartographer.service
beep-occupancy-grid.service
YahboomStart.service
tailscaled.service
```

`YahboomStart.service` is the sole intended MS200 LiDAR owner. Keep duplicate `XGO_Start.service` disabled and inactive; competing `oradar_scan` processes can advertise `/scan` while producing no usable data.

LiDAR-aware autonomous controllers reject stale scans, use local obstacle thresholds, guard Cartographer pose where required, watch turn progress, bound runs, and attempt SDK stop cleanup. Generic `/move` and vendor `/action` bypass those autonomy gates. At least one successful SDK stop attempt is authoritative for reporting; optional Yahboom app-socket stop failure is diagnostic.

Movement authentication is **not** implemented. Keep bridge access on trusted local/Tailscale networks.

Initial or replacement Tailscale enrollment on BEEP is:

```bash
sudo tailscale up --ssh --hostname beep
```

Authorize the generated login URL, then verify `tailscale ping beep` and bridge `/health` from GLADIS. Do not expose port `8766` beyond trusted local/Tailscale peers until movement authentication exists.

## Cold-boot caveat

The RTL8188EUS nano adapter can interfere with reliable cold boot. The known recovery/startup sequence is:

1. Power BEEP off.
2. Remove the nano Wi-Fi adapter.
3. Wait about ten seconds.
4. Power BEEP on.
5. Wait for the normal stretch and `DOGZILLA_WIFI` access point.
6. Insert the nano adapter.
7. Wait for the 2.4 GHz uplink and Tailscale.

Keep the phone hotspot or access point nearby, enabled, and in 2.4 GHz/compatibility mode when it is the uplink.

## Verification

From GLADIS over Tailscale:

```bash
tailscale ping beep
curl -s http://beep.tailb08b32.ts.net:8766/health | python3 -m json.tool
curl -s http://beep.tailb08b32.ts.net:8766/stop | python3 -m json.tool
curl -s http://beep.tailb08b32.ts.net:8766/mission | python3 -m json.tool
curl -s http://beep.tailb08b32.ts.net:8766/frame.jpg -o /tmp/beep-frame.jpg
```

From a machine connected directly to `DOGZILLA_WIFI`, replace the hostname with `192.168.8.88`.

If bridge, Jupyter, SSH, Tailscale, and the local DogZilla address are all unreachable, do not claim an action succeeded. Check power, battery, uplink placement, and the nano-adapter boot sequence.

## Historical AIC8800 adapter experiment

An earlier AIC/UGREEN-style USB adapter appeared as storage ID `a69c:5723`, mode-switched to `a69c:8d80`, but failed firmware upload with command timeouts. DKMS modules and experimental firmware were removed after testing. That adapter should not be used for demonstrations unless independently proven stable.

The working replacement is the Realtek RTL8188EUS adapter. Retaining this note prevents future archaeology involving the same unsuitable dongle, a service for which civilization should be grateful.

## Safety boundary

- Keep `/stop` reachable before movement.
- For asynchronous source `0.17.1-moving-eyes`, keep `/mission` reachable and test exact-ID cancellation before a long run.
- Do not depend on remote round-trip latency for obstacle stopping.
- Confirm fresh LiDAR and `moving=false` before autonomous runs.
- Use guarded-SLAM modes only when pose/map freshness and validity pass.
- Use `/lidar_walk` explicitly as local reactive navigation when SLAM is rejected.
- No battery telemetry exists; supervise long runs physically.
