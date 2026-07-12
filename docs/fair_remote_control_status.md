# Fair Remote Control Status

Updated after live setup attempt on BEEP.

## What is working

### Bridge auto-start

`beep-bridge.service` is installed on the Pi, enabled, and running.

Verified from host:

```text
http://192.168.8.88:8766/config
```

Current bridge:

```text
version: 0.7.1-scanmatch-localmap
motor_backend: sdk
sdk_step_default: 10
sdk_pace: slow
local_map: true
scan_seen: true
```

### Tailscale installed

Tailscale package `1.86.2` was installed offline on BEEP.

Verified on Pi:

```text
tailscale version: 1.86.2
tailscaled: enabled, active
tailscale status: Logged out
```

BEEP is ready to join Tailscale once it has an internet uplink.

## Current blocker

BEEP does **not** currently have a working internet uplink.

The second USB WiFi dongle is an AIC/UGREEN style adapter:

```text
storage mode: a69c:5723 aicsemi Aic MSC
mode-switched: a69c:8d80 AIC Wlan
expected WLAN mode: 368b:8d83 AICSemi AIC 8800D80
```

What was attempted:

- Installed `dkms` and `dctrl-tools` offline.
- Built and installed AIC8800 kernel modules:
  - `aic_load_fw`
  - `aic8800_fdrv`
- Patched DKMS build to pass `KBUILD_EXTRA_SYMBOLS` so cross-module symbols resolve.
- Loaded both modules successfully.
- Mode-switched the dongle with:

```bash
sudo usb_modeswitch -KQ -v a69c -p 5723
```

Result:

```text
a69c:5723 storage mode -> a69c:8d80 ejected/AIC Wlan mode
```

But firmware upload fails:

```text
fmacfw_8800d80_u02.bin upload
cmd timed-out
bin upload fail: err:-32
no wlan1 created
```

Conclusion: this AIC/UGREEN dongle is **not fair-safe** unless more driver debugging is done. Do not rely on it for next week.

## Cleanup performed

The failed AIC driver experiment was removed from BEEP after verification:

```text
AIC kernel modules: absent
AIC DKMS entry: absent
/usr/src/aic8800-radxa: removed
/var/lib/dkms/aic8800: removed
/lib/firmware/aic8800D80: removed
/home/pi/aic8800_bundle*: removed
/etc/usb_modeswitch.d/a69c:5723: removed
dkms/dctrl-tools packages: purged
tailscale package: retained
```

Bridge and Tailscale remained active after cleanup:

```text
beep-bridge: active
tailscaled: active
scan_seen: true
scan_age_s: <0.1s during verification
```

Note: `apt autoremove` removed ROS GUI/demo packages such as `rviz2`, `rqt`, `turtlesim`, and ROS examples. The bridge and live LiDAR `/scan` remained functional. Do not reinstall demo/GUI packages for the fair unless a concrete workflow needs them.

## Recommended fair path

Use one of these instead:

### Option A: Phone USB tethering

Preferred if the phone supports it.

Expected result on BEEP:

```text
usb0 or enx... interface appears
DHCP gives internet
Tailscale can log in
```

Advantages:

- no WiFi driver lottery,
- usually more reliable than cheap USB WiFi dongles,
- one cable also helps keep the phone powered if connected through a hub/power setup.

### Option B: Known-supported USB WiFi dongle

Use a dongle supported by the Pi kernel without vendor-driver surgery.

Prefer chipsets known to work with Linux/Raspberry Pi out of the box, e.g. common MediaTek or Realtek IDs already supported by kernel modules.

Avoid AIC8800D80 for the fair unless it is proven stable after reboot.

## Next setup once internet uplink exists

Run on BEEP:

```bash
sudo tailscale up --ssh --hostname beep
```

Authorize the login URL in Max's browser.

Then verify from GLADIS host:

```bash
tailscale status | grep beep
curl -s http://<beep-tailscale-ip>:8766/config | python3 -m json.tool
curl -s http://<beep-tailscale-ip>:8766/status | python3 -m json.tool
curl -s http://<beep-tailscale-ip>:8766/frame.jpg -o /tmp/beep-frame.jpg
```

## Fair safety still needed

Before doing remote movement over Tailscale, add bridge-side fair controls:

- movement auth token,
- fair-mode duration clamp,
- reject movement if LiDAR scan is stale,
- reject forward if front sector is too close,
- keep `/stop` fast and tested.

Remote reachability first. Remote movement second. Nobody gets sued by a status endpoint.
