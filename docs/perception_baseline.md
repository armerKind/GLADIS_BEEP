# BEEP onboard perception baseline

Measured: 2026-07-21

> **Historical measurement:** this file preserves the original probe and its observed failures. It is not the current camera status. The MJPEG path was subsequently restored and used for real temporal Gemini captures. Repository source now includes bounded moving 4/6/9-frame buffering, although moving-gait image quality still awaits physical validation on `0.17.1-moving-eyes`.

Probe host: GLADIS over direct Tailscale

BEEP bridge: deployed `0.14.1-coverage-pose-cadence`

Probe script: [`../scripts/probe_bridge_perception.py`](../scripts/probe_bridge_perception.py)

## Scope and safety

This baseline covers bridge/API latency, camera availability, microphone capture, installed perception libraries, services, and Pi resource headroom. It is observation-only.

Before the canonical run, GLADIS called `/stop` and confirmed:

- `moving=false`;
- `last_command=sdk:stop`;
- LiDAR age `0.044 s`;
- guarded Cartographer pose valid;
- occupancy-map age `0.434 s`.

No movement, action, gait, or autonomous-exploration endpoint was invoked.

The final post-probe `/stop` also succeeded and reported `moving=false` with LiDAR age `0.013 s`. Cartographer had, however, accumulated impossible stationary raw motion: raw pose `(-1.8096, 0.9331, 2.7307)` while the guarded accepted pose remained near the origin. The pose guard recorded `994` stationary-drift rejections and set `pose_valid=false`. This is evidence that the guard contained a bad localization trajectory; SLAM-guided movement must remain disabled until localization is reset or repaired.

## Reproduce

The script authenticates to BEEP's existing Jupyter server for onboard inspection and separately measures bridge latency from the calling host. It aborts if `/status` reports movement.

```bash
export BEEP_HOST=beep.tailb08b32.ts.net
export BEEP_JUPYTER_PASSWORD='<from private environment>'
python3 scripts/probe_bridge_perception.py \
  --samples 3 \
  --mic-seconds 5 \
  --output /tmp/beep-perception-task1.json
```

`BEEP_JUPYTER_PASSWORD` is required and must remain in the private environment; the script never commits or prints it. Raw reports are intentionally transient under `/tmp`; this document retains the useful conclusions without publishing household audio or credentials.

## Network and bridge latency

A Tailscale ping immediately before probing reached BEEP directly at approximately `15 ms`. The bridge and Jupyter management paths were reachable.

Three-sample canonical bridge results:

| Endpoint | BEEP-local median attempt | GLADIS/Tailscale median attempt | Result |
|---|---:|---:|---|
| `/status` | `3.39 ms` | `23.72 ms` | Success |
| `/scan` | `4.46 ms` | `32.30 ms` | Success |
| `/local_map` | `4.99 ms` | `22.22 ms` | Success |
| `/frame.jpg` | `6599.13 ms` | `6645.11 ms` | HTTP 500 on every attempt |

The ordinary state APIs are already fast enough for compact perception metadata. The camera failure occurs onboard as well as remotely, so Tailscale is not the cause.

## Camera

### Hardware and formats

USB inventory contains:

```text
05a3:9230 ARC International Camera
```

Video nodes `/dev/video0`, `/dev/video1`, and `/dev/video10` through `/dev/video16` exist. `/dev/video0` is a V4L2 capture device and advertises:

- MJPEG:
  - `1920x1080` at `30 fps`;
  - `1280x720` up to `60 fps`;
  - `640x480` at multiple rates including approximately `30 fps`;
  - additional `1024x768`, `800x600`, `1280x1024`, and `320x240` modes.
- YUYV:
  - `640x480` at `30 fps`;
  - lower rates at larger resolutions.

`640x480` MJPEG is the recommended initial source for temporal buffering. The eventual sampler should start at a low application rate rather than attempting the camera's advertised maximum.

### Current blocker

The bridge is configured to fetch:

```text
http://192.168.8.88:6500/video_feed
```

No process currently listens on port `6500`. The vendor process:

```text
python3 /home/pi/DOGZILLA/app_dogzilla/app_dogzilla.py
```

holds `/dev/video0` as PID `2476`, so direct OpenCV and `v4l2-ctl` capture cannot open it. Raw V4L2 capture returns:

```text
VIDIOC_S_FMT: failed: Device or resource busy
VIDIOC_REQBUFS returned -1 (Device or resource busy)
```

Consequently `/frame.jpg` waits roughly `6.6 s` and returns HTTP 500. The camera hardware is present, but no usable frame source was available during this probe.

No process was killed and no camera or vendor service was restarted. The next camera task must first inspect the vendor application's video-server lifecycle and decide between:

1. restoring its port-6500 MJPEG output;
2. making the bridge the single `/dev/video0` owner; or
3. exposing frames from the existing vendor camera owner.

Running two independent camera owners is not viable.

## USB microphone

USB inventory contains:

```text
08bb:2902 Texas Instruments PCM2902 Audio Codec
```

ALSA identifies it as:

```text
card 2: Device [USB PnP Sound Device], device 0: USB Audio
capture path: plughw:2,0
```

The canonical five-second capture succeeded with:

| Property | Result |
|---|---:|
| Format | signed 16-bit little-endian PCM |
| Sample rate | `16000 Hz` |
| Channels | `1` |
| Frames | `80000` |
| WAV bytes | `160044` |
| RMS | `19` |
| Peak | `78` |

Repeated one-, two-, and five-second captures also succeeded. The signal was very quiet, consistent with ambient silence during the test. This proves Linux capture, but not speech sensitivity or an appropriate VAD threshold. A later calibration should record deliberate speech at several distances and measure noise floor, speech RMS, clipping, and usable gain.

## Onboard libraries

Available in BEEP's Jupyter Python 3.8 environment:

- OpenCV (`cv2`)
- Pillow (`PIL`)
- NumPy
- Requests
- SoundDevice

Not available in that environment:

- `webrtcvad`
- `websockets`
- `rclpy`

The missing Jupyter `rclpy` module does not mean the deployed bridge lacks ROS access; Cartographer, occupancy-grid, LiDAR, and bridge services were active. It only describes the Jupyter kernel environment used by this probe.

OpenCV/Pillow/NumPy are sufficient to prototype multipanel rendering without installing another image stack. Initial audio event detection can use a measured adaptive energy threshold; WebRTC VAD remains optional pending speech calibration.

## Pi resource baseline

The Pi reports:

| Resource | Result |
|---|---:|
| RAM total | `7.63 GiB` |
| RAM available | approximately `6.17 GiB` |
| Disk total under `/home/pi` | `58.22 GiB` |
| Disk used | `24.26 GiB` |
| Disk free | `31.67 GiB` |
| Post-probe CPU utilization over 2 s | `36.1%` in canonical run; `40.2%` in disk follow-up |
| Post-probe temperature | `67.2–67.7 °C` |
| Load average around probe | approximately `2.2–2.6` |

Active services:

- `beep-bridge`
- `beep-cartographer`
- `beep-occupancy-grid`
- `tailscaled`

Memory and disk headroom are ample. CPU and temperature are not alarming, but they are high enough that continuous full-rate image decoding would be irresponsible. The initial camera buffer should therefore:

- sample conservatively, initially around one frame per second;
- retain JPEG bytes rather than decoded arrays between requests;
- decode only selected frames when composing a panel;
- bound frame count and total bytes;
- expose render/sampler timing and temperature;
- remain isolated from LiDAR and movement-control locks.

## Task 1 conclusions

1. **Direct compact metadata is fast.** Bridge state reaches GLADIS in tens of milliseconds over Tailscale.
2. **The USB microphone works.** `plughw:2,0` reliably captures 16 kHz mono PCM; speech calibration is still required.
3. **The camera hardware exists but is not currently usable.** Yahboom's application owns `/dev/video0` while failing to expose its expected MJPEG listener.
4. **Multipanel rendering dependencies already exist.** OpenCV, Pillow, and NumPy are available onboard.
5. **Resources support a conservative temporal buffer.** Memory and disk are abundant; CPU and thermal load require bounded sampling and on-demand composition.
6. **No additional runtime layer is needed.** Once camera ownership is repaired, perception buffering and panel composition can remain modules of the existing bridge process.
7. **The active Cartographer trajectory became invalid while stationary.** Perception development can continue, but map-guided movement cannot use this trajectory.

## Next recommended task

Before implementing the rolling camera buffer, repair and verify a single stable frame source:

1. inspect why `app_dogzilla.py` owns `/dev/video0` without listening on `6500`;
2. restore or replace that source without affecting movement control;
3. capture and decode repeated `640x480` JPEG frames;
4. measure sustainable sampling latency and CPU/temperature;
5. only then add the bounded rolling frame buffer and temporal multipanel endpoint.

## Subsequent outcome

The recommendations above were completed in later increments:

- the vendor MJPEG stream became usable through the bridge;
- finite chronological capture, quality metrics, contact-sheet rendering, strict Gemini transport, replay, and persistent world updates were implemented;
- real stationary temporal captures and Gemini inference succeeded;
- `ContinuousMovingCapture` now stores JPEG evidence in a bounded ring, renders configurable 4/6/9-frame panels, and continues capture during one active inference;
- during a motion lease, bridge frame capture bypasses vendor app-control preparation so camera access cannot inject its stop command.

Still pending is the hardware benchmark originally implied by this baseline: measure sustainable FPS, blur, panel usefulness, Gemini stability, CPU/temperature, and cancellation behavior while BEEP physically walks. Historical numbers above remain valuable precisely because they are not being rewritten to flatter the present.
