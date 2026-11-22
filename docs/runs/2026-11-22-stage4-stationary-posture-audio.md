# 2026-11-22 stationary posture and audio commissioning

Owner authorization: BEEP off charger, clear floor, owner beside robot. **No locomotion or autonomous exploration authorized or attempted.** Source head at start: `1e07c4a`.

## Connectivity and preflight

- Tailscale initially reported BEEP offline. Via saved backup DOGZILLA_WIFI link, inspected the Pi: AIDA was associated without IPv4/default route. Reconnected AIDA using NetworkManager, without rebooting. Restored the host's AIDA connection and verified Tailscale ping, bridge HTTP, and Jupyter HTTP over `100.100.141.33`.
- Deployed `0.22.12-stationary-social-posture` via `scripts/jupyter_deploy_bridge.py` over Tailscale; bridge `/status` and `/config` read back that version and configurable sit/stand action IDs 12/255. These IDs were source-derived, not independently SDK-hardware-certified.
- Issued `/stop` first. Readback: `moving=false`, motion lease null, standing, mission null; LiDAR scan age ~0.02 s. SLAM usable was false, but no walking was requested.

## One supervised sit / frame / stand

- Captured one 640×480 standing baseline. POST `/posture` sit returned success; readback sitting, acknowledged true, `moving=false`, motion lease null. Captured one 640×480 sitting frame. POST `/posture` stand returned success; readback standing, acknowledged true, `moving=false`, motion lease null. The final status remained standing/unowned, mission null.
- The owner visually confirmed **both physical postures were stable**. This validates a single supervised stationary sit-and-stand cycle, not broader motion safety.
- Both frames were very dark with a bright reddish lamp; visual comparison did **not** establish a reliable face-level upward camera view or a usable person description. No identity claim or `upward_social_turn()` greeting was made. Local images are retained only in the profile scratch directory for this review.

## Separate stationary audio checks

- ALSA name-based discovery: analog `Headphones` speaker was card 1; `USB PnP Sound Device` mic was card 2 **on this boot**. Speaker tone returned successful `aplay`, but the owner heard nothing at a deliberately low level. At 3× software gain and the initial 85% ALSA Headphone setting it was faint; at 100% Headphone, the owner heard it clearly. This mixer setting was left at 100%; do not hard-code the card number on later boots.
- USB mic captured 16 kHz mono PCM. In the cued spoken probe, half-second RMS rose from ambient ~16–21 to 32–36, peak 209. This is a measurable input response, **not an intelligibility or ASR verification**. Both raw mic recordings were deleted from the Pi after measurement.
- The uncommissioned live `run_social_voice.py` path was **not run**: `OPENAI_API_KEY` was not configured on the Pi or host; the script also requires a separate deployment and calibrated VAD. Its current default onset RMS 850 is far above the measured speech-probe peak 209, so an unmodified live run would not recognize that sample. Cloud mic/ASR/reply/TTS and persistent memory remain untested; no private recording was sent to a provider.

## Limits and next gates

- Improve illumination/position and recheck upward view only in a separately supervised posture trial. Do not claim person recognition from the current dark frames.
- Calibrate mic gain and VAD with intelligibility checks before a consented live voice turn. Provision cloud credentials outside chat if that adapter is used. Keep voice separate from motor authority and do not start a coverage walk based on this stationary trial.
