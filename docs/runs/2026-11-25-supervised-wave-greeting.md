# Task 4: supervised wave_hand + direct greeting, stationary

Date: 2026-11-25 (host CET; BEEP clock UTC+08). Operator Max nearby. This is **not** an automatic LifeLoop commissioning result. No patrol or other locomotion was dispatched.

## Deployment and preflight

- Host source commit before this run: `25d8330`; Tailscale maintenance access reauthorized by the user. Jupyter port 8888 was unavailable over the tailnet, so the bridge source was transferred by Tailscale SSH. The staged remote source SHA-256 matched the host source (`9d07184f140276e6fc313a0fcf09a86b5706bc0a3861fce1119e2332a7def6f7`); remote Python compiled, source was backed up, and the service was restarted while stopped/unowned. Host readback confirmed `0.22.15-gesture-posture-fence-offline` with `posture_uncertain=true` on startup.
- Initial post-restart safety gate **failed**: no fresh LiDAR scans, SLAM unusable. No motion was sent. Occupancy publication was stopped for diagnosis; the bridge still had no scans. The ROS graph reported a LiDAR publisher, but a timed sample probe produced no samples. With the nearby operator's fresh clear-floor/upright confirmation, the authoritative `YahboomStart` launch was restarted once, after stopped/no-lease preflight; scans resumed. Cartographer, occupancy publication and bridge were then restarted in dependency order. Cause of the silent scan publisher and oversized prior map is not established.
- Final no-motion gate: bridge `0.22.15`, `moving=false`, no motion lease/mission, posture uncertain true, six fresh numeric LiDAR sectors (scan age 0.014 s at the sampled gate), verified map revision 0 / epoch 0, fresh guarded pose and `slam.usable=true`. The operator confirmed stable upright stance and clear floor.
- One explicit `POST /posture {"posture":"stand"}` was sent. Its HTTP reply timed out, so it was **not retried**. Fresh readback subsequently reported acknowledged standing, `posture_uncertain=false`, no lease/mission, fresh scan and usable SLAM. The nearby operator separately confirmed physical upright stability before wave trials. The receipt timeout remains a transport anomaly, not proof of a failed stand.

## Narrow one-shot path and results

The opt-in host runner pre-renders a 6.312-second English Sonia greeting (37,872-byte MP3), then checks exact boot, stationary ownership, fresh LiDAR, verified/usable SLAM, and operator-confirmed physical posture. For an idle robot with no previous mission, the explicit stationary-preflight intent uses no fabricated mission-release receipt; the bridge atomically rechecks idle/standing before its SDK action. The host does not issue a late unowned global `/stop`. It reads back the bridge-owned gesture lease, sends raw audio through `POST /audio/play`, then explicitly invokes `POST /posture stand` and checks fresh readback. Human observation remains separate from software posture.

- **Cycle 1:** One `wave_hand` dispatched; matching SDK/action receipt. Direct audio playback acknowledged; operator physically saw a clear wave and heard the greeting intelligibly. Stand recovery acknowledged, `posture_uncertain=false`, stopped/unowned; fresh LiDAR and verified usable SLAM persisted. Operator confirmed upright, stable stance.
- **Cycle 2:** Same one-shot sequence and matching receipts; operator again saw a clear wave, heard the greeting, and confirmed upright stability. Postflight remained stopped/unowned with fresh LiDAR and usable verified SLAM.
- **Attempted cycle 3:** Initial stationary readback timed out **before any gesture or audio request**. Runner reported `dispatch_performed=false`; no recovery posture request was made. A subsequent independent `/mission` readback showed stopped/unowned, posture uncertainty false, fresh scan, usable SLAM. Operator confirmed no third wave and stable stance. This is **not** counted as a completed physical trial; no automatic retry occurred.

Two completed supervised cycles, not three. No automatic gesture allowance is commissioned. A successful SDK receipt and two observed waves do not establish unattended repeatability; the HTTP timeout and the earlier LiDAR outage warrant separate monitoring before travel. The supervised host runner still requires an explicit per-invocation opt-in and nearby physical confirmation.

## Host evidence

`NO_PROXY=127.0.0.1,localhost python -m unittest discover -s tests -q` passed **474 tests** after the opt-in host implementation. The default LifeLoop CLI shadow replay remained zero-dispatch (8 events; one allowed proposal; empty HTTP path list). Raw local trial reports are under the active profile scratch directory as `beep-task4-trial-1/`, `-2/`, and `-3/` (scratch retention is temporary). No user memory bank was pre-seeded and no phone-hotspot rehearsal was run in this task.
