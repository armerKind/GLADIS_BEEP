# Opt-in social voice loop (offline implementation)

`beep_audio.voice_loop.VoiceLoop` connects measured 20 ms PCM frames through existing VAD, `BargeInGate` and `SocialHandoff` to injected ASR, contextual reply, TTS, playback and `SemanticMemory`. `initiate_turn(trigger='session_start'|'person_noticed'|'idle', visual_context=None)` permits an opt-in first greeting, with a 30-second default post-speech cooldown and no interruption of an active utterance. It composes from a bounded memory summary, gaps (unknown speaker name/room or the operator-identified person's latest topic), unverified optional visual description and the last eight **in-session role messages**. A room recorded in a prior session does not establish the current room; the current-room gap closes only after a confirmed spoken room claim in this session. This is dialogue context, not face or voice recognition. Neither the voice module nor the script imports motor/bridge control.

The composer receives `(SocialTranscript | None, DialogueContext)` and returns plain reply text or `{'reply': ..., 'memory_updates': {'name': ..., 'room': ..., 'facts': [...], 'topics': [...]}}`. An optional `extract_memory(transcript, context)` callback can supply updates instead. Explicit introductions and place claims must match the human's current utterance; suggested facts must be literal substrings of it. Unsupported updates are discarded and transcript-grounded regexes still extract `I'm …`, `we're in …`, `remember that …` and simple natural facts. A spoken name cannot overwrite an existing confirmed person. These remain speaker **claims**, not biometric identification or map coordinates. A named follow-up gap requires an operator-supplied `person_id`; a visual description alone never resolves identity.

It rechecks consent and the exact owner after ASR, before TTS and before/after playback. Expired, replaced, unconsented or empty transcripts cannot produce a reply or memory update; no conversation or suggested fact is saved until playback returns success. A playback receipt does not establish that the human heard anything. The greeting is kept only in volatile dialogue history (not the conversation bank). History does not survive a restart.

A real, runnable **synthetic offline** path requires no sound card, network, secrets or robot:

```sh
python scripts/run_social_voice.py --offline-demo \
  --memory-file "$HOME/.local/share/beep/demo-memory.json"
```

It proactively asks for a name, then runs three synthetic measured-PCM turns (`Hey, I'm Nico`, `We're in the kitchen`, `Just drinking coffee`) with fake ASR/contextual reply/TTS/playback, and reloads the JSON bank. It checks the name, room and conversation persistence and prints a seven-message history count. Keep demo data separate from the actual private bank.

A separate **uncommissioned** live path in the same script contains optional PortAudio (`sounddevice`) capture from the USB microphone at its verified native 48 kHz rate, with bounded 20 ms three-tap decimation to 16 kHz mono for VAD/ASR, OpenAI transcription/chat/TTS adapters and ALSA `Headphones` output discovered by name at startup. It needs explicit `--consent`, a secret `OPENAI_API_KEY` supplied outside chat, local `sounddevice`/PortAudio, `ffmpeg`, `aplay`, and a stationary, willing speaker. Hermes/GLADIS's conversational provider authentication is **not automatically available to a standalone Pi Python process**: this particular adapter calls OpenAI's endpoints directly. Reusing the existing GLADIS session as the reply engine would require a separate, explicitly designed authenticated relay, not an implicit key handoff. The microphone remains open while TTS plays, but playback frames are discarded after the turn: **there is no commissioned acoustic echo cancellation or true barge-in**. Hardware timestamps, ASR/TTS responses, VAD thresholds and acoustic privacy remain unverified on BEEP. A single supervised stationary posture cycle and analog speaker/mic waveform checks have been completed separately; they do not commission cloud voice. A successful `aplay` return code is a software playback receipt, not proof that sound was heard.

For local-only calibration, run `python scripts/run_social_voice.py --calibrate-mic` on the Pi with its repository dependencies installed. The first quiet tone cues ~3 s of room silence; the second cues ~3 s of normal speech from the intended distance. No raw audio is saved and no cloud/key is used. The result prints median/p95 noise frame RMS and aggregate/p75 speech frame RMS and, **only when the distributions separate sufficiently**, suggested `--onset-rms` and `--offset-rms`. Speech RMS is not ASR intelligibility. The default live onset/offset (850/400) remain bench placeholders: pass measured thresholds explicitly for the first live trial. Inspect USB capture gain with `amixer -c <detected-mic-card> scontents` before calibrating, and set the gain conservatively if below 80%, without assuming the mic card number across boots. Calibrating the USB microphone does not modify bridge/motor authority.

For live playback, the `Headphones` mixer is explicitly unmuted and set to 100% at startup after device discovery. `--speaker-gain` defaults to 4.0 and clamps finite input to 1.0..8.0; ffmpeg applies `volume=<gain>,alimiter=limit=0.95:level=0` (no automatic make-up gain) before ALSA playback. Confirm speech volume with the nearby person; this is not permission to overpower a connected speaker.

For later consented stationary commissioning, provide `--memory-file`, `--consent` and measured VAD flags to `scripts/run_social_voice.py`, optionally `--person-id` for an operator-verified single speaker, `--greet-trigger person_noticed` and `--visual-context 'a short description'`. The CLI does **not** detect a person or capture a camera frame; these options only pass a caller's unverified string into the prompt. The live adapters must first be checked on the actual Pi/audio path and no motion should be initiated by this script. If voice identity is uncertain, omit `--person-id`: a spoken name is saved as a **claimed** introduction, not merged with a previously known person. Voice text is sent to cloud APIs only with opt-in. The bank is plaintext; protect and retain it accordingly.

# Telegram-to-BEEP stationary social relay (no USB mic)

While the USB microphone is unplugged, use a **real Telegram voice message**
received by GLADIS as the transcript source. GLADIS reads the Pi's existing
DialogueContext, composes a short reply itself (not via a second chat API),
optionally supplies strictly transcript-grounded `name`, `room`, `facts`, and
`topics` updates, and synthesizes **en-GB-SoniaNeural** using the host's
keyless Edge TTS. The host transfers only this bounded audio to BEEP via
Jupyter. The Pi runs `--speak`, discovers the current Headphones card by name,
sets its mixer to 100%, applies `volume=4.0,alimiter=limit=0.95:level=0`,
plays on `plughw:<Headphones-card>,0`, and commits the conversation to
`/home/pi/beep_social/memory_bank.json` **only after a successful `aplay`**.
It never starts mic capture, obtains motor authority, or issues motion.

Deploy only stationary files: `python -m scripts.deploy_social_relay`
(with `BEEP_HOST` set to the reachable Pi address). The host relay prefers
Tailscale; when explicitly using the DOGZILLA_WIFI backup set
`BEEP_HOST=192.168.8.88` and `NO_PROXY=192.168.8.88,127.0.0.1,localhost`.
On the host, use `python -m scripts.social_relay --read-context --person-id max`
before composing each reply. For the actual turn, use
`python -m scripts.social_relay --speak --heard-text '...' --reply '...' \
  --person-id max --memory-updates-json '{"facts":[],"topics":[]}'`.
Only use a person ID if independently verified by the operator; a Telegram
sender is not a voice biometric. The host relay uploads a uniquely named MP3,
checks its hash by reading it back, invokes Pi `--speak --tts-audio-file ...`,
reads back the *exact* latest conversation from the Pi memory file, and deletes
the temporary MP3. **Do not retry automatically** after a transport timeout:
playback might have happened even if its acknowledgement was lost. Inspect
memory and ask the nearby human before replaying. A spoken turn requires the
bridge's fresh `/mission` readback to show standing, stopped, unowned, and no
active mission; if the bridge is unavailable the CLI fails closed. Starting
any inactive bridge service is a separate supervised physical-safety action.

For local Pi TTS when available, `python scripts/run_social_voice.py --speak
--heard-text '...' --reply '...' --memory-file /home/pi/beep_social/memory_bank.json`
uses keyless Edge Sonia. The Pi currently has **no** `edge_tts`, piper, or
espeak installed; the tested route is host Edge synthesis and Pi playback.
`--read-context` is read-only and reports remembered observations and gaps,
not a verified current room. Existing direct microphone/cloud live mode remains
separate and uncommissioned. No separate OpenAI chat API key is used here.
