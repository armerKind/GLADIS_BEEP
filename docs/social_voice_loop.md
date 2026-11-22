# Opt-in social voice loop (offline implementation)

`beep_audio.voice_loop.VoiceLoop` connects measured 20 ms PCM frames through existing VAD, `BargeInGate` and `SocialHandoff` to an injected ASR provider, a short reply, TTS, playback and `SemanticMemory`. It rechecks consent and the exact owner after ASR and before/after playback. An expired, replaced, unconsented or empty transcript cannot produce a reply or memory update. It writes a conversation only after playback returns success, and extracts **explicitly spoken** introductions (`my name is …`), place claims (`this room is …`), and facts (`remember that …`). These are speaker claims, not biometric identification or map coordinates; a conflicting name cannot overwrite an existing confirmed person. No motor or bridge module is imported.

A real, runnable **synthetic offline** path requires no sound card, network, secrets or robot:

```sh
python scripts/run_social_voice.py --offline-demo \
  --memory-file "$HOME/.local/share/beep/demo-memory.json"
```

It runs three turns with synthetic measured PCM and fake ASR/reply/TTS/playback, then reloads the one JSON bank. Keep demo data separate from the actual private bank.

A separate **uncommissioned** live path in the same script contains optional PortAudio (`sounddevice`) 16 kHz mono mic capture, OpenAI transcription/chat/TTS adapters and ALSA `Headphones` output discovered by name at startup. It needs explicit `--consent`, a secret `OPENAI_API_KEY` supplied outside chat, local `sounddevice`/PortAudio, `ffmpeg`, `aplay`, and a stationary, willing speaker. The microphone remains open while TTS plays, but playback frames are discarded after the turn: **there is no commissioned acoustic echo cancellation or true barge-in**. Hardware timestamps, ASR/TTS responses, speaker audibility, gain, VAD thresholds and acoustic privacy have not been verified on BEEP. A successful `aplay` return code is a software playback receipt, not proof that sound was heard. This code has **not been deployed or run on the charging robot**. Do not treat the offline demo as a live conversation.

For later consented stationary commissioning, provide `--memory-file` and `--consent` to `scripts/run_social_voice.py`, optionally `--person-id` for an operator-verified single speaker. The live adapters must first be checked on the actual Pi/audio path and no motion should be initiated by this script. If voice identity is uncertain, omit `--person-id`: a spoken name is saved as a **claimed** introduction, not merged with a previously known person. Voice text is sent to cloud APIs only with opt-in. The bank is plaintext; protect and retain it accordingly.
