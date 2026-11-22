"""Opt-in, turn-taking social audio orchestration; no bridge or motor access.

Providers are injected. Live audio needs a measured-capture source, consent,
ASR/TTS credentials, an actual playback receipt, and hardware commissioning.
This loop intentionally does not claim AEC/barge-in during speaker playback.
"""
from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Callable

from beep_agent.memory import SemanticMemory
from .capture import AudioFrame, AudioContractError
from .echo import BargeInGate
from .social import SocialHandoff, SocialTranscript, TranscriptResult, TranscriptionRequest


@dataclass(frozen=True)
class VoiceOutcome:
    state: str
    transcript: str | None = None
    reply: str | None = None
    memory_written: bool = False
    motor_authority: bool = False


# Introductions are explicit claims by a speaker, not biometric identification.
_NAME = re.compile(r"\bmy name is ([a-z][a-z'-]{0,39})\b", re.I)
_ROOM = re.compile(r"\b(?:this room is|we are in) (?:the )?([a-z][a-z ]{0,48})\b", re.I)
_FACT = re.compile(r"\bremember that ([^.?!]{1,120})", re.I)


class VoiceLoop:
    """Measured PCM -> VAD/handoff -> ASR -> reply -> TTS -> playback -> memory."""

    def __init__(self, gate: BargeInGate, handoff: SocialHandoff,
                 memory: SemanticMemory, *,
                 transcribe: Callable[[TranscriptionRequest], str],
                 compose_reply: Callable[[SocialTranscript, str], str],
                 synthesize: Callable[[str], bytes],
                 play: Callable[[bytes], bool],
                 now_robot: Callable[[], float],
                 read_authority: Callable[[], tuple[str | None, str | None, bool]],
                 person_id: str | None = None):
        if (not isinstance(gate, BargeInGate) or not isinstance(handoff, SocialHandoff)
                or not isinstance(memory, SemanticMemory)
                or (gate.echo.session_id, gate.echo.boot_id) !=
                   (handoff.session_id, handoff.boot_id)
                or not all(callable(v) for v in
                           (transcribe, compose_reply, synthesize, play, now_robot, read_authority))):
            raise AudioContractError("invalid social audio pipeline")
        self.gate, self.handoff, self.memory = gate, handoff, memory
        self.transcribe, self.compose_reply = transcribe, compose_reply
        self.synthesize, self.play = synthesize, play
        self.now_robot, self.read_authority = now_robot, read_authority
        self.person_id = person_id

    def process(self, frame: AudioFrame) -> VoiceOutcome:
        """Handle one measured frame; the caller must keep the mic capturing.

        Caller must not pass captured speaker-playback frames as unprocessed near-
        end speech: without commissioned AEC, hold recognition during playback.
        """
        mission, lease, consent = self.read_authority()
        if not consent:
            # Feed the frame so an active utterance and pending receipt are
            # invalidated; no cloud call or saved transcript is permitted.
            decision = self.gate.observe(frame, playback_active=False)
            self.handoff.observe(frame, decision, mission_id=mission, lease_id=lease,
                                 upload_consented=False)
            return VoiceOutcome("no_consent")
        decision = self.gate.observe(frame, playback_active=False)
        action = self.handoff.observe(frame, decision, mission_id=mission, lease_id=lease,
                                      upload_consented=True)
        if action.transcription is None:
            return VoiceOutcome("utterance_aborted" if action.utterance_aborted else "listening")
        request = action.transcription
        text = self.transcribe(request)
        current = self.read_authority()
        if not isinstance(text, str) or not text.strip() or len(text) > 1000:
            return VoiceOutcome("asr_empty")
        result = TranscriptResult(request.utterance_id, request.session_id, request.boot_id,
                                  request.mission_id, request.lease_id,
                                  text.strip(), self.now_robot())
        accepted = self.handoff.accept_transcript(result, mission_id=current[0],
            lease_id=current[1], now_robot=self.now_robot(), upload_consented=current[2])
        if accepted is None:
            return VoiceOutcome("stale_or_unconsented")
        reply = self.compose_reply(accepted, self.memory.prompt_summary())
        if not isinstance(reply, str) or not 1 <= len(reply.strip()) <= 240:
            raise AudioContractError("invalid spoken reply")
        reply = reply.strip()
        audio = self.synthesize(reply)
        if not isinstance(audio, bytes) or not 0 < len(audio) <= 2_000_000:
            raise AudioContractError("invalid bounded TTS audio")
        if self.read_authority() != current:
            return VoiceOutcome("owner_changed_before_playback", accepted.text)
        if self.play(audio) is not True:
            return VoiceOutcome("playback_unconfirmed", accepted.text, reply)
        if self.read_authority() != current:
            return VoiceOutcome("owner_changed_after_playback", accepted.text, reply)
        self._remember(accepted, reply)
        return VoiceOutcome("replied", accepted.text, reply, True)

    def _remember(self, transcript: SocialTranscript, reply: str) -> None:
        text = transcript.text
        name_match = _NAME.search(text)
        room_match = _ROOM.search(text)
        fact_match = _FACT.search(text)
        name = name_match.group(1) if name_match else None
        room = room_match.group(1).strip().lower().replace(" ", "_") if room_match else None
        topic = "introduction" if name else "place" if room else "facts" if fact_match else "conversation"
        person_id = self.person_id or ("self-intro:" + name.casefold() if name else None)
        if name and person_id in self.memory.people:
            known_name = self.memory.people[person_id]["name"]
            if known_name is not None and known_name.casefold() != name.casefold():
                # A different voice must not rename a confirmed profile.
                person_id = "self-intro:" + name.casefold()
        # The accepted transcript is not proof of identity or metric location.
        # Record it as a claim, after successful audio output, never as a route.
        if person_id is not None and (name or fact_match):
            self.memory.record_person(person_id, name=name,
                known_facts=([fact_match.group(1).strip()] if fact_match else []),
                recent_topics=[topic])
        if room is not None:
            self.memory.record_place(room, label=room, notes=["speaker-described room"])
        transcript.remember_exchange(self.memory, spoken_reply=reply, person_id=person_id,
            place_label=room, topics=(topic,))
