"""Opt-in, turn-taking social audio orchestration; no bridge or motor access.

Providers are injected. Live audio needs a measured-capture source, consent,
ASR/TTS credentials, an actual playback receipt, and hardware commissioning.
This loop intentionally does not claim AEC/barge-in during speaker playback.
"""
from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Callable, Any

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


@dataclass(frozen=True)
class DialogueContext:
    memory_summary: str
    gaps: tuple[str, ...]
    history: tuple[dict[str, str], ...]
    visual_context: str | None
    trigger: str | None


# A speaker's claims are not biometric identification or map coordinates.
_NAME = re.compile(r"\b(?:my name is|call me|i(?:'|’)m)\s+([a-z][a-z'-]{0,39})\b", re.I)
_ROOM = re.compile(r"\b(?:this room is|we are in|we(?:'|’)re in)\s+(?:the )?([a-z][a-z ]{0,48})\b", re.I)
_THIS_ROOM = re.compile(r"\bthis is (?:the )?(kitchen|living room|hallway|bedroom|bathroom|office|dining room)\b", re.I)
_FACT = re.compile(r"\bremember that ([^.?!]{1,120})", re.I)
_NATURAL_FACT = re.compile(r"\b(?:i like|i love|i work as|just drinking)\s+([^.?!]{1,100})", re.I)
_TRIGGERS = frozenset({"session_start", "person_noticed", "idle"})
_NOT_NAMES = frozenset({"just", "drinking", "eating", "going", "having", "feeling",
                        "working", "doing", "looking", "wearing", "sitting",
                        "in", "at", "from", "here", "fine", "good", "okay",
                        "happy", "tired", "sorry", "not", "a", "the"})


def _spoken_name(text: str) -> str | None:
    # Conversational "I'm drinking" is not an introduction. This remains a
    # conservative heuristic, never a biometric or verified identity match.
    for match in _NAME.finditer(text):
        if match.group(1).casefold() not in _NOT_NAMES:
            return match.group(1)
    return None


def _short(value: Any, limit: int) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise AudioContractError("invalid bounded dialogue text")
    return " ".join(value.split())


def _visual(value: str | None) -> str | None:
    return None if value is None else _short(value, 240)


def _memory_updates(value: Any, heard: str) -> dict:
    """Validate model suggestions against explicit evidence in human speech."""
    if value is None:
        return {"name": None, "room": None, "facts": [], "topics": []}
    if not isinstance(value, dict) or set(value) - {"name", "room", "facts", "topics"}:
        raise AudioContractError("invalid structured memory fields")
    name, room = value.get("name"), value.get("room")
    facts, topics = value.get("facts", []), value.get("topics", [])
    if name is not None:
        name = _short(name, 40)
        explicit = _spoken_name(heard)
        if explicit is None or explicit.casefold() != name.casefold():
            raise AudioContractError("name is not supported by an introduction")
    if room is not None:
        room = _short(room, 48).lower().replace(" ", "_")
        explicit = _ROOM.search(heard) or _THIS_ROOM.search(heard)
        if explicit is None or explicit.group(1).strip().lower().replace(" ", "_") != room:
            raise AudioContractError("room is not supported by speech")
    if (not isinstance(facts, list) or len(facts) > 8 or
            not isinstance(topics, list) or len(topics) > 8):
        raise AudioContractError("unbounded structured memory lists")
    facts = [_short(fact, 120) for fact in facts]
    topics = [_short(topic, 80) for topic in topics]
    if any(fact.casefold() not in heard.casefold() for fact in facts):
        raise AudioContractError("fact is not present in the transcript")
    return {"name": name, "room": room, "facts": facts, "topics": topics}


class VoiceLoop:
    """Measured PCM -> VAD/handoff -> ASR -> dialogue -> TTS -> playback -> memory."""

    def __init__(self, gate: BargeInGate, handoff: SocialHandoff,
                 memory: SemanticMemory, *,
                 transcribe: Callable[[TranscriptionRequest], str],
                 compose_reply: Callable[[SocialTranscript | None, DialogueContext], str | dict],
                 synthesize: Callable[[str], bytes], play: Callable[[bytes], bool],
                 now_robot: Callable[[], float],
                 read_authority: Callable[[], tuple[str | None, str | None, bool]],
                 person_id: str | None = None,
                 extract_memory: Callable[[SocialTranscript, DialogueContext], dict | None] | None = None,
                 proactive_cooldown_s: float = 30.0):
        if (not isinstance(gate, BargeInGate) or not isinstance(handoff, SocialHandoff)
                or not isinstance(memory, SemanticMemory)
                or (gate.echo.session_id, gate.echo.boot_id) !=
                   (handoff.session_id, handoff.boot_id)
                or not all(callable(v) for v in
                           (transcribe, compose_reply, synthesize, play, now_robot, read_authority))
                or (extract_memory is not None and not callable(extract_memory))
                or isinstance(proactive_cooldown_s, bool)
                or not isinstance(proactive_cooldown_s, (int, float))
                or not 0 <= proactive_cooldown_s <= 600):
            raise AudioContractError("invalid social audio pipeline")
        self.gate, self.handoff, self.memory = gate, handoff, memory
        self.transcribe, self.compose_reply = transcribe, compose_reply
        self.synthesize, self.play = synthesize, play
        self.now_robot, self.read_authority = now_robot, read_authority
        self.person_id, self.extract_memory = person_id, extract_memory
        self.proactive_cooldown_s = proactive_cooldown_s
        self._last_spoken_at: float | None = None
        self._current_room: str | None = None
        self._history: list[dict[str, str]] = []

    @property
    def history(self) -> tuple[dict[str, str], ...]:
        return tuple(dict(item) for item in self._history)

    def _context(self, *, trigger: str | None = None,
                 visual_context: str | None = None) -> DialogueContext:
        person = self.memory.people.get(self.person_id or "")
        gaps = []
        if person is None or person["name"] is None:
            gaps.append("speaker_name_unknown")
        elif person["recent_topics"]:
            gaps.append("follow_up_topic:" + person["recent_topics"][-1][:80])
        # A previously recorded place does not establish where BEEP is now.
        if self._current_room is None:
            gaps.append("room_label_unknown")
        return DialogueContext(self.memory.prompt_summary(), tuple(gaps),
                               self.history, _visual(visual_context), trigger)

    def _speak(self, response: str | dict, authority: tuple) -> tuple[VoiceOutcome, dict | None]:
        updates = None
        if isinstance(response, dict):
            if set(response) != {"reply", "memory_updates"}:
                raise AudioContractError("structured dialogue fields differ")
            reply = _short(response["reply"], 240)
            updates = response["memory_updates"]
        else:
            reply = _short(response, 240)
        if self.read_authority() != authority or not authority[2]:
            return VoiceOutcome("owner_changed_before_tts"), updates
        audio = self.synthesize(reply)
        if not isinstance(audio, bytes) or not 0 < len(audio) <= 2_000_000:
            raise AudioContractError("invalid bounded TTS audio")
        if self.read_authority() != authority:
            return VoiceOutcome("owner_changed_before_playback"), updates
        if self.play(audio) is not True:
            return VoiceOutcome("playback_unconfirmed", reply=reply), updates
        if self.read_authority() != authority:
            return VoiceOutcome("owner_changed_after_playback", reply=reply), updates
        self._last_spoken_at = self.now_robot()
        return VoiceOutcome("replied", reply=reply), updates

    def initiate_turn(self, trigger: str = "session_start",
                      visual_context: str | None = None) -> VoiceOutcome:
        if trigger not in _TRIGGERS:
            raise AudioContractError("unknown proactive trigger")
        authority = self.read_authority()
        if not authority[2]:
            return VoiceOutcome("no_consent")
        now = self.now_robot()
        if (self._last_spoken_at is not None and
                now - self._last_spoken_at < self.proactive_cooldown_s):
            return VoiceOutcome("cooldown")
        if self.gate.vad.active or self.handoff._frames or self.handoff._pending:
            return VoiceOutcome("listening")
        context = self._context(trigger=trigger, visual_context=visual_context)
        response = self.compose_reply(None, context)
        if isinstance(response, dict) and response.get("memory_updates"):
            # A model cannot invent biographical claims while speaking first.
            _memory_updates(response["memory_updates"], "")
        outcome, _ = self._speak(response, authority)
        if outcome.state == "replied":
            self._history.append({"role": "assistant", "content": outcome.reply or ""})
            self._history = self._history[-8:]
            return VoiceOutcome("initiated", reply=outcome.reply)
        return outcome

    def process(self, frame: AudioFrame, *, visual_context: str | None = None) -> VoiceOutcome:
        """Process measured capture while playback is inactive (no commissioned AEC)."""
        mission, lease, consent = self.read_authority()
        decision = self.gate.observe(frame, playback_active=False)
        action = self.handoff.observe(frame, decision, mission_id=mission, lease_id=lease,
                                      upload_consented=consent)
        if not consent:
            return VoiceOutcome("no_consent")
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
        context = self._context(visual_context=visual_context)
        response = self.compose_reply(accepted, context)
        suggested = response.get("memory_updates") if isinstance(response, dict) else None
        if self.extract_memory is not None:
            suggested = self.extract_memory(accepted, context)
        try:
            updates = _memory_updates(suggested, accepted.text)
        except AudioContractError:
            # The model's unsupported memory claim is discarded; the reply
            # can still be spoken, with only transcript-grounded regex notes.
            updates = _memory_updates(None, accepted.text)
        outcome, _ = self._speak(response, current)
        if outcome.state != "replied":
            return VoiceOutcome(outcome.state, accepted.text, outcome.reply)
        self._remember(accepted, outcome.reply or "", updates)
        self._history.extend(({"role": "user", "content": accepted.text[:240]},
                              {"role": "assistant", "content": outcome.reply or ""}))
        self._history = self._history[-8:]
        return VoiceOutcome("replied", accepted.text, outcome.reply, True)

    def _remember(self, transcript: SocialTranscript, reply: str, suggested: dict) -> None:
        text = transcript.text
        spoken_name = _spoken_name(text)
        room_match = _ROOM.search(text) or _THIS_ROOM.search(text)
        fact_match = _FACT.search(text)
        natural_fact = _NATURAL_FACT.search(text)
        name = suggested["name"] or spoken_name
        room = suggested["room"] or (room_match.group(1).strip().lower().replace(" ", "_")
                                     if room_match else None)
        facts = suggested["facts"] or ([fact_match.group(1).strip()] if fact_match else
                                       [natural_fact.group(0).strip()] if natural_fact else [])
        topics = suggested["topics"] or (["introduction"] if name else ["place"] if room else
                                         ["facts"] if facts else ["conversation"])
        person_id = self.person_id or ("self-intro:" + name.casefold() if name else None)
        if name and person_id in self.memory.people:
            known_name = self.memory.people[person_id]["name"]
            if known_name is not None and known_name.casefold() != name.casefold():
                person_id = "self-intro:" + name.casefold()
        # The accepted transcript is not proof of identity or metric location.
        if person_id is not None and (name or facts or topics):
            self.memory.record_person(person_id, name=name, known_facts=facts,
                                      recent_topics=topics)
        if room is not None:
            self.memory.record_place(room, label=room, notes=["speaker-described room"])
        transcript.remember_exchange(self.memory, spoken_reply=reply, person_id=person_id,
            place_label=room, topics=tuple(topics))
        if room is not None:
            self._current_room = room
