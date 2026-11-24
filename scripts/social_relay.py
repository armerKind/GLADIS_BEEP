#!/usr/bin/env python3
"""Host-side Telegram relay: keyless Sonia synthesis -> Jupyter -> Pi speaker.

Operator supplies one actual Telegram transcript and GLADIS reply. No chat API,
mic capture, motor command or automatic retry. The Pi owns memory and playback.
"""
from __future__ import annotations
import argparse
import asyncio
import base64
import hashlib
import io
import json
import os
import time
import uuid
from contextlib import redirect_stdout
from pathlib import Path

import websockets
# Tailscale is the normal route; explicitly set BEEP_HOST=192.168.8.88
# only for the DOGZILLA_WIFI backup when the host tailnet is unavailable.
os.environ.setdefault('BEEP_HOST', '100.100.141.33')
from scripts.jupyter_deploy_bridge import BASE, HOST, login
from beep_agent.memory import SemanticMemory
from scripts.run_social_voice import edge_sonia, parse_args as social_args, speak_turn, voice_for_language

PI_SCRIPT = '/home/pi/beep_social/scripts/run_social_voice.py'
PI_MEMORY = '/home/pi/beep_social/memory_bank.json'
HOST_MEMORY = Path.home() / '.hermes/profiles/sol/beep_memory_bank.json'

def direct_turn(session, xsrf, args) -> dict:
    """Audio uses bridge HTTP; Jupyter carries only the small memory snapshot."""
    if not args.heard_text or not args.reply:
        raise ValueError('--heard-text and --reply required for --speak')
    language = getattr(args, 'language', 'en')
    voice = voice_for_language(language, getattr(args, 'tts_voice', None))
    url = BASE + '/api/contents/beep_social/memory_bank.json?content=1'
    prior = session.get(url, timeout=9)
    if prior.status_code != 200:
        prior.raise_for_status()
    original = prior.json()['content'].encode('utf-8')
    baseline = hashlib.sha256(original).digest()
    if HOST_MEMORY.exists() and hashlib.sha256(HOST_MEMORY.read_bytes()).digest() != baseline:
        raise RuntimeError('host/Pi semantic memory differs; reconcile before playback')
    HOST_MEMORY.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(HOST_MEMORY, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'wb') as out:
        out.write(original)
    cli = ['--speak', '--memory-file', str(HOST_MEMORY), '--bridge-url', args.bridge_url,
           '--heard-text', args.heard_text, '--reply', args.reply,
           '--speaker-gain', str(args.speaker_gain), '--language', language,
           '--tts-voice', voice]
    if args.person_id:
        cli += ['--person-id', args.person_id]
    if args.memory_updates_json:
        cli += ['--memory-updates-json', args.memory_updates_json]
    capture = io.StringIO()
    with redirect_stdout(capture):
        speak_turn(SemanticMemory(HOST_MEMORY), social_args(cli))
    receipt = json.loads(capture.getvalue())
    # This is the commit to Pi, not audio transport. Never overwrite a bank
    # that changed independently while BEEP spoke.
    current = session.get(url, timeout=9)
    current.raise_for_status()
    if hashlib.sha256(current.json()['content'].encode()).digest() != baseline:
        raise RuntimeError('Pi semantic memory changed during speech; host copy retained')
    fresh = HOST_MEMORY.read_text(encoding='utf-8')
    started = time.perf_counter()
    target = BASE + '/api/contents/beep_social/memory_bank.json'
    headers = {'X-XSRFToken': xsrf, 'Content-Type': 'application/json'}
    uploaded = session.put(target, headers=headers,
        json={'type': 'file', 'format': 'text', 'content': fresh}, timeout=10)
    uploaded.raise_for_status()
    check = session.get(url, timeout=9)
    check.raise_for_status()
    if hashlib.sha256(check.json()['content'].encode()).digest() != hashlib.sha256(fresh.encode()).digest():
        raise RuntimeError('Pi semantic memory upload readback differs')
    conversations = json.loads(check.json()['content'])['conversations']
    if not conversations or conversations[-1]['heard_text'] != args.heard_text or conversations[-1]['spoken_reply'] != args.reply:
        raise RuntimeError('Pi final conversation differs from delivered turn')
    receipt['memory_sync_s'] = round(time.perf_counter() - started, 3)
    receipt['verified_conversation_count'] = len(conversations)
    receipt['language'] = language
    receipt['tts_voice'] = voice
    return receipt

async def execute(session, xsrf, code: str, *, timeout=55) -> dict:
    headers = {'X-XSRFToken': xsrf, 'Content-Type': 'application/json'}
    resp = session.post(BASE + '/api/kernels', headers=headers,
                        json={'name': 'python3'}, timeout=8)
    resp.raise_for_status()
    kernel = resp.json()['id']
    msg_id = str(uuid.uuid4())
    cookie = '; '.join(f'{c.name}={c.value}' for c in session.cookies)
    message = {'header': {'msg_id': msg_id, 'username': 'gladis',
               'session': str(uuid.uuid4()), 'msg_type': 'execute_request', 'version':'5.3'},
               'parent_header': {}, 'metadata': {},
               'content': {'code': code, 'silent': False, 'store_history': False,
                           'user_expressions': {}, 'allow_stdin': False,
                           'stop_on_error': True}, 'channel': 'shell', 'buffers': []}
    output = []
    try:
        async with websockets.connect(f'ws://{HOST}:8888/api/kernels/{kernel}/channels',
                    additional_headers={'Cookie': cookie}, open_timeout=8,
                    ping_interval=None) as ws:
            await ws.send(json.dumps(message))
            while True:
                msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=timeout))
                if msg.get('parent_header', {}).get('msg_id') != msg_id:
                    continue
                if msg.get('msg_type') == 'stream':
                    output.append(msg['content'].get('text', ''))
                if msg.get('msg_type') == 'error':
                    raise RuntimeError('remote relay failed: ' +
                                       msg.get('content', {}).get('evalue', 'unknown error')[:240])
                if msg.get('msg_type') == 'execute_reply':
                    if msg.get('content', {}).get('status') != 'ok':
                        raise RuntimeError('remote relay execution failed')
                    return json.loads(''.join(output).strip())
    finally:
        session.delete(BASE + '/api/kernels/' + kernel, headers=headers, timeout=8).raise_for_status()

def relay(args) -> dict:
    session, xsrf = login()
    if getattr(args, 'bridge_url', None) and not args.read_context:
        return direct_turn(session, xsrf, args)
    if args.read_context:
        payload = {'mode': 'read', 'person_id': args.person_id}
        return asyncio.run(execute(session, xsrf, remote_code(payload)))
    if not args.heard_text or not args.reply:
        raise ValueError('--heard-text and --reply required for --speak')
    if len(args.heard_text) > 500 or len(args.reply) > 240:
        raise ValueError('turn text exceeds bounded memory or speech limits')
    updates = json.loads(args.memory_updates_json) if args.memory_updates_json else None
    # Keyless Edge TTS on the host; no chat/OpenAI API key.
    language = getattr(args, 'language', 'en')
    voice = voice_for_language(language, getattr(args, 'tts_voice', None))
    audio = edge_sonia(args.reply, voice)
    if not 0 < len(audio) <= 2_000_000:
        raise RuntimeError('Edge TTS returned invalid or oversized audio')
    name = 'relay-' + uuid.uuid4().hex + '.mp3'
    url = BASE + '/api/contents/beep_social/' + name
    headers = {'X-XSRFToken': xsrf, 'Content-Type': 'application/json'}
    try:
        uploaded = session.put(url, headers=headers,
            json={'type': 'file', 'format': 'base64',
                  'content': base64.b64encode(audio).decode()}, timeout=18)
        uploaded.raise_for_status()
        check = session.get(url + '?content=1', timeout=12)
        check.raise_for_status()
        readback = base64.b64decode(check.json()['content'])
        if hashlib.sha256(readback).digest() != hashlib.sha256(audio).digest():
            raise RuntimeError('Pi TTS upload readback differs')
        payload = {'mode': 'speak', 'heard': args.heard_text, 'reply': args.reply,
                   'person_id': args.person_id, 'updates': updates,
                   'gain': args.speaker_gain,
                   'audio_file': '/home/pi/beep_social/' + name}
        return asyncio.run(execute(session, xsrf, remote_code(payload)))
    finally:
        # Unlink rather than Jupyter's delete-to-trash: MP3 is ephemeral.
        try:
            cleanup = '''from pathlib import Path
import json
p=Path(%r)
p.unlink(missing_ok=True)
print(json.dumps({'removed': not p.exists()}))
''' % ('/home/pi/beep_social/' + name,)
            outcome = asyncio.run(execute(session, xsrf, cleanup))
            if not outcome.get('removed'):
                raise RuntimeError('Pi temporary audio cleanup failed')
            absent = session.get(url + '?content=0', timeout=8)
            if absent.status_code != 404:
                raise RuntimeError('Pi temporary audio still visible')
        except Exception:
            # Best-effort fallback; operator must inspect if the transport broke.
            session.delete(url, headers=headers, timeout=8)
            raise

def remote_code(payload: dict) -> str:
    encoded = base64.b64encode(json.dumps(payload, ensure_ascii=False).encode()).decode()
    return '''import base64,json,subprocess,sys
p=json.loads(base64.b64decode(%r))
a=[sys.executable,%r,'--memory-file',%r]
if p.get('person_id'):a+=['--person-id',p['person_id']]
if p['mode']=='read':a+=['--read-context']
else:
 a+=['--speak','--heard-text',p['heard'],'--reply',p['reply'],
     '--speaker-gain',str(p['gain']),'--tts-audio-file',p['audio_file']]
 if p.get('updates') is not None:a+=['--memory-updates-json',json.dumps(p['updates'])]
r=subprocess.run(a,capture_output=True,text=True,timeout=48)
if r.returncode:raise RuntimeError('Pi social CLI rejected turn: '+r.stderr[-240:])
result=json.loads(r.stdout.strip())
if p['mode']=='speak':
 from pathlib import Path
 bank=json.loads(Path(%r).read_text())
 last=bank['conversations'][-1]
 if result.get('state')!='replied' or not result.get('memory_written') or last['heard_text']!=p['heard'] or last['spoken_reply']!=p['reply']:
  raise RuntimeError('Pi memory readback failed')
 result={'state':'replied','memory_written':True,'verified_conversation_count':len(bank['conversations'])}
print(json.dumps(result,ensure_ascii=False))
''' % (encoded, PI_SCRIPT, PI_MEMORY, PI_MEMORY)

def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--read-context', action='store_true')
    mode.add_argument('--speak', action='store_true')
    p.add_argument('--heard-text')
    p.add_argument('--reply')
    p.add_argument('--memory-updates-json')
    p.add_argument('--person-id')
    p.add_argument('--speaker-gain', type=float, default=3.0)
    p.add_argument('--language', default='en',
                   help='Reply language code or locale (en, de, es, fr, zh, pt-BR...); no automatic translation')
    p.add_argument('--tts-voice', help='Optional Edge Neural voice in the selected language')
    p.add_argument('--bridge-url', help='Direct bridge audio over HTTP; Jupyter syncs semantic memory only')
    args = p.parse_args()
    print(json.dumps(relay(args), ensure_ascii=False))

if __name__=='__main__':
    main()
