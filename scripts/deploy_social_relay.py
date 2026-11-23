#!/usr/bin/env python3
"""Deploy only stationary social CLI and orchestration to BEEP via Jupyter.

Does not touch bridge, vendor service, mic, GPIO or motor authority.
"""
from __future__ import annotations
import hashlib
from pathlib import Path
from scripts.jupyter_deploy_bridge import BASE, login

FILES = ('scripts/run_social_voice.py', 'beep_audio/voice_loop.py')

def main():
    session, xsrf = login()
    headers = {'X-XSRFToken': xsrf, 'Content-Type': 'application/json'}
    for name in FILES:
        source = Path(name).read_text(encoding='utf-8')
        dest = BASE + '/api/contents/beep_social/' + name
        resp = session.put(dest, headers=headers,
            json={'type': 'file', 'format': 'text', 'content': source}, timeout=12)
        resp.raise_for_status()
        check = session.get(dest + '?content=1', timeout=12)
        check.raise_for_status()
        remote = check.json()['content']
        if hashlib.sha256(source.encode()).digest() != hashlib.sha256(remote.encode()).digest():
            raise RuntimeError('remote source readback differs: ' + name)
        print('deployed_and_verified:', name)

if __name__ == '__main__':
    main()
