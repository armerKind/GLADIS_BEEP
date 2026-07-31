#!/usr/bin/env python3
"""Apply BEEP-safe compatibility patches to Yahboom's vendor app."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

DEFAULT_APP = Path("/home/pi/DOGZILLA/app_dogzilla/app_dogzilla.py")

OLD_IP_LOOKUP = '''def get_ip_address():
    ip = os.popen(
        "/sbin/ifconfig eth0 | grep 'inet' | awk '{print $2}'").read()
    ip = ip[0: ip.find('\\n')]
    if(ip == '' or len(ip) > 15):
        ip = os.popen(
            "/sbin/ifconfig wlan0 | grep 'inet' | awk '{print $2}'").read()
        ip = ip[0: ip.find('\\n')]
        if(ip == ''):
            ip = 'x.x.x.x'
    if len(ip) > 15:
        ip = 'x.x.x.x'
    return ip
'''

NEW_IP_LOOKUP = '''def get_ip_address():
    # Interface numbering changes when the USB Wi-Fi dongle is present.
    # Accept either radio and Tailscale instead of blocking startup on wlan0.
    for interface in ('eth0', 'wlan0', 'wlan1', 'tailscale0'):
        command = "/sbin/ifconfig %s | grep 'inet ' | awk '{print $2}'" % interface
        ip = os.popen(command).read().splitlines()
        if ip and 0 < len(ip[0]) <= 15:
            return ip[0]
    return 'x.x.x.x'
'''

OLD_TCP = 'task_tcp = threading.Thread(target=start_tcp_server, name="task_tcp", args=(ip, 6000))'
NEW_TCP = 'task_tcp = threading.Thread(target=start_tcp_server, name="task_tcp", args=("0.0.0.0", 6000))'
OLD_ACTION = '    g_dog.action(14) # 开机展示伸懒腰动作'
NEW_ACTION = "    if os.environ.get('DOGZILLA_SKIP_STARTUP_ACTION') != '1':\n        g_dog.action(14) # 开机展示伸懒腰动作"


def apply_patch(path: Path) -> bool:
    text = path.read_text()
    original = text

    if NEW_IP_LOOKUP not in text:
        if text.count(OLD_IP_LOOKUP) != 1:
            raise RuntimeError("unexpected vendor IP-discovery source")
        text = text.replace(OLD_IP_LOOKUP, NEW_IP_LOOKUP)

    if NEW_TCP not in text:
        if text.count(OLD_TCP) != 1:
            raise RuntimeError("unexpected vendor TCP-bind source")
        text = text.replace(OLD_TCP, NEW_TCP)

    if NEW_ACTION not in text:
        if text.count(OLD_ACTION) != 1:
            raise RuntimeError("unexpected vendor startup-action source")
        text = text.replace(OLD_ACTION, NEW_ACTION)

    if text == original:
        return False

    backup = path.with_suffix(path.suffix + ".gladis-original")
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(text)
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("path", nargs="?", type=Path, default=DEFAULT_APP)
    args = parser.parse_args()
    changed = apply_patch(args.path)
    print("patched" if changed else "already patched")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
