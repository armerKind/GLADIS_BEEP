#!/usr/bin/env bash
# Restore the USB hotspot uplink without touching the built-in DOGZILLA_WIFI AP.
set -euo pipefail
profile=AIDA
adapter_mac=cc:ba:bd:b4:9c:b4
iface=''
for path in /sys/class/net/*/address; do
    if [[ $(<"$path") == "$adapter_mac" ]]; then
        iface=${path%/address}
        iface=${iface##*/}
        break
    fi
done
if [[ -z $iface ]]; then
    logger -t beep-uplink-repair 'USB uplink adapter absent; leaving AP untouched'
    exit 0
fi
# A cached NM "connected" flag alone is insufficient: the observed failure
# had no IPv4 address or default route, leaving Tailscale offline.
if [[ $(nmcli -g GENERAL.CONNECTION device show "$iface") == "$profile" ]] &&
   [[ -n $(ip -4 -o addr show dev "$iface" scope global) ]] &&
   [[ -n $(ip -4 route show default dev "$iface") ]]; then
    exit 0
fi
logger -t beep-uplink-repair "recovering $profile on $iface (address/route missing)"
nmcli connection down "$profile" >/dev/null 2>&1 || true
nmcli connection up "$profile" ifname "$iface" >/dev/null
if [[ -z $(ip -4 -o addr show dev "$iface" scope global) ]] ||
   [[ -z $(ip -4 route show default dev "$iface") ]]; then
    logger -t beep-uplink-repair "uplink still lacks IPv4/default route on $iface"
    exit 1
fi
logger -t beep-uplink-repair "uplink restored on $iface"
