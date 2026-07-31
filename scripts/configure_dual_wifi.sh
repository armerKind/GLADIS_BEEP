#!/usr/bin/env bash
set -euo pipefail

# Persist BEEP's dual-radio roles by hardware MAC rather than unstable wlanN names.
# Run as root on BEEP. The migration keeps a temporary AIDA clone active while
# moving the uplink from the built-in radio to the USB RTL8188EUS adapter.

USB_MAC="${BEEP_USB_WIFI_MAC:-cc:ba:bd:b4:9c:b4}"
BUILTIN_MAC="${BEEP_BUILTIN_WIFI_MAC:-d8:3a:dd:66:50:75}"
UPLINK_PROFILE="${BEEP_UPLINK_PROFILE:-AIDA}"
AP_PROFILE="${BEEP_AP_PROFILE:-DOGZILLA_WIFI}"
BOOTSTRAP_PROFILE="${UPLINK_PROFILE}_USB_BOOTSTRAP"
LEGACY_PROFILE="${UPLINK_PROFILE}_LEGACY"

if [[ ${EUID} -ne 0 ]]; then
  echo "Run as root" >&2
  exit 2
fi

normalize_mac() {
  printf '%s' "$1" | tr '[:upper:]' '[:lower:]'
}

interface_for_mac() {
  local wanted
  wanted="$(normalize_mac "$1")"
  local path actual
  for path in /sys/class/net/*/address; do
    [[ -r "$path" ]] || continue
    actual="$(normalize_mac "$(<"$path")")"
    if [[ "$actual" == "$wanted" ]]; then
      basename "$(dirname "$path")"
      return 0
    fi
  done
  return 1
}

USB_IFACE="$(interface_for_mac "$USB_MAC")" || {
  echo "USB Wi-Fi adapter $USB_MAC not found" >&2
  exit 3
}
BUILTIN_IFACE="$(interface_for_mac "$BUILTIN_MAC")" || {
  echo "Built-in Wi-Fi $BUILTIN_MAC not found" >&2
  exit 4
}

if [[ "$USB_IFACE" == "$BUILTIN_IFACE" ]]; then
  echo "Resolved both radio roles to $USB_IFACE" >&2
  exit 5
fi

printf 'USB uplink: %s (%s)\n' "$USB_IFACE" "$USB_MAC"
printf 'Built-in AP: %s (%s)\n' "$BUILTIN_IFACE" "$BUILTIN_MAC"

nmcli radio wifi on
nmcli connection show "$UPLINK_PROFILE" >/dev/null
nmcli connection show "$AP_PROFILE" >/dev/null

# Create a second uplink profile so connectivity survives while the original
# active profile releases the built-in radio.
nmcli connection delete "$BOOTSTRAP_PROFILE" >/dev/null 2>&1 || true
nmcli connection delete "$LEGACY_PROFILE" >/dev/null 2>&1 || true
nmcli connection clone "$UPLINK_PROFILE" "$BOOTSTRAP_PROFILE"
nmcli connection modify "$BOOTSTRAP_PROFILE" \
  connection.interface-name "" \
  connection.autoconnect yes \
  connection.autoconnect-priority 50 \
  connection.autoconnect-retries 0 \
  802-11-wireless.mac-address "$USB_MAC" \
  802-11-wireless.band bg \
  802-11-wireless.powersave 2 \
  ipv4.method auto \
  ipv4.never-default no \
  ipv4.route-metric 100

nmcli --wait 30 connection up "$BOOTSTRAP_PROFILE" ifname "$USB_IFACE"
USB_ADDRESS="$(nmcli -g IP4.ADDRESS device show "$USB_IFACE" | sed -n '1p')"
if [[ -z "$USB_ADDRESS" ]]; then
  echo "USB uplink has no IPv4 address" >&2
  exit 6
fi
printf 'USB uplink address: %s\n' "$USB_ADDRESS"

# Preserve the old profile until the bootstrap profile is active, then retire it.
ORIGINAL_UUID="$(nmcli -g connection.uuid connection show "$UPLINK_PROFILE")"
nmcli connection modify uuid "$ORIGINAL_UUID" connection.id "$LEGACY_PROFILE" connection.autoconnect no
nmcli connection down uuid "$ORIGINAL_UUID" >/dev/null 2>&1 || true
nmcli connection modify "$BOOTSTRAP_PROFILE" connection.id "$UPLINK_PROFILE"

# Bind the original DogZilla AP profile to the built-in radio by permanent MAC.
nmcli connection modify "$AP_PROFILE" \
  connection.interface-name "" \
  connection.autoconnect yes \
  connection.autoconnect-priority 100 \
  connection.autoconnect-retries 0 \
  802-11-wireless.mac-address "$BUILTIN_MAC" \
  802-11-wireless.band bg \
  802-11-wireless.powersave 2 \
  ipv4.method shared \
  ipv4.addresses 192.168.8.88/24 \
  ipv4.never-default yes

nmcli --wait 30 connection up "$AP_PROFILE" ifname "$BUILTIN_IFACE"
AP_ADDRESS="$(nmcli -g IP4.ADDRESS device show "$BUILTIN_IFACE" | sed -n '1p')"
if [[ "$AP_ADDRESS" != 192.168.8.88/* ]]; then
  echo "DogZilla AP has unexpected address: $AP_ADDRESS" >&2
  exit 7
fi

# The migration succeeded; remove the disabled interface-name-bound profile.
nmcli connection delete "$LEGACY_PROFILE" >/dev/null

printf '\nActive connections:\n'
nmcli -f NAME,TYPE,DEVICE connection show --active
printf '\nDevice status:\n'
nmcli device status
printf '\nRoutes:\n'
ip route
