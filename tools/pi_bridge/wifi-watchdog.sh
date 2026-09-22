#!/bin/bash
# Belt-and-suspenders wifi retry. NetworkManager's netplan-rendered
# connection already has autoconnect=yes (confirmed via `nmcli connection
# show`), so this is not the primary reconnect mechanism -- it exists for
# the case autoconnect's own backoff gives up or wlan0 wedges in a state
# NetworkManager doesn't notice on its own. Runs every 2 minutes via the
# paired .timer unit; each run is a few seconds and idle (no ping, no
# nmcli call) when the link is already up.
set -euo pipefail

if ! ip route get 1.1.1.1 >/dev/null 2>&1; then
    logger -t wifi-watchdog "no default route -- nudging wlan0"
    nmcli device disconnect wlan0 >/dev/null 2>&1 || true
    sleep 2
    nmcli device connect wlan0 >/dev/null 2>&1 || \
        nmcli connection up netplan-wlan0-TurboBox_ZT7C3C >/dev/null 2>&1 || true
fi
