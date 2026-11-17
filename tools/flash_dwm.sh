#!/bin/sh
# Flash a DWM1001-DEV with this repo's nRF52 firmware (_build/uwb_tdma_twr.hex),
# the same image the 24 Sep 2026 survey boards ran (built from 69485cf).
#
# Full chip erase first, so a board that carried PANS or another firmware
# boots clean on this firmware's defaults (921600 baud, uncalibrated antenna
# delay). It still has to be provisioned afterwards (tools/fleet.py).
#
#   tools/flash_dwm.sh               # the only DWM1001 plugged in
#   tools/flash_dwm.sh 760217925     # pick one by J-Link serial
#   tools/flash_dwm.sh --list        # list connected DWM1001s and their serials
set -e
cd "$(dirname "$0")/.."
HEX="$PWD/_build/uwb_tdma_twr.hex"
BIN="$PWD/_build/uwb_tdma_twr.bin"

if [ "$1" = "--list" ]; then
    for p in /dev/cu.usbmodem000*; do
        [ -e "$p" ] || { echo "no DWM1001 connected"; exit 1; }
        sn=${p#/dev/cu.usbmodem000}; echo "${sn%?}  $p"
    done
    exit 0
fi

SEL=""
[ -n "$1" ] && SEL="-SelectEmuBySN $1"
SCRIPT=$(mktemp -t flash_dwm)
LOG=$(mktemp -t flash_dwm_log)
trap 'rm -f "$SCRIPT" "$LOG"' EXIT
JL="JLinkExe $SEL -device NRF52832_XXAA -if SWD -speed 4000 -autoconnect 1 -NoGui 1 -ExitOnError 1"

# Connect-only pass first. On first use a J-Link OB updates its own firmware
# and reboots, and the DLL can then reconnect to a DIFFERENT probe when
# several are plugged in (seen 2026-11-17: asked for 760227775, got
# 760227722). So never erase in the same session that may update firmware,
# and check the serial it actually reports before touching flash.
printf 'qc\n' > "$SCRIPT"
$JL -CommanderScript "$SCRIPT" > "$LOG" 2>&1 || true
printf 'r\nh\nerase\nloadfile %s\nverifybin %s, 0x0\nr\ng\nqc\n' "$HEX" "$BIN" > "$SCRIPT"
$JL -CommanderScript "$SCRIPT" > "$LOG" 2>&1 || { cat "$LOG"; echo "FAILED"; exit 1; }
cat "$LOG"
if [ -n "$1" ]; then
    got=$(grep -o 'S/N: [0-9]*' "$LOG" | head -1 | cut -d' ' -f2)
    if [ "$got" != "$1" ]; then
        echo "WRONG PROBE: asked for $1, flashed $got"; exit 1
    fi
fi
grep -q 'Verify successful' "$LOG" || { echo "VERIFY FAILED"; exit 1; }
echo "flashed ${1:-board} with $(basename "$HEX")"
