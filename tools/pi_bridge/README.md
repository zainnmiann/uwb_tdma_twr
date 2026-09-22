# pi_bridge

Deployed 2026-09-22. Tags plugged into a Raspberry Pi (`d7tech@192.168.0.165`,
key `~/.ssh/id_ed25519_pi`), streamed to a Mosquitto broker at
`192.168.0.180:1883` (this Mac, via Homebrew -- `brew services` /
`/opt/homebrew/opt/mosquitto/sbin/mosquitto`, anonymous connections allowed).

Replaces an unrelated leftover `uwb-bridge.service` from a prior PANS-based
deployment (see the `uwb-mqtt-bridge` memory note) that was found already
installed and running on this Pi -- wrong protocol entirely, removed
outright rather than adapted.

## Deploy / redeploy

```
scp -i ~/.ssh/id_ed25519_pi uwb_bridge.py d7tech@192.168.0.165:/tmp/
ssh -i ~/.ssh/id_ed25519_pi d7tech@192.168.0.165 \
  'sudo mv /tmp/uwb_bridge.py /opt/uwb_bridge/uwb_bridge.py && sudo systemctl restart uwb-bridge'
```

First-time install additionally needs `uwb-bridge.service` copied to
`/etc/systemd/system/`, `sudo usermod -a -G dialout d7tech` (serial port
access without root), `sudo systemctl daemon-reload && sudo systemctl enable
--now uwb-bridge`.

## Verify

```
mosquitto_sub -h 192.168.0.180 -t 'uwb/ranging/#' -v
sudo journalctl -u uwb-bridge -f          # on the Pi
```

## Design notes

- **Auto-discovery, not hardcoded ports.** `uwb_bridge.py` queries every
  `/dev/serial/by-id/*` (falling back to `/dev/ttyUSB*`/`/dev/ttyACM*`)
  candidate's own `$CFG` and matches `node_id` against the `TAGS` table --
  it does not hardcode which USB serial number is which tag. A board
  swapped for a spare, or plugged into a different port, is found on the
  next rescan (default every 15s) with no config change needed. An anchor,
  or anything else on the bus, answers `$CFG` with `role != INIT` (or an
  unrecognized `node_id`) and is left alone.
- **`TAGS`/`ANCHORS` tables must be kept in sync** with `tools/uwb.py`'s
  `BOARDS` and `tools/fleet.py`'s provisioning -- this file has no other
  source of truth for node_id -> label.
- **One thread per tag**, each with its own reconnect loop; a dropped USB
  cable on one tag doesn't affect the others or the MQTT connection.
- Payload schema and the `timestamp_sent`/`timestamp_received` convention
  match `tools/raw_json.py` on the Mac side (see that file for the
  reasoning) -- a message from either is drop-in comparable.
- Checksum framing (`cs()`, `check_line()`) is copied verbatim from
  `tools/uwb.py`, not reimplemented, so the two never silently drift apart.

## Known gotcha hit during setup

Python 3.13's `threading.Thread` sets an internal instance attribute
literally named `_handle` (added for the new thread-handle machinery) --
a subclass method of the same name gets silently shadowed by it (instance
`__dict__` wins over class methods), surfacing as
`TypeError: '_thread._ThreadHandle' object is not callable` the moment the
method is called via `self._handle(...)`. The line handler here is named
`_on_line`, not `_handle`, to avoid this. Worth remembering for any future
`threading.Thread` subclass on Python >= 3.13.
