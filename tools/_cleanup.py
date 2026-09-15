"""Shared teardown for the live tools.

Every tool here holds serial ports open and some also mutate board state
(raw_json/tdma_order set out_mask to enable RAW_TS). If a tool is killed
before its teardown runs, it leaves the port held AND the board degraded --
which is exactly what happened during testing: 18 orphaned processes
contending for the same ports and repeatedly re-enabling RAW_TS, which cost
~12% tx_late and made the network look like it had a firmware regression.

Using this guarantees teardown on normal exit, Ctrl-C, SIGTERM and
uncaught exceptions, and covers work done during startup/calibration --
not just the main loop.
"""
import atexit, signal, sys

_actions = []
_done = False


def on_exit(fn):
    """Register a teardown callback. Runs exactly once, whatever happens."""
    _actions.append(fn)


def _run(*_a):
    global _done
    if _done:
        return
    _done = True
    for fn in reversed(_actions):
        try:
            fn()
        except Exception as e:
            print(f"# cleanup: {fn!r} failed: {e}", file=sys.stderr)


def install():
    """Arm teardown for every exit path. Call once, early."""
    atexit.register(_run)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        try:
            signal.signal(sig, lambda s, f: (_run(), sys.exit(128 + s)))
        except (ValueError, OSError):
            pass       # not in main thread / unsupported platform
