#!/usr/bin/env python3
"""
Watches for RealDash's recurring stale-connection bug and recovers from it automatically.
Recurring pattern seen throughout this project (see HANDOFF.md): after the VCU (Teensy)
reboots — which happens on every sleep/wake cycle, not just a fresh flash — RealDash
sometimes ends up feeding its display from a stale TCP connection to the VCU's port 35000
(realdash_tcp.cpp) while the VCU has moved on. Net effect: gauges freeze.

Detection is based on data freshness (ss -tni's lastrcv, ms since data last arrived on a
connection), NOT connection count. An earlier version of this script counted ESTABLISHED
connections and flagged >1 as broken — wrong: RealDash routinely holds 2-3 simultaneous
connections to the VCU during completely normal operation (confirmed via many snapshots
taken during known-healthy periods elsewhere in this project's history), so that triggered
false-positive restarts constantly, even while data was flowing fine. The signal that
actually matters is whether *any* connection has received data recently — how many
connections exist alongside it doesn't matter.

This has needed the same manual fix every time it's genuinely broken: kill both RealDash
processes, relaunch. This script automates exactly that recipe once the freshest
connection's lastrcv exceeds STALE_THRESHOLD_MS, requiring it to persist across two
consecutive checks (not just a single snapshot) for extra margin against a momentary blip.
Zero connections at all is deliberately NOT treated as stale — that's most likely just
nothing to connect to (e.g. the VCU is asleep), not a RealDash bug.

Deployed on the Odroid M2 as a systemd service — see realdash-watchdog.service in this
same directory. Runs as root (same as RealDash itself) since the recovery needs to signal
a root-owned X11 application.
"""
import re
import subprocess
import sys
import time

VCU_HOST = "192.168.10.10"  # Teensy static IP (see realdash_tcp.cpp)
VCU_PORT = 35000

CHECK_INTERVAL_S = 10
STALE_THRESHOLD_MS = 5000  # VCU sends RealDash frames every 62.5ms normally — 5s is generous
RESTART_LOG = "/root/realdash_restart.log"

LASTRCV_RE = re.compile(r"lastrcv:(\d+)")


def freshest_lastrcv_ms():
    """Return the smallest (freshest) lastrcv in ms across all connections to the VCU's
    RealDash port, or None if there are no connections at all."""
    try:
        out = subprocess.run(["ss", "-tni", "dst", f"{VCU_HOST}:{VCU_PORT}"],
                              capture_output=True, text=True, timeout=5).stdout
    except (subprocess.TimeoutExpired, OSError) as e:
        print(f"ss failed ({e}), treating as no connections this check", file=sys.stderr)
        return None
    values = [int(m) for m in LASTRCV_RE.findall(out)]
    return min(values) if values else None


def recover():
    print("Stale RealDash connection detected (persisted across two checks) — "
          "killing and relaunching RealDash")
    # NOT "-f realdash" — that's a substring match against the full command line, and this
    # script's own filename (realdash_watchdog.py) contains "realdash", so it killed itself
    # before ever reaching the relaunch step (confirmed on hardware). Anchored to match only
    # command lines actually ending in "realdash" (the two real process names,
    # "/usr/bin/realdash" and bare "realdash"), not anything with a suffix after it.
    subprocess.run(["pkill", "-f", "(^|/)realdash$"], check=False)
    time.sleep(1)
    with open(RESTART_LOG, "ab") as log:
        subprocess.Popen(
            ["realdash"],
            env={"DISPLAY": ":0"},
            stdout=log, stderr=log, stdin=subprocess.DEVNULL,
            start_new_session=True,
        )
    print("RealDash relaunched")


def main():
    stale_streak = 0
    while True:
        lastrcv = freshest_lastrcv_ms()
        if lastrcv is not None and lastrcv > STALE_THRESHOLD_MS:
            stale_streak += 1
            print(f"Freshest connection to {VCU_HOST}:{VCU_PORT} last received data "
                  f"{lastrcv}ms ago (streak={stale_streak})")
            if stale_streak >= 2:
                recover()
                stale_streak = 0
                time.sleep(5)  # give the relaunch a moment before resuming checks
        else:
            stale_streak = 0
        time.sleep(CHECK_INTERVAL_S)


if __name__ == "__main__":
    main()
