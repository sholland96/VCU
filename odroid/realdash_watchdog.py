#!/usr/bin/env python3
"""
Watches for RealDash's recurring stale-duplicate-connection bug and recovers from it
automatically. Recurring pattern seen throughout this project (see HANDOFF.md): after the
VCU (Teensy) reboots — which happens on every sleep/wake cycle, not just a fresh flash —
RealDash sometimes ends up with two simultaneous ESTABLISHED TCP connections to the VCU's
port 35000 (realdash_tcp.cpp, single-client server: "a new connection replaces the old
one" at the VCU's application layer, but the raw TCP handshake for a second connection
still completes at the stack level before the VCU's code ever calls accept() on it, since
it only does so once it thinks its current client has disconnected). RealDash's own
process appears to switch its display to the new, data-starved connection while the VCU
keeps feeding the old, orphaned one — net effect: gauges freeze.

This has needed the same manual fix every time: kill both RealDash processes, relaunch.
This script automates exactly that recipe once it detects the duplicate-connection
pattern, requiring it to persist across two consecutive checks (not just a single
snapshot) so a normal, brief connection handover during a fresh RealDash launch doesn't
trigger an unnecessary kill.

Deployed on the Odroid M2 as a systemd service — see realdash-watchdog.service in this
same directory. Runs as root (same as RealDash itself) since the recovery needs to signal
a root-owned X11 application.
"""
import subprocess
import sys
import time

VCU_HOST = "192.168.10.10"  # Teensy static IP (see realdash_tcp.cpp)
VCU_PORT = 35000

CHECK_INTERVAL_S = 10
RESTART_LOG = "/root/realdash_restart.log"


def established_connection_count():
    """Count ESTABLISHED TCP connections to the VCU's RealDash port."""
    try:
        out = subprocess.run(["ss", "-tn", "dst", f"{VCU_HOST}:{VCU_PORT}"],
                              capture_output=True, text=True, timeout=5).stdout
    except (subprocess.TimeoutExpired, OSError) as e:
        print(f"ss failed ({e}), assuming 0 connections this check", file=sys.stderr)
        return 0
    return sum(1 for line in out.splitlines() if line.startswith("ESTAB"))


def recover():
    print("Duplicate RealDash connection detected (persisted across two checks) — "
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
    duplicate_streak = 0
    while True:
        count = established_connection_count()
        if count > 1:
            duplicate_streak += 1
            print(f"Saw {count} connections to {VCU_HOST}:{VCU_PORT} "
                  f"(streak={duplicate_streak})")
            if duplicate_streak >= 2:
                recover()
                duplicate_streak = 0
                time.sleep(5)  # give the relaunch a moment before resuming checks
        else:
            duplicate_streak = 0
        time.sleep(CHECK_INTERVAL_S)


if __name__ == "__main__":
    main()
