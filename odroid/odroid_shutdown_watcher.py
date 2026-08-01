#!/usr/bin/env python3
"""
Watches a dedicated TCP link from the VCU (Teensy) for a "shut down now" signal and
triggers a graceful Armbian shutdown. Separate from RealDash's own Data Multicast feed
and from realdash_tcp's dashboard link (port 35000) entirely — that link is single-client
("a new connection replaces the old one"), so a second client watching it here would kick
RealDash's own connection off. This uses its own port instead.

Background: this project runs 12V relays for peripheral power sequencing, tied to KL15R
(the ignition key position). When KL15R goes low, the VCU signals here before cutting
power to the Odroid/VU12 relay (~15s later), so the OS gets a chance to unmount/poweroff
cleanly instead of losing power hot mid-write.

Wire protocol: any single byte received on this connection means "shut down now" — the
VCU (odroid_shutdown.cpp) sends exactly one byte (value 1) per KL15R-low cycle.

Deployed on the Odroid M2 as a systemd service — see odroid-shutdown-watcher.service in
this same directory. Requires a scoped NOPASSWD sudoers entry (see that file's header)
since this runs as a non-root user.
"""
import socket
import subprocess
import sys
import time

VCU_HOST = "192.168.10.10"  # Teensy static IP (see realdash_tcp.cpp)
VCU_PORT = 35001            # dedicated shutdown-signal port, separate from RealDash's 35000

RECONNECT_DELAY_S = 3


def trigger_shutdown():
    print("Shutdown signal received from VCU — shutting down now")
    subprocess.run(["sudo", "shutdown", "-h", "now"], check=False)


def main():
    while True:
        try:
            with socket.create_connection((VCU_HOST, VCU_PORT), timeout=5) as sock:
                print(f"Connected to VCU shutdown signal at {VCU_HOST}:{VCU_PORT}")
                while True:
                    data = sock.recv(1)
                    if not data:
                        print("VCU shutdown-signal connection closed, reconnecting...")
                        break
                    trigger_shutdown()
                    return  # system is going down — no need to keep watching
        except (ConnectionRefusedError, OSError) as e:
            print(f"VCU shutdown signal not reachable ({e}), retrying in {RECONNECT_DELAY_S}s...",
                  file=sys.stderr)
        time.sleep(RECONNECT_DELAY_S)


if __name__ == "__main__":
    main()
