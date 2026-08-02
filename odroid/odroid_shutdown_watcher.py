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
    # Just killing RealDash wasn't enough (confirmed on hardware: it kept coming back).
    # Root cause: nodm.service is an auto-login X session manager whose whole job is to
    # respawn the X session if it exits, and /home/ek9/.xsession.new is literally just
    # "exec /usr/bin/realdash" with no wrapper — so killing RealDash kills the entire X
    # session, which nodm dutifully restarts (NODM_MIN_SESSION_TIME in /etc/default/nodm
    # documents this exact behavior). Stopping nodm itself removes the supervisor, not just
    # the supervised app. pkill kept as a fast belt-and-suspenders in case nodm takes a
    # moment to tear the session down.
    subprocess.run(["sudo", "systemctl", "stop", "nodm"], check=False)
    subprocess.run(["sudo", "pkill", "-f", "realdash"], check=False)
    time.sleep(1)
    subprocess.run(["sudo", "shutdown", "-h", "now"], check=False)


def main():
    while True:
        try:
            with socket.create_connection((VCU_HOST, VCU_PORT), timeout=5) as sock:
                # timeout=5 above only governs the connect handshake; clear it so recv()
                # blocks indefinitely rather than tearing the connection down every 5s
                # regardless of activity (an earlier bug here).
                sock.settimeout(None)
                # The VCU (Teensy) reboots on every sleep/wake cycle (AIRCR reset) without
                # ever sending a FIN/RST — a rebooting peer can't, it has no memory of the
                # old session. Without TCP keepalive, this socket would report ESTABLISHED
                # forever and recv() would block until the heat death of the universe,
                # even though the VCU's server is long gone (confirmed on hardware: `ss -tn`
                # showed ESTAB minutes after the VCU had already reset). Keepalive actively
                # probes the connection so a dead peer gets detected in ~9-12s instead.
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 3)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
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
