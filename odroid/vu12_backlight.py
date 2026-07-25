#!/usr/bin/env python3
"""
Reads RealDash's Data Multicast stream (TCP, raw binary — 8-byte records: 1-byte
targetId/ECU-specific ID + 3 bytes padding + 4-byte little-endian float32 value)
and writes the VU12 display's backlight brightness to its control serial port
whenever the "VCU: Backlight" value changes.

Background: this value started as a bash-script design polling RealDash's HTTP
API, which doesn't exist. RealDash's actual external-data mechanism is "Data
Multicast" (raw TCP, port 6558 by default, binary protocol, no JSON/HTTP at
all). Its exported-value list is fixed to RealDash's own standard fields —
custom "Dummy"/name-based values aren't exportable UNLESS explicitly added as
an "ECU Specific value" first, which is what makes "VCU: Backlight" (defined
in dbc/realdash_vcu.xml, frame 0xC90) show up here at all. Confirmed on
hardware: ID 169 (0xA9).

Requires: RealDash's "Use Data Multicast" enabled, mode "TCP/IP Server",
default port 6558, and "VCU: Backlight" added under ECU Specific + checked in
the multicast value list.

Deployed on the Odroid M2 as a systemd service — see vu12-backlight.service in
this same directory.
"""
import serial
import socket
import struct
import time
import sys

SERIAL_PORT = "/dev/ttyACM1"  # VU12 control interface (WCH USB-serial) — confirmed via
                              # /dev/serial/by-id/usb-wch.cn_USB_Serial-if00, NOT ttyACM0
                              # (that's the u-blox GNSS receiver).
SERIAL_BAUD = 115200

MULTICAST_HOST = "127.0.0.1"
MULTICAST_PORT = 6558
BACKLIGHT_ID = 169  # 0xA9 — "VCU: Backlight", confirmed via live capture

RECORD_SIZE = 8  # 1-byte id + 3 bytes padding + 4-byte float32 LE
RAW_MIN = 25     # ~10% — safety floor, defense-in-depth alongside RealDash's own
                 # rangeMin=25 on this field (dbc/realdash_vcu.xml)
RAW_MAX = 255

RECONNECT_DELAY_S = 3


def open_serial():
    try:
        return serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    except serial.SerialException as e:
        print(f"Warning: could not open {SERIAL_PORT}: {e}", file=sys.stderr)
        return None


def send_brightness(ser, raw_value):
    raw_value = max(RAW_MIN, min(RAW_MAX, int(round(raw_value))))
    frame = f"@B{raw_value:03d}#".encode("ascii")
    if ser is not None:
        try:
            ser.write(frame)
        except serial.SerialException as e:
            print(f"Warning: serial write failed: {e}", file=sys.stderr)
    print(f"Backlight -> {frame!r} (raw={raw_value})")
    return raw_value


def read_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None  # connection closed
        buf += chunk
    return buf


def main():
    ser = open_serial()
    last_raw = None

    while True:
        try:
            with socket.create_connection((MULTICAST_HOST, MULTICAST_PORT), timeout=5) as sock:
                print(f"Connected to Data Multicast at {MULTICAST_HOST}:{MULTICAST_PORT}")
                while True:
                    record = read_exact(sock, RECORD_SIZE)
                    if record is None:
                        print("Multicast connection closed, reconnecting...")
                        break
                    id_byte = record[0]
                    if id_byte != BACKLIGHT_ID:
                        continue
                    value = struct.unpack("<f", record[4:8])[0]
                    raw_value = int(round(value))
                    if raw_value != last_raw:
                        last_raw = send_brightness(ser, raw_value)
        except (ConnectionRefusedError, OSError) as e:
            print(f"Data Multicast not reachable ({e}), retrying in {RECONNECT_DELAY_S}s...",
                  file=sys.stderr)
        if ser is None:
            ser = open_serial()
        time.sleep(RECONNECT_DELAY_S)


if __name__ == "__main__":
    main()
