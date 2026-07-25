/* realdash_tcp.h — RealDash dashboard feed over Ethernet (Teensy <-> Odroid M2, direct cable).
 *
 * Reuses the same CAN IDs/payloads already sent on physical CAN3 by displayStatus()
 * (see display.cpp), wrapped in RealDash's native "44" frame format so RealDash on the
 * Odroid can consume them directly over TCP instead of needing a CAN-USB adapter.
 * Physical CAN3 output is unaffected — this is an additional transport, not a replacement.
 */
#pragma once
#include <stdint.h>

void realdashInit();    // call once from setup() — brings up Ethernet + TCP server (skip on CAN-wake boots)
void realdashService(); // call from loop() — accepts new clients and flushes queued frames to the socket

// ISR-safe: called from displayStatus() (callback_t1() context). Only buffers the frame —
// actual network I/O happens later in realdashService(), called from loop().
void realdashQueueFrame(uint32_t canId, const uint8_t *payload, uint8_t len);
