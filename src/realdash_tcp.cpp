/* realdash_tcp.cpp — RealDash dashboard feed over Ethernet (Teensy <-> Odroid M2, direct cable).
 *
 * RealDash connects as a TCP client to this device on port 35000 and expects a stream of
 * "44" frames: 4-byte tag + 4-byte little-endian CAN ID + up to 8 bytes of payload, no CRC.
 *
 * Only one dashboard is ever really attached, but figuring out *in software* which held
 * connection is "the real one" turned out to be unreliable in both directions — tried and
 * discarded on hardware:
 *   - Single client slot, only replaced once !connected(): fails to detect a peer that
 *     vanished without a clean FIN/RST (e.g. the Odroid rebooting, which now happens
 *     routinely from the relay/shutdown feature) — the VCU keeps "feeding" a dead
 *     connection forever and never accepts RealDash's fresh reconnect attempt.
 *   - Single client slot, always replaced by any new incoming connection: RealDash is
 *     confirmed (repeatedly, on hardware) to routinely hold 2-3 simultaneous connections
 *     to this port as normal behavior, so this fired on completely healthy state and
 *     killed a perfectly good, actively-used connection — visible as data repeatedly
 *     stopping and restarting.
 *   - Single client slot, replaced only once local write() had failed for a while: doesn't
 *     work either — TCP write() typically just queues into the local send buffer and can
 *     keep "succeeding" for a long time even to a peer that's completely gone, since
 *     nothing forces an ACK-based liveness check on the timescale that matters here.
 *
 * Sidesteps the whole question instead: hold a small pool of connections and broadcast the
 * same frames to all of them. It doesn't matter which one RealDash is actually reading from
 * — it'll get fed regardless — and a stale/orphaned connection sitting in the pool alongside
 * the real one is harmless (small fixed memory cost, lwIP is configured for 8 total TCP PCBs
 * on this stack, comfortably more than needed here).
 */
#include <Arduino.h>
#include <QNEthernet.h>
#include "defines.h"
#include "realdash_tcp.h"

using namespace qindesign::network;

// No DHCP server on this direct point-to-point cable — both ends use fixed static IPs
// (same addresses already proven on hardware by the ethernet_test bring-up sketch).
static const IPAddress VCU_IP(192, 168, 10, 10);
static const IPAddress SUBNET_MASK(255, 255, 255, 0);
static const IPAddress ODROID_IP(192, 168, 10, 1);
static const uint16_t REALDASH_TCP_PORT = 35000;

static EthernetServer realdashServer(REALDASH_TCP_PORT);
static bool ethernetUp = false;

// Small connection pool, broadcast to — see file header for why this replaced a
// single-client-slot design.
static const uint8_t MAX_REALDASH_CLIENTS = 3;
static EthernetClient realdashClients[MAX_REALDASH_CLIENTS];

// displayStatus() runs from callback_t1(), a hardware-timer ISR (same category as
// callback_t2(), where SD-card I/O was already found to be unsafe — see sdLogPending in
// main.cpp/sdlog.cpp). QNEthernet's TCP stack is likewise not ISR-safe, so frames queued by
// realdashQueueFrame() are only actually written to the socket from realdashService() in
// loop(). Fixed-size, one slot per known frame ID — no dynamic allocation from ISR context.
struct QueuedFrame {
  uint32_t canId;
  uint8_t  payload[8];
};
static const uint8_t MAX_QUEUED_FRAMES = 4;
static QueuedFrame queuedFrames[MAX_QUEUED_FRAMES];
static volatile uint8_t queuedFrameCount = 0;

void realdashInit() {
  // Static IP config only configures the interface — does not block waiting on DHCP,
  // so this is safe to call even on the CAN-wake fast-response path. Callers skip it
  // anyway (see init.cpp) to keep that path's behavior identical to before this feature.
  if (!Ethernet.begin(VCU_IP, SUBNET_MASK, ODROID_IP)) {
    Serial.println("RealDash: Ethernet.begin() failed");
    return;
  }
  realdashServer.begin();
  ethernetUp = true;
  Serial.println("RealDash: TCP server listening on port 35000");
}

// ISR-safe: called from callback_t1() via displayStatus(). Only ever buffers bytes —
// must never touch the network stack directly.
void realdashQueueFrame(uint32_t canId, const uint8_t *payload, uint8_t len) {
  if (queuedFrameCount >= MAX_QUEUED_FRAMES) return; // loop() hasn't drained yet — drop, don't block
  QueuedFrame &f = queuedFrames[queuedFrameCount];
  f.canId = canId;
  memcpy(f.payload, payload, len < 8 ? len : 8);
  if (len < 8) memset(f.payload + len, 0, 8 - len);
  queuedFrameCount++;
}

static void realdashSendFrame(uint32_t canId, const uint8_t *payload) {
  uint8_t packet[16];
  memcpy(packet, serialBlockTag, 4); // {0x44,0x33,0x22,0x11} — RealDash "44" frame tag
  packet[4] = (uint8_t)(canId);
  packet[5] = (uint8_t)(canId >> 8);
  packet[6] = (uint8_t)(canId >> 16);
  packet[7] = (uint8_t)(canId >> 24);
  memcpy(packet + 8, payload, 8);
  for (uint8_t i = 0; i < MAX_REALDASH_CLIENTS; i++) {
    if (realdashClients[i] && realdashClients[i].connected()) {
      realdashClients[i].write(packet, sizeof(packet));
    }
  }
}

void realdashService() {
  if (!ethernetUp) return;

  // Adopt any newly-arrived connection into the first free (or dead) slot. If every slot is
  // genuinely occupied by a live connection already (all MAX_REALDASH_CLIENTS in active
  // use — not expected in practice, only one real dashboard is ever attached), evict slot 0
  // rather than silently dropping the new connection.
  EthernetClient incoming = realdashServer.accept();
  if (incoming) {
    uint8_t slot = 0;
    for (uint8_t i = 0; i < MAX_REALDASH_CLIENTS; i++) {
      if (!realdashClients[i] || !realdashClients[i].connected()) { slot = i; break; }
    }
    if (realdashClients[slot]) realdashClients[slot].stop();
    realdashClients[slot] = incoming;
    Serial.printf("RealDash: client connected (slot %u)\n", slot);
  }

  // Drain whatever displayStatus() queued since the last loop() pass (main context only —
  // safe to touch the network stack here, unlike from callback_t1()'s ISR).
  noInterrupts();
  uint8_t count = queuedFrameCount;
  queuedFrameCount = 0;
  interrupts();

  for (uint8_t i = 0; i < count; i++) {
    realdashSendFrame(queuedFrames[i].canId, queuedFrames[i].payload);
  }
}
