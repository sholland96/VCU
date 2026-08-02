/* realdash_tcp.cpp — RealDash dashboard feed over Ethernet (Teensy <-> Odroid M2, direct cable).
 *
 * RealDash connects as a TCP client to this device on port 35000 and expects a stream of
 * "44" frames: 4-byte tag + 4-byte little-endian CAN ID + up to 8 bytes of payload, no CRC.
 * Only one dashboard is ever attached, so a single client slot is enough — a new incoming
 * connection replaces whatever was there before.
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
static EthernetClient realdashClient;
static bool ethernetUp = false;

// Time of the last write() that actually reported sending data — the real signal for
// "this client is genuinely alive", since realdashClient.connected() can fail to detect a
// peer that vanished without a clean FIN/RST (e.g. the Odroid rebooting, which now happens
// routinely from the relay/shutdown feature) for a long time or indefinitely. A stale
// timeout here is what actually distinguishes a dead client from a live one — NOT the mere
// arrival of a new incoming connection: RealDash is confirmed (on hardware, repeatedly) to
// routinely hold 2-3 simultaneous connections to this port as normal behavior, so treating
// every new SYN as proof the old client is dead caused constant unnecessary switching away
// from a perfectly good, actively-used connection (stop/restart data cycling).
static uint32_t lastGoodWriteMs = 0;
static const uint32_t CLIENT_STALE_TIMEOUT_MS = 5000; // real cadence is ~62.5ms, 5s is generous

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
  size_t sent = realdashClient.write(packet, sizeof(packet));
  if (sent == sizeof(packet)) lastGoodWriteMs = millis();
}

void realdashService() {
  if (!ethernetUp) return;

  bool clientStale = !realdashClient || !realdashClient.connected() ||
                      (millis() - lastGoodWriteMs > CLIENT_STALE_TIMEOUT_MS);

  EthernetClient incoming = realdashServer.accept();
  if (incoming) {
    if (clientStale) {
      // Current client is dead (or there wasn't one) — take over.
      if (realdashClient) realdashClient.stop();
      realdashClient = incoming;
      lastGoodWriteMs = millis();
      Serial.println("RealDash: client connected");
    } else {
      // Current client is still actively receiving data — RealDash routinely opens extra
      // simultaneous connections as normal behavior, not just when something's broken, so
      // a new SYN alone isn't reason enough to drop a working connection. Politely refuse.
      incoming.stop();
    }
  }

  // Drain whatever displayStatus() queued since the last loop() pass (main context only —
  // safe to touch the network stack here, unlike from callback_t1()'s ISR).
  noInterrupts();
  uint8_t count = queuedFrameCount;
  queuedFrameCount = 0;
  interrupts();

  if (count > 0 && realdashClient && realdashClient.connected()) {
    for (uint8_t i = 0; i < count; i++) {
      realdashSendFrame(queuedFrames[i].canId, queuedFrames[i].payload);
    }
  }
}
