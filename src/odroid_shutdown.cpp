/* odroid_shutdown.cpp — see odroid_shutdown.h. Called only from loop() context (the KL15R
 * relay logic in main.cpp), so direct QNEthernet calls here are safe — no ISR involved.
 */
#include <Arduino.h>
#include <QNEthernet.h>
#include "odroid_shutdown.h"

using namespace qindesign::network;

static const uint16_t ODROID_SHUTDOWN_PORT = 35001;
static EthernetServer shutdownServer(ODROID_SHUTDOWN_PORT);
static EthernetClient shutdownClient;

void odroidShutdownInit() {
  // Ethernet.begin() is already called by realdashInit() — same interface, just a second
  // server/port on it.
  shutdownServer.begin();
  Serial.println("Odroid shutdown signal: TCP server listening on port 35001");
}

void odroidShutdownService() {
  if (!shutdownClient || !shutdownClient.connected()) {
    EthernetClient incoming = shutdownServer.accept();
    if (incoming) {
      shutdownClient.stop();
      shutdownClient = incoming;
      Serial.println("Odroid shutdown signal: watcher connected");
    }
  }
}

void odroidShutdownSignal() {
  if (shutdownClient && shutdownClient.connected()) {
    shutdownClient.write((uint8_t)1);
    Serial.println("Odroid shutdown signal: sent");
  } else {
    Serial.println("Odroid shutdown signal: no watcher connected — nothing to send");
  }
}
