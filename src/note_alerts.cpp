/* note_alerts.cpp — see note_alerts.h.
 *
 * Notecard syncs small JSON "Notes" to Notehub.io rather than sending SMS directly — actual
 * phone delivery needs a Notehub route configured separately (e.g. to Twilio). This side only
 * needs to get the alert to Notehub reliably; "sync":true on note.add forces an immediate sync
 * for these event-driven alerts instead of waiting on the periodic outbound schedule, since
 * they're infrequent and time-sensitive (fault conditions, KL15R state changes).
 */
#include <Arduino.h>
#include <Notecard.h>
#include "note_alerts.h"

static Notecard notecard;

// Notehub.io project product UID — see https://notehub.io project settings.
static const char *PRODUCT_UID = "com.gmail.stephen.holland:ek9_vcu";

void notecardInit() {
  notecard.begin(); // I2C, default address 0x17, Wire (shared with GNSS/ADS1115 on I2C0)
  notecard.setDebugOutputStream(Serial); // verbose request/response logging while bringing this up

  J *req = notecard.newRequest("hub.set");
  JAddStringToObject(req, "product", PRODUCT_UID);
  JAddStringToObject(req, "mode", "periodic");
  JAddNumberToObject(req, "outbound", 60); // sync at least hourly even without an explicit alert
  Serial.printf("Notecard hub.set: %s\n", notecard.sendRequest(req) ? "OK" : "FAIL");
}

// Mirrors the old 0xC79 SMS message codes — see README's "0xC79 SMS message codes" table.
static const char *alertMessage(uint8_t code) {
  switch (code) {
    case 0:  return "KL15R on";
    case 1:  return "KL15C on";
    case 2:  return "Pre-charge failed";
    case 3:  return "Something happened";
    case 4:  return "Charging stopped";
    case 5:  return "Temperature warning";
    default: return "Invalid request";
  }
}

// No direct serial adapter to the Notecard itself, so this is the only visibility into
// whether it's actually connected/syncing over cellular — hub.set/note.add succeeding just
// means the local I2C request was accepted, not that anything reached Notehub. Rate-limited
// internally; call from loop() (NOT an ISR context — this is a request/response I2C
// transaction and may take a while, same reasoning as why SD/Ethernet I/O only happens
// from loop() elsewhere in this project).
void notecardCheckStatus() {
  static uint32_t lastCheck = 0;
  if (millis() - lastCheck < 60000) return; // at most once/minute
  lastCheck = millis();

  J *rsp = notecard.requestAndResponse(notecard.newRequest("hub.status"));
  if (rsp == NULL) {
    Serial.println("Notecard hub.status: no response");
    return;
  }
  // Full JSON already printed via the debug output stream set in notecardInit() — this is
  // just a quick human-readable summary on top of that.
  Serial.printf("Notecard hub.status summary: connected=%s status=\"%s\"\n",
                JGetBool(rsp, "connected") ? "yes" : "no",
                JGetString(rsp, "status"));
  notecard.deleteResponse(rsp);

  // card.wireless shows which SIM is actually selected (built-in eSIM vs an installed
  // external SIM) and signal/registration detail — more specific than hub.status alone.
  // Field names vary more than hub.status's, so rely on the full raw JSON already printed
  // via the debug stream rather than guessing at a brittle summary here.
  Serial.println("Notecard card.wireless:");
  J *wrsp = notecard.requestAndResponse(notecard.newRequest("card.wireless"));
  if (wrsp) notecard.deleteResponse(wrsp);
}

void notecardSendAlert(uint8_t code) {
  J *req = notecard.newRequest("note.add");
  JAddStringToObject(req, "file", "alerts.qo"); // Blues convention: ".qo" = outbound queue
  JAddBoolToObject(req, "sync", true);
  J *body = JCreateObject();
  JAddNumberToObject(body, "code", code);
  JAddStringToObject(body, "message", alertMessage(code));
  JAddItemToObject(req, "body", body);
  bool ok = notecard.sendRequest(req);
  Serial.printf("Notecard alert \"%s\": %s\n", alertMessage(code), ok ? "sent" : "FAILED");

  // TEMPORARY DIAGNOSTIC — "sync":true above should force an immediate sync, but the
  // modem was still showing {modem-off} a minute later even after this succeeded.
  // Explicitly force hub.sync and watch card.wireless every 2s for the next ~14s, to see
  // whether the modem actually attempts to power on at all right after being asked, rather
  // than waiting on the sparse 60s-interval background check to maybe catch it. Blocking
  // is acceptable here — this only runs once per Idle/Fault entry, not continuously — but
  // remove this whole block once cellular connectivity is confirmed working.
  Serial.printf("Notecard hub.sync: %s\n",
                notecard.sendRequest(notecard.newRequest("hub.sync")) ? "requested" : "FAILED");
  for (uint8_t i = 0; i < 7; i++) {
    delay(2000);
    Serial.printf("Notecard card.wireless (+%us):\n", (i + 1) * 2);
    J *wrsp = notecard.requestAndResponse(notecard.newRequest("card.wireless"));
    if (wrsp) notecard.deleteResponse(wrsp);
  }
}
