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

  J *req = notecard.newRequest("hub.set");
  JAddStringToObject(req, "product", PRODUCT_UID);
  JAddStringToObject(req, "mode", "periodic");
  JAddNumberToObject(req, "outbound", 60); // sync at least hourly even without an explicit alert
  notecard.sendRequest(req);
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

void notecardSendAlert(uint8_t code) {
  J *req = notecard.newRequest("note.add");
  JAddStringToObject(req, "file", "alerts.qo"); // Blues convention: ".qo" = outbound queue
  JAddBoolToObject(req, "sync", true);
  J *body = JCreateObject();
  JAddNumberToObject(body, "code", code);
  JAddStringToObject(body, "message", alertMessage(code));
  JAddItemToObject(req, "body", body);
  notecard.sendRequest(req);
}
