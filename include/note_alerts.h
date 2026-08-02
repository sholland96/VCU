/* note_alerts.h — Blues Wireless Notecard (NOTE-WBNA on a Notecarrier-A, I2C) alert
 * notifications. Replaces the earlier Portenta H7 + Quectel 4G wireless-gateway's SMS
 * mechanism (0xC79 over CAN3) — see README's CAN3 section for the gateway this supersedes.
 *
 * Named note_alerts.{h,cpp}, not notecard.{h,cpp}, deliberately: this project builds on a
 * case-insensitive Windows filesystem, and the Blues library's own public header is
 * Notecard.h — a same-named-but-for-case notecard.h here would collide and silently shadow
 * the real library header on #include <Notecard.h>, which is exactly what happened on first
 * attempt (every symbol from the library came back "not declared in this scope").
 */
#pragma once
#include <stdint.h>

void notecardInit();          // call once from setup() — I2C begin() + hub.set with product UID
void notecardSendAlert(uint8_t code); // mirrors the old 0xC79 SMS message codes
