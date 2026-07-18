#pragma once
/* sdlog.h — SD card logging API.
 * Data log (LOG_xxxx.CSV): one row per 200 ms via sdLogPending flag.
 * Event log (EVT_xxxx.TXT): FSM transitions + key events; ISR events buffered.
 */

// Set by callback_t2 (ISR context), cleared and consumed by loop().
extern volatile bool sdLogPending;

void sdInit();
void sdLogEvent(const char *msg);       // main-loop context (uses noInterrupts)
void sdQueueEventISR(const char *msg);  // ISR context (no lock needed)
void sdFlushAll();
void sdLogData();    // write one CSV row — call from loop() when sdLogPending set
void sdDrainEvents();// drain event queue — call from loop()
