/*
 * pMBB32 shutdown / wake / poll test
 * Teensy 4.1 — CAN1 at 500 kbps
 *
 * Compile and flash with:  pio run -e pMBB32_test -t upload
 * Monitor with:            pio device monitor
 *
 * Phases
 *   1. Enable PDU-8 CH2 (12V to pMBB32), wait 2 s for modules to boot.
 *   2. Wake all three modules, poll 5 s, report which responded.
 *   3. For each module in turn: shutdown → 2 s wait → wake → poll 5 s → report.
 *   4. Continuous polling with a status line every 5 s.
 *
 * The PDU-8 CH2 keep-alive is sent every 100 ms throughout. Without it the
 * PDU-8 watchdog times out and kills power to the pMBB32 modules.
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>

// ── pMBB32 protocol ──────────────────────────────────────────────────────────
static const uint32_t MOD_ID[3] = { 0xAF0100, 0xAF0200, 0xAF0300 };
static const uint8_t  WAKEUP    = 0x01;
static const uint8_t  CH_COUNT  = 0x10;   // 16-cell module
static const uint8_t  NUM_DEV   = 0x02;   // 2 AFE per module
static const uint8_t  SHUTDOWN  = 0x55;
static const uint32_t TRIGGER   = 0xFF0000;

// ── CAN ──────────────────────────────────────────────────────────────────────
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// ── Receive log ring buffer (ISR → main loop) ────────────────────────────────
struct RxEntry {
    uint32_t id;
    uint32_t ts;      // millis()
    uint8_t  len;
    uint8_t  buf[8];
};
static volatile RxEntry rxBuf[256];  // power-of-2 size → natural uint8 wrap
static volatile uint8_t rxHead = 0;
static volatile uint8_t rxTail = 0;

// ── Per-module tracking (written by ISR) ─────────────────────────────────────
static volatile uint32_t lastSeen[3]    = {0, 0, 0};
static volatile uint32_t frameCount[3]  = {0, 0, 0};

// ── CAN receive ISR ──────────────────────────────────────────────────────────
static void onRx(const CAN_message_t &m) {
    uint32_t now = millis();

    // Update stale tracking for cell/temp frames (0x18FF01xx–0x18FF0Cxx)
    if ((m.id >> 16) == 0x18FF) {
        uint8_t ft  = (m.id >> 8) & 0xFF;
        uint8_t mod = m.id & 0xFF;
        if (ft >= 0x01 && ft <= 0x0C && mod >= 1 && mod <= 3) {
            lastSeen[mod - 1]   = now;
            frameCount[mod - 1]++;
        }
    }

    // Enqueue for printing — drop silently if buffer full
    uint8_t next = rxHead + 1;
    if (next != rxTail) {
        rxBuf[rxHead].id  = m.id;
        rxBuf[rxHead].ts  = now;
        rxBuf[rxHead].len = m.len;
        memcpy((void *)rxBuf[rxHead].buf, m.buf, m.len > 8 ? 8 : m.len);
        rxHead = next;
    }
}

// ── Drain & print any queued RX frames ───────────────────────────────────────
static void printRx() {
    while (rxTail != rxHead) {
        RxEntry e = *(RxEntry *)&rxBuf[rxTail];
        rxTail++;

        // Label frame source for readability
        const char *tag = "RX";
        if (e.id == 0x0A0610 || e.id == 0x0A0611)  tag = "PDU8";
        else if ((e.id >> 16) == 0x18FF)             tag = "pMBB";

        Serial.printf("[%s +%5lums] 0x%08X [%u]:", tag, e.ts, e.id, e.len);
        for (uint8_t i = 0; i < e.len; i++) Serial.printf(" %02X", e.buf[i]);
        Serial.println();
    }
}

// ── PDU-8 CH2 keep-alive ─────────────────────────────────────────────────────
static CAN_message_t pdu1;              // 0x0A0620 — output levels (refreshed every 100 ms)
static uint32_t      nextPDURefresh = 0;

static void txPDU8Enable() {
    // Output levels: CH2 = 2A (buf[1] = 0x05), all others off
    pdu1.flags.extended = 1;
    pdu1.id  = 0x0A0620;
    pdu1.len = 8;
    memset(pdu1.buf, 0, 8);
    pdu1.buf[1] = 0x05;
    can1.write(pdu1);

    // Current limit for CH2 (sent once — PDU-8 latches this)
    CAN_message_t pdu2;
    pdu2.flags.extended = 1;
    pdu2.id  = 0x0A0630;
    pdu2.len = 8;
    memset(pdu2.buf, 0, 8);
    pdu2.buf[1] = 0xFE;
    can1.write(pdu2);

    nextPDURefresh = millis() + 100;
    Serial.println("[TX] PDU-8 CH2 enabled — 0x0A0620 buf[1]=0x05");
}

static void txPDU8Refresh() {
    uint32_t now = millis();
    if (now >= nextPDURefresh) {
        can1.write(pdu1);
        nextPDURefresh = now + 100;
    }
}

// ── Transmit helpers ─────────────────────────────────────────────────────────
static void txWake(uint8_t mod) {
    CAN_message_t m;
    m.flags.extended = 1;
    m.id     = MOD_ID[mod];
    m.len    = 3;
    m.buf[0] = WAKEUP;
    m.buf[1] = CH_COUNT;
    m.buf[2] = NUM_DEV;
    can1.write(m);
    Serial.printf("[TX] Wake     → module %u (0x%06X) [%02X %02X %02X]\n",
                  mod + 1, MOD_ID[mod], WAKEUP, CH_COUNT, NUM_DEV);
}

static void txShutdown(uint8_t mod) {
    CAN_message_t m;
    m.flags.extended = 1;
    m.id     = MOD_ID[mod];
    m.len    = 1;
    m.buf[0] = SHUTDOWN;
    can1.write(m);
    Serial.printf("[TX] Shutdown → module %u (0x%06X) [%02X]\n",
                  mod + 1, MOD_ID[mod], SHUTDOWN);
}

static void txTrigger() {
    CAN_message_t m;
    m.flags.extended = 1;
    m.id  = TRIGGER;
    m.len = 0;
    can1.write(m);
}

// ── Poll for durationMs, sending trigger every 200 ms ────────────────────────
static void pollFor(uint32_t durationMs) {
    uint32_t end      = millis() + durationMs;
    uint32_t nextTrig = millis();
    while (millis() < end) {
        txPDU8Refresh();
        if (millis() >= nextTrig) { txTrigger(); nextTrig += 200; }
        printRx();
        delay(1);
    }
    printRx();  // flush any last frames
}

// ── Status report ─────────────────────────────────────────────────────────────
static void printStatus(const char *label) {
    uint32_t now = millis();
    Serial.printf("── %s ──\n", label);
    for (uint8_t i = 0; i < 3; i++) {
        uint32_t age = (lastSeen[i] > 0) ? (now - lastSeen[i]) : 99999UL;
        Serial.printf("   Module %u: %-10s  frames=%lu  last=%lums ago\n",
                      i + 1,
                      (lastSeen[i] > 0 && age < 600) ? "RESPONDING" : "SILENT",
                      frameCount[i], age);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 4000);
    delay(200);
    Serial.println("\n════════════════════════════════════");
    Serial.println(" pMBB32 shutdown / wake / poll test");
    Serial.println("════════════════════════════════════");

    can1.begin();
    can1.setBaudRate(500000);
    can1.setMaxMB(16);
    can1.enableFIFO();
    can1.enableFIFOInterrupt();
    can1.onReceive(FIFO, onRx);
    Serial.println("CAN1 ready at 500 kbps");

    // ── Enable PDU-8 CH2 (12V to pMBB32) ────────────────────────────────────
    Serial.println("\n══ Enabling PDU-8 CH2 — waiting 2 s for pMBB32 boot ══");
    txPDU8Enable();
    // Poll (and refresh PDU-8) for 2 s to let modules boot before any wake cmd
    pollFor(2000);
    Serial.println("Boot wait done.");

    // ── Phase 1: Wake all modules ─────────────────────────────────────────────
    Serial.println("\n══ Phase 1: Wake all modules ══");
    for (uint8_t i = 0; i < 3; i++) { txWake(i); delay(10); }
    txTrigger();
    Serial.println("Polling 5 s...");
    pollFor(5000);
    printStatus("Phase 1 result");

    // ── Phase 2: Shutdown / wake each module ──────────────────────────────────
    for (uint8_t mod = 0; mod < 3; mod++) {
        Serial.printf("\n══ Phase 2.%u: Module %u shutdown → wake ══\n", mod + 1, mod + 1);

        txShutdown(mod);
        Serial.println("Waiting 2 s (polling other modules)...");
        pollFor(2000);
        printStatus("After shutdown");

        txWake(mod);
        delay(10);
        txTrigger();
        Serial.println("Polling 5 s after wake...");
        pollFor(5000);
        printStatus("After wake");
    }

    Serial.println("\n══ Phase 3: Continuous polling — status every 5 s ══\n");
    Serial.println("(Reflash with main firmware when done)\n");
}

void loop() {
    static uint32_t nextTrig   = 0;
    static uint32_t nextStatus = 0;

    uint32_t now = millis();

    txPDU8Refresh();
    if (now >= nextTrig)   { txTrigger(); nextTrig = now + 200; }
    if (now >= nextStatus) { printStatus("Status"); nextStatus = now + 5000; }

    printRx();
}
