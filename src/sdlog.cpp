/* sdlog.cpp — SD card data and event logging. */
#include <Arduino.h>
#include <SdFat.h>
#include "vcu.h"
#include "sdlog.h"

#define SD_EVT_SLOTS  8
#define SD_EVT_LEN   48

static SdFs   sd;
static FsFile dataFile;
static FsFile eventFile;
static bool   sdOK = false;

volatile bool    sdLogPending = false;
static char      sdEvtBuf[SD_EVT_SLOTS][SD_EVT_LEN];
static volatile uint8_t sdEvtHead = 0;
static volatile uint8_t sdEvtTail = 0;

static const char* const vcuStateStr[] = {
    "OFF", "PRECHARGE", "IDLE", "DRIVE", "CHARGE",
    "HEAT", "COOL", "FAULT", "KL30C"
};

static void sdQueueRaw(const char *msg) {
    uint8_t next = (sdEvtHead + 1) % SD_EVT_SLOTS;
    if (next == sdEvtTail) return; // queue full — drop
    strncpy(sdEvtBuf[sdEvtHead], msg, SD_EVT_LEN - 1);
    sdEvtBuf[sdEvtHead][SD_EVT_LEN - 1] = '\0';
    sdEvtHead = next;
}

void sdLogEvent(const char *msg)      { noInterrupts(); sdQueueRaw(msg); interrupts(); }
void sdQueueEventISR(const char *msg) { sdQueueRaw(msg); }

void sdFlushAll() {
    if (!sdOK) return;
    if (dataFile)  dataFile.flush();
    if (eventFile) eventFile.flush();
}

void sdDrainEvents() {
    while (true) {
        noInterrupts();
        if (sdEvtTail == sdEvtHead) { interrupts(); break; }
        uint8_t t = sdEvtTail;
        char copy[SD_EVT_LEN];
        memcpy(copy, sdEvtBuf[t], SD_EVT_LEN);
        sdEvtTail = (t + 1) % SD_EVT_SLOTS;
        interrupts();
        if (sdOK && eventFile) {
            eventFile.printf("[%lu] %s\n", millis(), copy);
            eventFile.flush();
        }
    }
}

void sdLogData() {
    if (!sdOK || !dataFile) return;
    static uint8_t flushCtr = 0;
    dataFile.printf("%lu,%s,%ld,%ld,%ld,%u,%u,%u,%ld,%d,%u\n",
        millis(),
        vcuStateStr[VCUstate],
        IVTpackVoltage, IVTpackCurrent, IVTpreChargeV,
        highestCellV, lowestCellV,
        SIM100MODRpKohms,
        LDUrpm, (int)LDUtorqueSetpoint,
        EVCCstage);
    if (++flushCtr >= 25) { flushCtr = 0; dataFile.flush(); }
}

void sdInit() {
    if (!sd.begin(SdioConfig(FIFO_SDIO))) {
        Serial.println("[SD] init failed — no card or unsupported format");
        return;
    }
    sdOK = true;

    uint16_t session = 1;
    {
        FsFile f = sd.open("SESSION.TXT", O_RDONLY);
        if (f) { session = (uint16_t)f.parseInt() + 1; f.close(); }
    }
    {
        FsFile f = sd.open("SESSION.TXT", O_WRONLY | O_CREAT | O_TRUNC);
        if (f) { f.printf("%u\n", session); f.close(); }
    }

    char name[16];
    snprintf(name, sizeof(name), "LOG_%04u.CSV", session);
    dataFile = sd.open(name, O_WRONLY | O_CREAT | O_TRUNC);
    if (dataFile)
        dataFile.println("ms,state,packV_mV,packI_mA,preV_mV,hiCellV,loCellV,sim_kohm,rpm,throttle,evcc_stage");

    snprintf(name, sizeof(name), "EVT_%04u.TXT", session);
    eventFile = sd.open(name, O_WRONLY | O_CREAT | O_TRUNC);

    sdLogEvent("BOOT");
    Serial.printf("[SD] session %u\n", session);
}
