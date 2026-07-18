/* globals.cpp
 * Variable definitions for all globals declared in defines.h.
 */
#include <Arduino.h>
#include "defines.h"  // brings in UBLOX_GNSS, library includes, and extern declarations
#ifdef UBLOX_GNSS
SFE_UBLOX_GNSS myGNSS;
#endif
Adafruit_ADS1115 ads;

// pMBB32 stale / update timestamps
uint8_t  pMBB32stale1   = 0;
uint8_t  pMBB32stale2   = 0;
uint8_t  pMBB32stale3   = 0;
uint8_t  pMBB32staleMax = 0;
uint32_t lastUpdatePMBB1 = 0;
uint32_t lastUpdatePMBB2 = 0;
uint32_t lastUpdatePMBB3 = 0;

// Misc / legacy
uint8_t  counter = 0;
float    ADCres  = 0.000076293945f;
uint8_t  serialBlockTag[] = {0x44, 0x33, 0x22, 0x11};
uint8_t  frameID[]        = {0x0c, 0x08};
uint8_t  who = 0;
uint32_t temp = 0;

// Cell voltage tracking (per module)
uint8_t  minCell1 = 0, maxCell1 = 0, minCell2 = 0, maxCell2 = 0, minCell3 = 0, maxCell3 = 0;
uint8_t  minModule1 = 0, maxModule1 = 0, minModule2 = 0, maxModule2 = 0;
uint8_t  minModule3 = 0, maxModule3 = 0;
uint16_t minCellV1 = 0xFFFF, maxCellV1 = 0;
uint16_t minCellV2 = 0xFFFF, maxCellV2 = 0;
uint16_t minCellV3 = 0xFFFF, maxCellV3 = 0;
uint16_t lowestCellV  = 0, lowestCell  = 0, lowestModule  = 0;
uint16_t highestCellV = 0, highestCell = 0, highestModule = 0;

// IVT-S-1K-U3-I-CAN1-12V
int32_t IVTpackCurrent    = 0;
int32_t IVTpackVoltage    = 0;
int32_t IVTpreChargeV     = 0;
int32_t IVTvoltage3       = 0;
int32_t IVTtemp           = 0;
int32_t IVTpower          = 0;
int32_t IVTcoulombCounter = 0;
int32_t IVTenergyCounter  = 0;

// SIM100MOD isolation monitor
uint16_t SIM100MODohmsPerVolt = 0;
uint16_t SIM100MODRpKohms     = 0;
uint16_t SIM100MODRnKohms     = 0;
uint16_t SIM100MODCpnF        = 0;
uint16_t SIM100MODCnnF        = 0;
uint16_t SIM100MODVp          = 0;
uint16_t SIM100MODVn          = 0;
uint16_t SIM100MODVb          = 0;
uint16_t SIM100MODVbMax       = 0;
uint8_t  SIMM100MODerrorFlags = 0;
uint32_t SIM100MODtemp        = 0;
uint16_t batteryVoltage       = 0;

// OpenInverter Tesla LDU V2
int32_t  LDUrpm             = 0;
int16_t  LDUtorque          = 0;
int16_t  LDUtorqueSetpoint  = 0;
int16_t  LDUspeedLimit      = 6000;
int16_t  LDUregenLimit      = 0;
uint8_t  LDUdirection       = 0;   // LDU_DIR_NEUTRAL = 0
uint8_t  LDUseqCounter      = 0;
int16_t  LDUmotorTemp       = 0;
int16_t  LDUinverterTemp    = 0;
uint16_t LDUdcVoltage       = 0;
uint16_t LDUstatus          = 0;
uint16_t LDUfaults          = 0;

// BMW changeover valve
uint8_t  valvePosition = 0;
uint8_t  valveStatus   = 0;
bool     valveOnline   = false;

// EMP WP29 pumps — inverter cooling (CAN2)
uint16_t invPumpSpeed    = 0;
uint8_t  invPumpStatus   = 0;
uint8_t  invPumpFaults   = 0;
uint8_t  invPumpSetpoint = 0;
// EMP WP29 pumps — battery cooling (CAN1)
uint16_t battPumpSpeed    = 0;
uint8_t  battPumpStatus   = 0;
uint8_t  battPumpFaults   = 0;
uint8_t  battPumpSetpoint = 0;

// Advantics EVCC
uint8_t  EVCCstage         = 0;
uint8_t  EVCCplugType      = 0;
bool     EVCCsessionActive = false;
bool     EVCCemergencyStop = false;
bool     EVCCsystemEnable  = false;
bool     normalEndOfCharge = false;

// Keypad / drive state
uint8_t  keypadStatus = 0;
uint16_t rpm          = 0;
uint16_t power        = 0;
uint16_t throttle     = 0;

// EVWest dual pot throttle
uint16_t throttlePot1Raw      = 0;
uint16_t throttlePot2Raw      = 0;
int16_t  brakeRaw             = 0;
bool     throttlePlausibility = true;
bool     brakePedal           = false;
bool     regenActive          = false;
bool     IVTfaultActive       = false;
uint32_t preChargeStartTime   = 0;
bool     chargeMode           = false;
bool     reducedPowerActive   = false;

// GPS / GNSS
uint16_t groundSpeed = 0;
uint32_t GPSaltitude = 0;
uint8_t  fixType     = 0;

// KL signals
bool KL17state = false;
bool KL15state = false;

// Button debounce
uint16_t debounceDelay    = 250;
uint8_t  button_0x01_state = 0, button_0x02_state = 0;
uint8_t  button_0x04_state = 0, button_0x08_state = 0;
uint8_t  button_0x10_state = 0, button_0x20_state = 0;
uint8_t  button_0x40_state = 0, button_0x80_state = 0;
uint8_t  new_0x01_state = 0, new_0x02_state = 0;
uint8_t  new_0x04_state = 0, new_0x08_state = 0;
uint8_t  new_0x10_state = 0, new_0x20_state = 0;
uint8_t  new_0x40_state = 0, new_0x80_state = 0;
uint8_t  last_0x01_state = 0, last_0x02_state = 0;
uint8_t  last_0x04_state = 0, last_0x08_state = 0;
uint8_t  last_0x10_state = 0, last_0x20_state = 0;
uint8_t  last_0x40_state = 0, last_0x80_state = 0;
uint8_t  last_0x01_time = 0, last_0x02_time = 0;
uint8_t  last_0x04_time = 0, last_0x08_time = 0;
uint32_t last_0x10_time = 0;
uint8_t  last_0x20_time = 0, last_0x40_time = 0, last_0x80_time = 0;

// Legacy / unused
char         ReplyBuffer[]  = "acknowledged";
uint8_t      displayBuffer[16] = {0};
unsigned int digitalPins    = 0;
int          analogPins[7]  = {0};
byte         buf[8]         = {0};
unsigned int kpa            = 992;
unsigned int tps            = 965;
unsigned int clt            = 80;
unsigned int textCounter    = 0;
