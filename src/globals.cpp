/* globals.cpp — storage for the extern globals declared in defines.h. */
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include "defines.h"
#ifdef UBLOX_GNSS
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#endif

Adafruit_ADS1115 ads;
#ifdef UBLOX_GNSS
SFE_UBLOX_GNSS myGNSS;
#endif

uint8_t pMBB32stale1;
uint8_t pMBB32stale2;
uint8_t pMBB32stale3;
uint8_t pMBB32staleMax;
uint32_t lastUpdatePMBB1 = 0;
uint32_t lastUpdatePMBB2 = 0;
uint32_t lastUpdatePMBB3 = 0;

uint8_t counter = 0;

float ADCres = 0.000076293945;

uint8_t serialBlockTag[] = {0x44,0x33,0x22,0x11};
uint8_t frameID[] = {0x0c,0x08};

uint8_t who;
uint32_t temp;
uint8_t minCell1;
uint8_t maxCell1;
uint8_t minCell2;
uint8_t maxCell2;
uint8_t minCell3;
uint8_t maxCell3;
uint8_t minModule1;
uint8_t maxModule1;
uint8_t minModule2;
uint8_t maxModule2;
uint8_t minModule3;
uint8_t maxModule3;
uint16_t minCellV1 = 0xFFFF;
uint16_t maxCellV1 = 0;
uint16_t minCellV2 = 0xFFFF;
uint16_t maxCellV2 = 0;
uint16_t minCellV3 = 0xFFFF;
uint16_t maxCellV3 = 0;
uint16_t lowestCellV;
uint16_t lowestCell;
uint16_t lowestModule;
uint16_t highestCellV;
uint16_t highestCell;
uint16_t highestModule;
int32_t IVTpackCurrent;//1mA resolution, signed (negative = discharge)
int32_t IVTpackVoltage;//1mV resolution
int32_t IVTpreChargeV;//1mV resolution
int32_t IVTvoltage3;//1mV resolution
int32_t IVTtemp;//0.1°C resolution, signed
int32_t IVTpower;//1W resolution, signed (negative = regen)
int32_t IVTcoulombCounter;//1As resolution, signed
int32_t IVTenergyCounter;//1Wh resolution, signed
uint16_t SIM100MODohmsPerVolt;
uint16_t SIM100MODRpKohms;
uint16_t SIM100MODRnKohms;
uint16_t SIM100MODCpnF;
uint16_t SIM100MODCnnF;
uint16_t SIM100MODVp;
uint16_t SIM100MODVn;
uint16_t SIM100MODVb;
uint16_t SIM100MODVbMax;
uint8_t SIMM100MODerrorFlags;
uint32_t SIM100MODtemp;
uint16_t batteryVoltage;

int32_t  LDUrpm;                    // motor speed (RPM)
int16_t  LDUtorque;                 // actual torque (Nm)
int16_t  LDUtorqueSetpoint = 0;     // throttle demand 0–100 % — written by readThrottle(), read by t0
int16_t  LDUspeedLimit     = 6000;  // max RPM — TODO: map via separate CAN rx if needed
int16_t  LDUregenLimit     = 0;     // regen limit — TODO: implement
uint8_t  LDUdirection      = LDU_DIR_NEUTRAL; // internal direction state, set by keypad
uint8_t  LDUseqCounter     = 0;     // 2-bit sequence counter; incremented each t0 tick
int16_t  LDUmotorTemp;              // motor temperature (°C × 10)
int16_t  LDUinverterTemp;           // inverter temperature (°C × 10)
uint16_t LDUdcVoltage;              // DC bus voltage (V × 10)
uint16_t LDUstatus;                 // status bitfield
uint16_t LDUfaults;                 // fault bitfield

uint8_t  valvePosition = 0;     // last commanded position (encoding TBD)
uint8_t  valveStatus   = 0;     // last reported status byte
bool     valveOnline   = false; // true once first valid response received

// Inverter cooling pump (CAN2)
uint16_t invPumpSpeed    = 0;  // raw measured speed (0.5 rpm/bit)
uint8_t  invPumpStatus   = 0;  // controller status nibble (byte 0 bits[5:2])
uint8_t  invPumpFaults   = 0;  // service indicator [1:0] + operation status [3:2] (byte 7)
uint8_t  invPumpSetpoint = 0;  // commanded speed 0–100 %
// Battery cooling pump (CAN1)
uint16_t battPumpSpeed    = 0;
uint8_t  battPumpStatus   = 0;
uint8_t  battPumpFaults   = 0;
uint8_t  battPumpSetpoint = 0;

bool     odroidShutdownSignaled   = false;
uint32_t odroidShutdownSignalTime = 0;

uint8_t  EVCCstage          = 0;     // Advantics_Controller_Status.State from 0x68009
uint8_t  EVCCplugType       = 0;     // Plug_and_pins from New_Charge_Session
bool     EVCCsessionActive  = false; // true from New_Charge_Session until Finished/Emergency
bool     EVCCemergencyStop  = false; // set on Emergency_Stop received
bool     EVCCsystemEnable   = false; // VCU authorisation flag → drives System_Enable in Power_Modules_Status
bool     normalEndOfCharge  = false; // set when highestCellV ≥ EVCC_CELL_V_FULL

uint8_t keypadStatus;// 0x01 = Park, 0x02 = Reverse, 0x03 = Neutral, 0x04 = Drive, 0x05 = Ignition, 0x06 = SpeedMode, 0x07 = AUX, 0x08 = DriveMode
uint16_t rpm = 0;
uint16_t power = 0;
uint16_t throttle = 0;  // 0–100 %, written by readThrottle(), consumed by displayStatus()

uint16_t throttlePot1Raw;             // ADS1115 AIN0 count, track 1 (updated in loop())
uint16_t throttlePot2Raw;             // ADS1115 AIN1 count, track 2 (updated in loop())
int16_t  brakeRaw = 0;                // ADS1115 AIN2 count (updated in loop())
bool     throttlePlausibility = true; // false = tracks disagree → throttle forced to 0
bool     brakePedal           = false;// true when brake pedal is pressed
bool     regenActive          = false;// true when LDU torque is negative and motor is spinning
bool     IVTfaultActive       = false;// set in can2Sniff on overcurrent / overvoltage
uint32_t preChargeStartTime   = 0;   // millis() when PreCharge state was entered
bool     chargeMode           = false;// true for AC charge session — routes pre-charge to Charge state (TODO: set from EVCC_NEW_SESSION when AC plug type detected)
bool     reducedPowerActive   = false;// true when BMS temp fault limits available power
uint16_t groundSpeed = 0;
uint32_t GPSaltitude = 0;
uint8_t fixType;
bool KL17state;
bool KL15state = false;  // true when keypad key 1 (Start/Stop) has been pressed to start

uint16_t debounceDelay = 250;//debounce time (ms)
uint8_t buttonStartStop;
uint8_t buttonPark;
uint8_t buttonReverse;
uint8_t buttonNeutral;
uint8_t buttonDrive;
uint8_t buttonSpeedMode;
bool    stopRequested = false;
uint8_t new_0x01_state;
uint8_t new_0x02_state;
uint8_t new_0x04_state;
uint8_t new_0x08_state;
uint8_t new_0x10_state;
uint8_t new_0x20_state;
uint8_t new_0x40_state;
uint8_t new_0x80_state;
uint8_t last_0x01_state;
uint8_t last_0x02_state;
uint8_t last_0x04_state;
uint8_t last_0x08_state;
uint8_t last_0x10_state;
uint8_t last_0x20_state;
uint8_t last_0x40_state;
uint8_t last_0x80_state;
uint8_t  last_0x01_time;
uint8_t  last_0x02_time;
uint8_t  last_0x04_time;
uint8_t  last_0x08_time;
uint32_t last_0x10_time;  // uint32_t — holds millis() for debounce
uint8_t  last_0x20_time;
uint8_t  last_0x40_time;
uint8_t  last_0x80_time;

char ReplyBuffer[] = "acknowledged";
uint8_t displayBuffer[16];

unsigned int digitalPins = 0;
int analogPins[7] = {0};

byte buf[8];

unsigned int kpa = 992; // 99.2
unsigned int tps = 965; // 96.5
unsigned int clt = 80;  // 80 - 100
unsigned int textCounter = 0;
