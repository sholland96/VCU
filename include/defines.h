#pragma once
/* defines.h
 * Macros, types, and extern declarations.
 * Actual variable definitions live in globals.cpp.
 */

// ── Feature flags — must be defined before any #ifdef checks ─────────────────
#define UBLOX_GNSS
//#define PMBBB32_DEBUG

// ── Library includes (needed here for type declarations) ──────────────────────
#include <Adafruit_ADS1X15.h>
#ifdef UBLOX_GNSS
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#endif

// ── Enum types ────────────────────────────────────────────────────────────────
typedef enum {
  VCU_STATE_OFF = 0,
  VCU_STATE_PRECHARGE,
  VCU_STATE_IDLE,
  VCU_STATE_DRIVE,
  VCU_STATE_CHARGE,
  VCU_STATE_HEAT_PACK,
  VCU_STATE_COOL_PACK,
  VCU_STATE_FAULT,
  VCU_STATE_KL30C   // external CAN wake (EVCC/gateway) with KL15R low
} VCUStateEnum;

// ── Keypad LED color index (byte 4 of SET_LED command) ───────────────────────
#define KEYPAD_COLOR_OFF          0
#define KEYPAD_COLOR_RED          1
#define KEYPAD_COLOR_GREEN        2
#define KEYPAD_COLOR_BLUE         3
#define KEYPAD_COLOR_YELLOW       4
#define KEYPAD_COLOR_CYAN         5
#define KEYPAD_COLOR_MAGENTA      6
#define KEYPAD_COLOR_WHITE        7
#define KEYPAD_COLOR_AMBER        8
#define KEYPAD_COLOR_YELLOW_GREEN 9

// Keypad LED mode (byte 5)
#define KEYPAD_MODE_SOLID         0x01
#define KEYPAD_MODE_BLINK         0x02
#define KEYPAD_MODE_ALT_BLINK     0x03

// Keypad functional control commands (byte 2)
#define KEYPAD_CMD_SET_LED                0x01
#define KEYPAD_CMD_LIVE_BRIGHTNESS        0x03
#define KEYPAD_CMD_LIVE_BACKLIGHT_COLOR   0x1C
#define KEYPAD_CMD_BATCH_MODE             0x37
#define KEYPAD_CMD_BACKLIGHT_BRIGHTNESS   0x7B
#define KEYPAD_CMD_STATUS_LED_BRIGHTNESS  0x7C
#define KEYPAD_CMD_BACKLIGHT_COLOR        0x7D
#define KEYPAD_CMD_EVENT_TX               0x72
#define KEYPAD_CMD_PERIODIC_TX            0x71
#define KEYPAD_CMD_TX_SPEED               0x77
#define KEYPAD_CMD_BAUD_RATE              0x6F
#define KEYPAD_CMD_SOURCE_ADDR            0x70
#define KEYPAD_CMD_FACTORY_RESET          0x34

#define KEYPAD_BATCH_SINGLE           0x00
#define KEYPAD_BATCH_ENABLE           0x01

#define KEYPAD_BAUD_125K              0x00
#define KEYPAD_BAUD_250K              0x01
#define KEYPAD_BAUD_500K              0x02
#define KEYPAD_BAUD_1M                0x03

#define pMBB32powerOn 0xFE;
#define pMBB32powerOff 0;

#define ON 0xFF;// duty cycle - 100/0.392157 = 0xFF = 100%
#define OFF 0;

#define t1CallbackRate 62.5ms
#define t2CallbackRate 200ms
#define t3CallbackRate 1000ms

// ADS1115 — I2C0 (Wire, SDA=18, SCL=19), ADDR→GND = 0x48
#define ADS1115_ADDR  0x48

#ifdef UBLOX_GNSS
#define GPS_PPS_PIN       35
#define GNSS_EXTINT_PIN   33
#define CAN_STBY_PIN      32
#define CAN2_RX_PIN       0
#define PRECHARGE_EN_PIN  34
#define SIM7080_SERIAL    Serial4
#define SIM7080_BAUD      115200
#define SIM7080_PWRKEY    36
#endif

// ── Library object instances (defined in globals.cpp) ────────────────────────
extern Adafruit_ADS1115 ads;
#ifdef UBLOX_GNSS
extern SFE_UBLOX_GNSS myGNSS;
#endif

// ── pMBB32 stale / update timestamps ─────────────────────────────────────────
extern uint8_t  pMBB32stale1;
extern uint8_t  pMBB32stale2;
extern uint8_t  pMBB32stale3;
extern uint8_t  pMBB32staleMax;
extern uint32_t lastUpdatePMBB1;
extern uint32_t lastUpdatePMBB2;
extern uint32_t lastUpdatePMBB3;

// ── Misc / legacy ─────────────────────────────────────────────────────────────
extern uint8_t  counter;
extern float    ADCres;
extern uint8_t  serialBlockTag[];
extern uint8_t  frameID[];
extern uint8_t  who;
extern uint32_t temp;

// ── Cell voltage tracking (per module) ───────────────────────────────────────
extern uint8_t  minCell1, maxCell1, minCell2, maxCell2, minCell3, maxCell3;
extern uint8_t  minModule1, maxModule1, minModule2, maxModule2, minModule3, maxModule3;
extern uint16_t minCellV1, maxCellV1;
extern uint16_t minCellV2, maxCellV2;
extern uint16_t minCellV3, maxCellV3;
extern uint16_t lowestCellV,  lowestCell,  lowestModule;
extern uint16_t highestCellV, highestCell, highestModule;

// ── IVT-S-1K-U3-I-CAN1-12V ───────────────────────────────────────────────────
extern int32_t IVTpackCurrent;
extern int32_t IVTpackVoltage;
extern int32_t IVTpreChargeV;
extern int32_t IVTvoltage3;
extern int32_t IVTtemp;
extern int32_t IVTpower;
extern int32_t IVTcoulombCounter;
extern int32_t IVTenergyCounter;

// ── SIM100MOD isolation monitor ──────────────────────────────────────────────
extern uint16_t SIM100MODohmsPerVolt;
extern uint16_t SIM100MODRpKohms;
extern uint16_t SIM100MODRnKohms;
extern uint16_t SIM100MODCpnF;
extern uint16_t SIM100MODCnnF;
extern uint16_t SIM100MODVp;
extern uint16_t SIM100MODVn;
extern uint16_t SIM100MODVb;
extern uint16_t SIM100MODVbMax;
extern uint8_t  SIMM100MODerrorFlags;
extern uint32_t SIM100MODtemp;
extern uint16_t batteryVoltage;

// ── OpenInverter Tesla LDU V2 ────────────────────────────────────────────────
#define LDU_CMD_ID          0x201
#define LDU_CANIO_CRUISE    (1 << 0)
#define LDU_CANIO_START     (1 << 1)
#define LDU_CANIO_BRAKE     (1 << 2)
#define LDU_CANIO_FORWARD   (1 << 3)
#define LDU_CANIO_REVERSE   (1 << 4)
#define LDU_CANIO_BMS       (1 << 5)
#define LDU_DIR_NEUTRAL     0
#define LDU_DIR_FORWARD     1
#define LDU_DIR_REVERSE     2
#define LDU_DIR_STOP        3

extern int32_t  LDUrpm;
extern int16_t  LDUtorque;
extern int16_t  LDUtorqueSetpoint;
extern int16_t  LDUspeedLimit;
extern int16_t  LDUregenLimit;
extern uint8_t  LDUdirection;
extern uint8_t  LDUseqCounter;
extern int16_t  LDUmotorTemp;
extern int16_t  LDUinverterTemp;
extern uint16_t LDUdcVoltage;
extern uint16_t LDUstatus;
extern uint16_t LDUfaults;

// ── BMW i4/i5/i7/iX Changeover Valve 64119462114 ────────────────────────────
#define LIN_BAUD      19200
#define LIN_VALVE_ID  0x10

extern uint8_t  valvePosition;
extern uint8_t  valveStatus;
extern bool     valveOnline;

// ── EMP WP29-12V-CV-A Smart Flow Water Pump ──────────────────────────────────
#define EMP_WP29_ADDR    0x8Au
#define VCU_CAN_ADDR     0xA3u
#define EMP_WP29_CMD_ID  (0x18EF0000UL | ((uint32_t)EMP_WP29_ADDR << 8) | VCU_CAN_ADDR)
#define EMP_WP29_STATUS1 (0x18FF0300UL | EMP_WP29_ADDR)
#define EMP_WP29_STATUS3 (0x18FF2400UL | EMP_WP29_ADDR)

extern uint16_t invPumpSpeed;
extern uint8_t  invPumpStatus;
extern uint8_t  invPumpFaults;
extern uint8_t  invPumpSetpoint;
extern uint16_t battPumpSpeed;
extern uint8_t  battPumpStatus;
extern uint8_t  battPumpFaults;
extern uint8_t  battPumpSetpoint;

// ── Advantics ADM-CS-EVCC ─────────────────────────────────────────────────────
#define EVCC_NEW_SESSION    0x68001u
#define EVCC_INS_TEST       0x68002u
#define EVCC_PRECHARGE      0x68003u
#define EVCC_STATUS_CHANGE  0x68004u
#define EVCC_CHARGING_LOOP  0x68005u
#define EVCC_EMERG_STOP     0x68006u
#define EVCC_SESSION_END    0x68007u
#define EVCC_CTRL_STATUS    0x68009u
#define EVCC_PWR_STATUS     0x60010u
#define EVCC_PWR_LIMITS     0x60011u
#define EVCC_SEQ_CTRL       0x60012u
#define EVCC_EVSE_INFO      0x600u
#define EVCC_AC_CTRL        0x601u
#define EVCC_AC_STATUS      0x611u
#define EVCC_PLUG_CCS_DC_CORE  0u
#define EVCC_PLUG_CCS_DC_EXT   1u
#define EVCC_PLUG_CHADEMO      2u
#define EVCC_PLUG_IS_DC(p) ((p) == EVCC_PLUG_CCS_DC_CORE || \
                            (p) == EVCC_PLUG_CCS_DC_EXT  || \
                            (p) == EVCC_PLUG_CHADEMO)
#define EVCC_PINS_CCS_AC       1u
#define EVCC_PINS_CCS_AC_1PH   2u
#define EVCC_PINS_CCS_AC_3PH   3u
#define EVCC_PINS_CCS_DC_CORE  4u
#define EVCC_PINS_CCS_DC_EXT   5u
#define EVCC_PINS_MCS          6u
#define EVCC_PINS_IS_AC(p)     ((p) >= 1u && (p) <= 3u)
#define EVCC_MAX_VOLTAGE_x10  4200u
#define EVCC_MAX_CURRENT_x10  1000u
#define EVCC_CELL_V_EMPTY  39322u
#define EVCC_CELL_V_FULL   47841u

extern uint8_t  EVCCstage;
extern uint8_t  EVCCplugType;
extern bool     EVCCsessionActive;
extern bool     EVCCemergencyStop;
extern bool     EVCCsystemEnable;
extern bool     normalEndOfCharge;

// ── Keypad / drive state ──────────────────────────────────────────────────────
extern uint8_t  keypadStatus;
extern uint16_t rpm;
extern uint16_t power;
extern uint16_t throttle;

// ── EVWest dual pot throttle ──────────────────────────────────────────────────
#define ADS_CH_THROTTLE1          0
#define ADS_CH_THROTTLE2          1
#define ADS_CH_BRAKE              2
#define THROTTLE_POT1_MIN         800
#define THROTTLE_POT1_MAX         31200
#define THROTTLE_POT2_MIN         800
#define THROTTLE_POT2_MAX         31200
#define THROTTLE_PLAUSIBILITY_PCT 5
#define THROTTLE_FAULT_LIMIT      20
#define BRAKE_THRESHOLD           1600
#define PRECHARGE_TIMEOUT_MS      2000

extern uint16_t throttlePot1Raw;
extern uint16_t throttlePot2Raw;
extern int16_t  brakeRaw;
extern bool     throttlePlausibility;
extern bool     brakePedal;
extern bool     regenActive;
extern bool     IVTfaultActive;
extern uint32_t preChargeStartTime;
extern bool     chargeMode;
extern bool     reducedPowerActive;

// ── GPS / GNSS ────────────────────────────────────────────────────────────────
extern uint16_t groundSpeed;
extern uint32_t GPSaltitude;
extern uint8_t  fixType;

// ── KL signals ───────────────────────────────────────────────────────────────
extern bool KL17state;
extern bool KL15state;

// ── Button debounce ───────────────────────────────────────────────────────────
extern uint16_t debounceDelay;
extern uint8_t  button_0x01_state, button_0x02_state, button_0x04_state, button_0x08_state;
extern uint8_t  button_0x10_state, button_0x20_state, button_0x40_state, button_0x80_state;
extern uint8_t  new_0x01_state, new_0x02_state, new_0x04_state, new_0x08_state;
extern uint8_t  new_0x10_state, new_0x20_state, new_0x40_state, new_0x80_state;
extern uint8_t  last_0x01_state, last_0x02_state, last_0x04_state, last_0x08_state;
extern uint8_t  last_0x10_state, last_0x20_state, last_0x40_state, last_0x80_state;
extern uint8_t  last_0x01_time, last_0x02_time, last_0x04_time, last_0x08_time;
extern uint32_t last_0x10_time;
extern uint8_t  last_0x20_time, last_0x40_time, last_0x80_time;

// ── Legacy / unused ───────────────────────────────────────────────────────────
extern char         ReplyBuffer[];
extern uint8_t      displayBuffer[16];
extern unsigned int digitalPins;
extern int          analogPins[7];
extern byte         buf[8];
extern unsigned int kpa, tps, clt, textCounter;
