/* defines.h
*
* Macros, types, and extern declarations. Actual variable/object
* definitions with their initializers live in globals.cpp.
*/

typedef enum {
  VCU_STATE_OFF = 0,
  VCU_STATE_PRECHARGE,
  VCU_STATE_IDLE,
  VCU_STATE_DRIVE,
  VCU_STATE_CHARGE,
  VCU_STATE_HEAT_PACK,
  VCU_STATE_COOL_PACK,
  VCU_STATE_FAULT,
  VCU_STATE_KL15C   // external CAN wake (EVCC/gateway) with KL15R low
} VCUStateEnum;

extern VCUStateEnum VCUstate; // defined in main.cpp

#include <FlexCAN_T4.h>

// Shared CAN bus/message objects — defined in main.cpp.
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
extern CAN_message_t msg1;
extern CAN_message_t msg2;
extern CAN_message_t msg3;
extern CAN_message_t PDUmsg1;
extern CAN_message_t PDUmsg2;
extern CAN_message_t LDUmsg;

// pMBB32 ghost-SA / frame-type tracking — defined in main.cpp, set by can1Sniff,
// consumed/reset by callback_t2.
extern volatile bool     pMBB32ghostSA;
extern volatile uint16_t pMBB32ftSeen[3];

// Wireless gateway status request (0xC84) — set by can3Sniff(), consumed/answered
// (0xC85 response) by callback_t2(). See README CAN3 section for wake/retry design.
extern volatile bool     gatewayStatusRequestPending;
// Set by callback_t2() right after sending the 0xC85 response; consumed by loop(), which
// sleeps promptly instead of waiting out the normal KLR debounce (enterSleep() calls SD
// card I/O, which must not run from callback_t2()'s ISR context).
extern volatile bool     gatewayResponseSent;

#include "Fsm.h"
extern Fsm fsm; // defined in main.cpp

// FSM State objects — defined in main.cpp, wired up with add_transition() in init.cpp's setup().
extern State state_Off;
extern State state_PreCharge;
extern State state_Idle;
extern State state_Drive;
extern State state_Charge;
extern State state_HeatPack;
extern State state_CoolPack;
extern State state_Fault;
extern State state_KL15C;

extern bool extWakePending; // set in setup() when CAN wake detected; consumed by FSM
extern uint32_t klrLowSince;  // reset in Off_enter() — start of Off-state KLR debounce
extern uint32_t klrHighSince; // mirror of klrLowSince — debounces KL15R read-as-high, so
                               // relay-driver switching noise on the sense line can't
                               // spuriously re-energize RELAY_ODROID_PIN right after a
                               // genuine key-off cuts it
extern bool gnssInitialized; // set in setup() only if GNSS bring-up actually ran this boot

// FSM events — unique integers required by arduino-fsm
#define KL15_ON          1
#define KL15_OFF         2
#define DRIVE_ON         3
#define DRIVE_OFF        4
#define CHARGE_ON        5
#define CHARGE_OFF       6
#define PRECHARGE_OK     7
#define PRECHARGE_CHARGE 8  // pre-charge succeeded via EVCC path → go to Charge
#define PRECHARGE_FAIL   9
#define FAULT_EV         10
#define FAULT_CLEAR      11
#define TEMP_LOW         12
#define TEMP_HIGH        13
#define TEMP_OK          14
#define EXT_WAKE         15  // CAN/EVCC external wake without KL15R
#define AC_CHARGE_START  16  // AC plug detected in KL15C → close main contactors via PreCharge

#define KL15C_SLEEP_TIMEOUT_MS 60000UL  // sleep after 60 s of inactivity in KL15C

// Cross-module state flags — defined in main.cpp.
extern uint32_t lastExtActivityMs; // last EVCC heartbeat or gateway frame (for KL15C timeout)
extern bool     kl15cKL15Rstate;   // tracks KL15R pin state inside KL15C for LED edge detect
extern bool     evccIsACSession;   // true when plug type is AC → VCU closes main contactors
extern bool     acReadyToDeliver;  // set by AC_Control (0x601); EVCC grants AC power delivery

#include <TeensyTimerTool.h>
// Shared timer objects — defined in main.cpp.
extern IntervalTimer t0;
extern TeensyTimerTool::PeriodicTimer t1, t2, t3;

#define KL15R_PIN 2  // physical key position 1 (accessory) — wakes hardware, no FSM role

// SNVS_LPGPR0 (battery-backed low-power general-purpose register) carries the wake-cause
// flag across enterSleep()'s AIRCR reset. Used in enterSleep() and setup() to detect
// CAN-wake vs key-on. Plain DMAMEM/OCRAM was tried first but did not reliably retain its
// value through this STOP-mode config (confirmed on hardware — same wake path, same
// garbage readback, regardless of wake source); SNVS_LPGPR0 is purpose-built for this.
#define SLEEP_MAGIC_CAN_WAKE 0xC4A8B3E1UL

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
#define KEYPAD_CMD_SET_LED                0x01  // set individual button LED color/mode
#define KEYPAD_CMD_LIVE_BRIGHTNESS        0x03  // live brightness command
#define KEYPAD_CMD_LIVE_BACKLIGHT_COLOR   0x1C  // live global backlight color (byte 3: color index)
#define KEYPAD_CMD_BATCH_MODE             0x37  // switch single vs batch LED mode
#define KEYPAD_CMD_BACKLIGHT_BRIGHTNESS   0x7B  // global backlight brightness (byte 3: 0x00–0x3F)
#define KEYPAD_CMD_STATUS_LED_BRIGHTNESS  0x7C  // button status LED brightness (byte 3: 0x00–0x3F)
#define KEYPAD_CMD_BACKLIGHT_COLOR        0x7D  // global backlight color (byte 3: color index)
#define KEYPAD_CMD_EVENT_TX               0x72  // toggle event-driven transmissions
#define KEYPAD_CMD_PERIODIC_TX            0x71  // toggle periodic heartbeat transmission
#define KEYPAD_CMD_TX_SPEED               0x77  // periodic TX interval (byte 3: value × 10ms)
#define KEYPAD_CMD_BAUD_RATE              0x6F  // change CAN baud rate
#define KEYPAD_CMD_SOURCE_ADDR            0x70  // change device source address
#define KEYPAD_CMD_FACTORY_RESET          0x34  // factory reset (byte 3: 0x01 to confirm)

// PKP1600SI 6-button keypad key numbers (replaces the earlier 8-button pad — see
// dbc/PKP1600SI_J1939.dbc). Key Contact state messages (command 01h) use these in byte 3.
#define KEYPAD_KEY_START_STOP  1
#define KEYPAD_KEY_PARK        2
#define KEYPAD_KEY_REVERSE     3
#define KEYPAD_KEY_NEUTRAL     4
#define KEYPAD_KEY_DRIVE       5
#define KEYPAD_KEY_SPEED_MODE  6

// Batch LED mode selections (byte 3 for KEYPAD_CMD_BATCH_MODE)
#define KEYPAD_BATCH_SINGLE           0x00  // standard single-LED mode (default)
#define KEYPAD_BATCH_ENABLE           0x01  // batch LED mode

// CAN baud rate selections (byte 3 for KEYPAD_CMD_BAUD_RATE)
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

#define UBLOX_GNSS
//#define PMBBB32_DEBUG  // print all 0x18FF__ CAN1 frames + stale counters to Serial

// ADS1115 16-bit 4-channel ADC — I2C0 (Wire, SDA=18, SCL=19), ADDR pin → GND = 0x48.
// Channels: AIN0=throttle pot 1, AIN1=throttle pot 2, AIN2=brake pedal, AIN3=spare.
// Gain: GAIN_ONE (±4.096 V) — suitable for 3.3V-referenced sensors.
// If sensors are 5V-referenced change to GAIN_TWOTHIRDS (±6.144 V) and re-calibrate.
#include <Adafruit_ADS1X15.h>
extern Adafruit_ADS1115 ads;
#define ADS1115_ADDR  0x48

#ifdef UBLOX_GNSS
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// SK Pang board: originally NEO-M8M, swapped to NEO-M8U, on I2C0 — Wire (SDA=pin 18, SCL=pin 19)
// 1PPS output from the module is routed to Teensy pin 35 on the SK Pang board.
// EXTINT header on SK Pang board wired to this Teensy output — rising edge
// wakes the module from backup mode before AIRCR reset.
#define GPS_PPS_PIN       35
#define GNSS_EXTINT_PIN   33  // free GPIO — wire to module's EXTINT header on SK Pang board
// NEO-M8U swap: module doesn't reliably start its I2C/UART interface on power-on alone the
// way the M8M did — confirmed on hardware that toggling !RST after power-up brings it up
// every time. Bodge-wired from this free Teensy GPIO to JP6 pin 1 (RESET) on the SK Pang
// board, which breaks out the module's RST pin (not connected to anything by default).
#define GNSS_RESET_PIN    9

// 12V relay power control — VCU-switched power to peripheral groups, staged relative to
// KL15R/EVCC state (see loop() for the actual on/off logic).
#define RELAY_PDU_PIN     10  // PDU-8, IVT-S, SIM100MOD, keypad
#define RELAY_ODROID_PIN  11  // Odroid M2 + VU12 display — needs graceful shutdown signal before cutting power

// Delay between signaling the Odroid to shut down (see odroid_shutdown.cpp) and actually
// cutting RELAY_ODROID_PIN — margin for a normal Armbian graceful shutdown to complete.
// Measured on hardware: networking (an early-ish milestone in the shutdown sequence, not
// necessarily the final disk sync/unmount) goes down ~6s after the signal; 10s keeps some
// margin above that observed floor rather than cutting it exactly at the measured point.
#define ODROID_SHUTDOWN_DELAY_MS  10000UL

extern bool     odroidShutdownSignaled;    // true once the TCP signal is confirmed delivered this cycle
extern uint32_t odroidShutdownSignalTime;  // millis() when it was confirmed delivered
extern uint32_t odroidShutdownLastAttempt; // millis() of the last delivery attempt — retry-rate limit

// CAN transceiver standby — drive HIGH to put all three transceivers into standby during sleep.
// STBY pins lifted from GND on SK Pang board and wired to this pin.
#define CAN_STBY_PIN      32

// CAN2 RXD wake input — MCP2562 in standby still drives RXD low on dominant bus edges,
// allowing the EVCC to wake the VCU without KLR. CANRX2 / UART RX1 = pin 0 (confirmed).
#define CAN2_RX_PIN       0

// CAN3 RXD wake input — same standby behaviour as CAN2, allows the wireless gateway to
// wake the VCU without KLR. CRX3 = pin 30 (confirmed via FlexCAN_T4 source).
#define CAN3_RX_PIN       30

// TPS131PXQ1EVM-400 active pre-charge enable (TIDA-050082).
// Drive HIGH to enable; LOW to disable. Monitored by check_PreCharge() via IVT U2.
#define PRECHARGE_EN_PIN  34

extern SFE_UBLOX_GNSS myGNSS;
#endif

extern uint8_t pMBB32stale1;
extern uint8_t pMBB32stale2;
extern uint8_t pMBB32stale3;
extern uint8_t pMBB32staleMax;
extern uint32_t lastUpdatePMBB1;
extern uint32_t lastUpdatePMBB2;
extern uint32_t lastUpdatePMBB3;

extern uint8_t counter;

extern float ADCres;

extern uint8_t serialBlockTag[];
extern uint8_t frameID[];

extern uint8_t who;
extern uint32_t temp;
extern uint8_t minCell1;
extern uint8_t maxCell1;
extern uint8_t minCell2;
extern uint8_t maxCell2;
extern uint8_t minCell3;
extern uint8_t maxCell3;
extern uint8_t minModule1;
extern uint8_t maxModule1;
extern uint8_t minModule2;
extern uint8_t maxModule2;
extern uint8_t minModule3;
extern uint8_t maxModule3;
extern uint16_t minCellV1;
extern uint16_t maxCellV1;
extern uint16_t minCellV2;
extern uint16_t maxCellV2;
extern uint16_t minCellV3;
extern uint16_t maxCellV3;
extern uint16_t lowestCellV;
extern uint16_t lowestCell;
extern uint16_t lowestModule;
extern uint16_t highestCellV;
extern uint16_t highestCell;
extern uint16_t highestModule;
extern int32_t IVTpackCurrent;//1mA resolution, signed (negative = discharge)
extern int32_t IVTpackVoltage;//1mV resolution
extern int32_t IVTpreChargeV;//1mV resolution
extern int32_t IVTvoltage3;//1mV resolution
extern int32_t IVTtemp;//0.1°C resolution, signed
extern int32_t IVTpower;//1W resolution, signed (negative = regen)
extern int32_t IVTcoulombCounter;//1As resolution, signed
extern int32_t IVTenergyCounter;//1Wh resolution, signed
extern uint16_t SIM100MODohmsPerVolt;
extern uint16_t SIM100MODRpKohms;
extern uint16_t SIM100MODRnKohms;
extern uint16_t SIM100MODCpnF;
extern uint16_t SIM100MODCnnF;
extern uint16_t SIM100MODVp;
extern uint16_t SIM100MODVn;
extern uint16_t SIM100MODVb;
extern uint16_t SIM100MODVbMax;
extern uint8_t SIMM100MODerrorFlags;
extern uint32_t SIM100MODtemp;
extern uint16_t batteryVoltage;

// OpenInverter Tesla LDU V2 (CAN2 @ 500kbps)
//
// From firmware v5.32 the control frame has a FIXED bit-packed layout (not
// freely mappable). Configure the inverter terminal once:
//   potmode    2       (enable CAN throttle)
//   potmin     0
//   potmax     4095
//   controlid  513     (= 0x201 in decimal)
//   controlcheck 0     (disable CRC — implement later)
//   save
//
// Fixed control frame layout (8 bytes, little-endian packed):
//   data[0] bits  0-11 : pot          (12-bit throttle, 0-4095)
//   data[0] bits 12-23 : pot2         (12-bit regen channel, 0 = not used)
//   data[0] bits 24-29 : canio        (6-bit digital IO, see below)
//   data[0] bits 30-31 : ctr1         (2-bit sequence counter)
//   data[1] bits  0-13 : cruisespeed  (14-bit, 0 = not used)
//   data[1] bits 14-15 : ctr2         (must equal ctr1 every frame)
//   data[1] bits 16-23 : regenpreset  (8-bit regen %, 0 = not used)
//   data[1] bits 24-31 : crc          (8-bit, ignored when controlcheck=0)
//
// canio bits (6-bit field, bits 24-29 of data[0]):
//   bit 0 (0x01) = cruise
//   bit 1 (0x02) = start / enable
//   bit 2 (0x04) = brake
//   bit 3 (0x08) = forward
//   bit 4 (0x10) = reverse
//   bit 5 (0x20) = bms
//
// Safety: ctr1 must equal ctr2 AND differ from the previous frame's counter.
// After 5 consecutive invalid frames the inverter shuts down.
// t0 increments the counter each 10ms tick — well within the 500ms timeout.
//
// RX from LDU: TODO confirm actual IDs from inverter 'can tx' terminal output
#define LDU_CMD_ID          0x201

// canio 6-bit field definitions
#define LDU_CANIO_CRUISE    (1 << 0)
#define LDU_CANIO_START     (1 << 1)
#define LDU_CANIO_BRAKE     (1 << 2)
#define LDU_CANIO_FORWARD   (1 << 3)
#define LDU_CANIO_REVERSE   (1 << 4)
#define LDU_CANIO_BMS       (1 << 5)

// Internal direction state (set by keypad, translated to canio bits at transmit)
#define LDU_DIR_NEUTRAL     0
#define LDU_DIR_FORWARD     1
#define LDU_DIR_REVERSE     2
#define LDU_DIR_STOP        3

extern int32_t  LDUrpm;                    // motor speed (RPM)
extern int16_t  LDUtorque;                 // actual torque (Nm)
extern int16_t  LDUtorqueSetpoint;         // throttle demand 0–100 % — written by readThrottle(), read by t0
extern int16_t  LDUspeedLimit;             // max RPM — TODO: map via separate CAN rx if needed
extern int16_t  LDUregenLimit;             // regen limit — TODO: implement
extern uint8_t  LDUdirection;              // internal direction state, set by keypad
extern uint8_t  LDUseqCounter;             // 2-bit sequence counter; incremented each t0 tick
extern int16_t  LDUmotorTemp;              // motor temperature (°C × 10)
extern int16_t  LDUinverterTemp;           // inverter temperature (°C × 10)
extern uint16_t LDUdcVoltage;              // DC bus voltage (V × 10)
extern uint16_t LDUstatus;                 // status bitfield
extern uint16_t LDUfaults;                 // fault bitfield

// BMW i4/i5/i7/iX Changeover Valve 64119462114 — LIN slave on Serial6
// Serial6 hardware pins on Teensy 4.1: TX6=pin24, RX6=pin25
// TODO: confirm LIN node ID and full frame spec from BMW service docs
#define LIN_BAUD      19200     // LIN 2.x
#define LIN_VALVE_ID  0x10      // TODO: confirm BMW LIN node address
// #define LIN_EN_PIN ?         // TODO: assign LIN transceiver enable pin
extern uint8_t  valvePosition; // last commanded position (encoding TBD)
extern uint8_t  valveStatus;   // last reported status byte
extern bool     valveOnline;   // true once first valid response received

// EMP WP29-12V-CV-A Smart Flow Water Pump — EMP proprietary protocol (9980001068 Rev. N)
// Two pumps, same J1939 address (0x8A), on separate buses:
//   Inverter cooling loop: CAN2 @ 500kbps
//   Battery cooling loop:  CAN1 @ 500kbps
// Command (VCU → pump): TX ID = 0x18EF{pump_addr}{vcu_addr}
//   byte 0: 0xFD = Motor On (fwd) + DNC Power Hold; 0xFC = Motor Off + DNC
//   byte 3: %speed × 2 (0.5 %/bit); bytes 1-2, 4-7: 0xFF. Must send ≥ 1 Hz.
// Status (pump → VCU): Motor Status Message 1 @ 0x18FF03{pump_addr} (1 Hz)
//   byte 0: bits[1:0]=direction, bits[5:2]=controller_status, bits[7:6]=command_src
//   bytes 1-2: measured speed (little-endian uint16, 0.5 rpm/bit)
//   bytes 3-4: external temp (little-endian int16, 0.03125 °C/bit, offset −273)
//   bytes 5-6: motor power (little-endian uint16, 0.5 W/bit)
//   byte 7: bits[1:0]=service_indicator, bits[3:2]=operation_status
// Motor Status Message 3 @ 0x18FF24{pump_addr} (100 ms): voltage, current, HVIL
#define EMP_WP29_ADDR    0x8Au   // pump J1939 source address — both pumps share this address (separate buses)
#define VCU_CAN_ADDR     0xA3u   // VCU J1939 source address
#define EMP_WP29_CMD_ID  (0x18EF0000UL | ((uint32_t)EMP_WP29_ADDR << 8) | VCU_CAN_ADDR)
#define EMP_WP29_STATUS1 (0x18FF0300UL | EMP_WP29_ADDR)  // Motor Status Message 1 (1 Hz) — confirmed via DBC
#define EMP_WP29_STATUS3 (0x18FF2400UL | EMP_WP29_ADDR)  // Motor Status Message 3 (100 ms)
// Inverter cooling pump (CAN2)
extern uint16_t invPumpSpeed;    // raw measured speed (0.5 rpm/bit)
extern uint8_t  invPumpStatus;   // controller status nibble (byte 0 bits[5:2])
extern uint8_t  invPumpFaults;   // service indicator [1:0] + operation status [3:2] (byte 7)
extern uint8_t  invPumpSetpoint; // commanded speed 0–100 %
// Battery cooling pump (CAN1)
extern uint16_t battPumpSpeed;
extern uint8_t  battPumpStatus;
extern uint8_t  battPumpFaults;
extern uint8_t  battPumpSetpoint;

// Advantics ADM-CS-EVCC DC Fast Charge Controller (CAN2 @ 500kbps)
// Generic Power Modules protocol — 29-bit extended IDs (DBC raw values have bit 31 set).
// EVCC drives DCFC contactors autonomously via its own hardware outputs.
// DCFC inlet connects directly to battery pack — independent of main pack contactors (PDU-8).
//
// Received from EVCC (EVCC → VCU, extended IDs)
#define EVCC_NEW_SESSION    0x68001u  // New_Charge_Session: Protocol(8b), Plug_and_pins(8b), EV_Max_V(16b 0.1V), EV_Max_I(16b 0.1A), Capacity(8b 2kWh), SoC(8b %)
#define EVCC_INS_TEST       0x68002u  // Insulation_Test — informational
#define EVCC_PRECHARGE      0x68003u  // Precharge — informational
#define EVCC_STATUS_CHANGE  0x68004u  // Charge_Status_Change: Vehicle_Ready_for_Charging(8b)
#define EVCC_CHARGING_LOOP  0x68005u  // Charging_Loop: Target_V(16b 0.1V LE), Target_I(16b s 0.1A LE), SoC(8b %)
#define EVCC_EMERG_STOP     0x68006u  // Emergency_Stop: Origin(8b)
#define EVCC_SESSION_END    0x68007u  // Charge_Session_Finished: State(8b)
#define EVCC_CTRL_STATUS    0x68009u  // Advantics_Controller_Status: State(8b) — 200ms heartbeat
//
// Sent by VCU (VCU → EVCC, extended IDs, every 62.5ms)
#define EVCC_PWR_STATUS     0x60010u  // Power_Modules_Status: Present_V(16b 0.1V LE), Present_I(16b s 0.1A LE), Reserved(16b), System_Enable(8b), Insulation_R(8b 2kΩ/bit)
#define EVCC_PWR_LIMITS     0x60011u  // Power_Modules_Limits: Max_V(16b 0.1V LE), Max_I(16b s 0.1A LE), Reserved(32b)
#define EVCC_SEQ_CTRL       0x60012u  // Sequence_Control: Start_Auth(b0), CHAdeMO_Btn(b1) | CCS_Done(b0), CCS_Valid(b1), Params_Done(b2) | User_Stop(b0)
//
// Advantics v2.5 PEV protocol — standard 11-bit IDs (AC handshake, coexists with extended-ID protocol)
#define EVCC_EVSE_INFO      0x600u    // EVSE_Information: Stage(b0), Protocol(b1), Pins(b2), Max_I(b3:4 s A), RCD(b5.0) — received from EVCC
#define EVCC_AC_CTRL        0x601u    // AC_Control: Ready_To_Deliver_Power(b0) — received from EVCC
#define EVCC_AC_STATUS      0x611u    // AC_Status: Ready_To_Charge(b0) — sent by VCU every 62.5ms
//
// Plug_and_pins values (New_Charge_Session 0x68001 byte 1, old extended-ID protocol)
#define EVCC_PLUG_CCS_DC_CORE  0u  // CCS DC Core (DIN 70121 / ISO 15118 basic)
#define EVCC_PLUG_CCS_DC_EXT   1u  // CCS DC Extended
#define EVCC_PLUG_CHADEMO      2u  // CHAdeMO
// True for all defined DC plug types; any other value is treated as AC.
#define EVCC_PLUG_IS_DC(p) ((p) == EVCC_PLUG_CCS_DC_CORE || \
                            (p) == EVCC_PLUG_CCS_DC_EXT  || \
                            (p) == EVCC_PLUG_CHADEMO)
//
// Pins values (EVSE_Information 0x600 byte 1, v2.5 standard-ID protocol)
#define EVCC_PINS_CCS_AC       1u   // AC (generic)
#define EVCC_PINS_CCS_AC_1PH   2u   // AC single-phase core
#define EVCC_PINS_CCS_AC_3PH   3u   // AC three-phase core
#define EVCC_PINS_CCS_DC_CORE  4u
#define EVCC_PINS_CCS_DC_EXT   5u
#define EVCC_PINS_MCS          6u
#define EVCC_PINS_IS_AC(p)     ((p) >= 1u && (p) <= 3u)
//
// Battery charge limits — TODO: calibrate for actual pack configuration
#define EVCC_MAX_VOLTAGE_x10  4200u  // 420.0 V pack maximum (0.1V units)
#define EVCC_MAX_CURRENT_x10  1000u  // 100.0 A maximum charge current (0.1A units)
// pMBB32 cell voltage thresholds (16-bit ADC, 5 V ref → 1 count ≈ 76.3 µV)
#define EVCC_CELL_V_EMPTY  39322u  // ≈ 3.0 V — 0 % SoC reference (TODO: calibrate for cell chemistry)
#define EVCC_CELL_V_FULL   47841u  // ≈ 3.65 V — 100 % SoC (TODO: calibrate)
//
extern uint8_t  EVCCstage;          // Advantics_Controller_Status.State from 0x68009
extern uint8_t  EVCCplugType;       // Plug_and_pins from New_Charge_Session
extern bool     EVCCsessionActive;  // true from New_Charge_Session until Finished/Emergency
extern bool     EVCCemergencyStop;  // set on Emergency_Stop received
extern bool     EVCCsystemEnable;   // VCU authorisation flag → drives System_Enable in Power_Modules_Status
extern bool     normalEndOfCharge;  // set when highestCellV ≥ EVCC_CELL_V_FULL

extern uint8_t keypadStatus;// 0x01 = Park, 0x02 = Reverse, 0x03 = Neutral, 0x04 = Drive, 0x05 = Ignition, 0x06 = SpeedMode, 0x07 = AUX, 0x08 = DriveMode
extern uint16_t rpm;
extern uint16_t power;
extern uint16_t throttle;  // 0–100 %, written by readThrottle(), consumed by displayStatus()

// EVWest dual pot throttle (OEM pedal) — read via ADS1115 on I2C0
#define ADS_CH_THROTTLE1        0   // ADS1115 AIN0 — throttle pot 1
#define ADS_CH_THROTTLE2        1   // ADS1115 AIN1 — throttle pot 2
#define ADS_CH_BRAKE            2   // ADS1115 AIN2 — brake pedal
// Calibration endpoints in ADS1115 counts (GAIN_ONE ±4.096 V, int16_t 0–32767 for 0–4.096 V)
// Values below are ×8 approximations of the old 12-bit values — TODO: bench calibrate
#define THROTTLE_POT1_MIN       800
#define THROTTLE_POT1_MAX       31200
#define THROTTLE_POT2_MIN       800
#define THROTTLE_POT2_MAX       31200
#define THROTTLE_PLAUSIBILITY_PCT 5   // max allowable % difference between tracks
#define THROTTLE_FAULT_LIMIT    20    // max throttle % permitted when IVT or SIM fault active
#define BRAKE_THRESHOLD         1600  // ADS1115 counts — TODO: bench calibrate
#define PRECHARGE_TIMEOUT_MS    2000  // pre-charge relay must raise U2 to ≥95% of U1 within this window
extern uint16_t throttlePot1Raw;      // ADS1115 AIN0 count, track 1 (updated in loop())
extern uint16_t throttlePot2Raw;      // ADS1115 AIN1 count, track 2 (updated in loop())
extern int16_t  brakeRaw;             // ADS1115 AIN2 count (updated in loop())
extern bool     throttlePlausibility; // false = tracks disagree → throttle forced to 0
extern bool     brakePedal;           // true when brake pedal is pressed
extern bool     regenActive;          // true when LDU torque is negative and motor is spinning
extern bool     IVTfaultActive;       // set in can2Sniff on overcurrent / overvoltage
extern uint32_t preChargeStartTime;   // millis() when PreCharge state was entered
extern bool     chargeMode;           // true for AC charge session — routes pre-charge to Charge state (TODO: set from EVCC_NEW_SESSION when AC plug type detected)
extern bool     reducedPowerActive;   // true when BMS temp fault limits available power
extern uint16_t groundSpeed;
extern uint32_t GPSaltitude;
extern uint8_t fixType;
extern bool KL17state;
extern bool KL15state;  // true when keypad key 1 (Start/Stop) has been pressed to start

extern uint16_t debounceDelay;//debounce time (ms)
extern uint8_t buttonStartStop; // keypad key 1 — raw press state
extern uint8_t buttonPark;      // keypad key 2 — raw press state
extern uint8_t buttonReverse;   // keypad key 3 — raw press state
extern uint8_t buttonNeutral;   // keypad key 4 — raw press state
extern uint8_t buttonDrive;     // keypad key 5 — raw press state
extern uint8_t buttonSpeedMode; // keypad key 6 — raw press state
extern bool    stopRequested;   // set on a Start/Stop press while KL15state is already true;
                                 // consumed (and cleared) once LDUrpm==0 allows KL15_OFF
extern uint8_t new_0x01_state;
extern uint8_t new_0x02_state;
extern uint8_t new_0x04_state;
extern uint8_t new_0x08_state;
extern uint8_t new_0x10_state;
extern uint8_t new_0x20_state;
extern uint8_t new_0x40_state;
extern uint8_t new_0x80_state;
extern uint8_t last_0x01_state;
extern uint8_t last_0x02_state;
extern uint8_t last_0x04_state;
extern uint8_t last_0x08_state;
extern uint8_t last_0x10_state;
extern uint8_t last_0x20_state;
extern uint8_t last_0x40_state;
extern uint8_t last_0x80_state;
extern uint8_t  last_0x01_time;
extern uint8_t  last_0x02_time;
extern uint8_t  last_0x04_time;
extern uint8_t  last_0x08_time;
extern uint32_t last_0x10_time;  // uint32_t — holds millis() for debounce
extern uint8_t  last_0x20_time;
extern uint8_t  last_0x40_time;
extern uint8_t  last_0x80_time;

extern char ReplyBuffer[];
extern uint8_t displayBuffer[16];

extern unsigned int digitalPins;
extern int analogPins[7];

void displayStatus();

void wakepMBB32();
void shutdownpMBB32();

void ReadDigitalStatuses();
void ReadAnalogStatuses();
void linInit();
void linReadValve();
void linWriteValve(uint8_t position);
void readThrottle();

void check_KL15();
void check_PreCharge();
void check_Idle();
void check_DriveState();
void check_Charge();
void check_Fault();
void check_HeatPack();
void check_CoolPack();
void Off_enter();
void Off_exit();
void PreCharge_enter();
void PreCharge_exit();
void Idle_enter();
void Idle_exit();
void Drive_enter();
void Drive_exit();
void Charge_enter();
void Charge_exit();
void Fault_enter();
void Fault_exit();
void HeatPack_enter();
void HeatPack_exit();
void CoolPack_enter();
void CoolPack_exit();
void on_trans_Off_PreCharge();
void on_trans_PreCharge_Idle();
void on_trans_PreCharge_Charge();
void on_trans_PreCharge_Fault();
void on_trans_Fault_Off();
void KL15C_enter();
void check_KL15C();
void KL15C_exit();
void enterSleep();

extern byte buf[8];

extern unsigned int kpa; // 99.2
extern unsigned int tps; // 96.5
extern unsigned int clt;  // 80 - 100
extern unsigned int textCounter;
//uint16_t packV = 0;
//uint16_t packA = 0;
