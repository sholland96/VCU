/* defines.h
*
*/

typedef enum {
  VCU_STATE_OFF = 0,
  VCU_STATE_PRECHARGE,
  VCU_STATE_IDLE,
  VCU_STATE_DRIVE,
  VCU_STATE_CHARGE,
  VCU_STATE_HEAT_PACK,
  VCU_STATE_COOL_PACK,
  VCU_STATE_FAULT
} VCUStateEnum;

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
Adafruit_ADS1115 ads;
#define ADS1115_ADDR  0x48

#ifdef UBLOX_GNSS
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// SK Pang board: NEO-M8M GNSS on I2C0 — Wire (SDA=pin 18, SCL=pin 19)
// 1PPS output from NEO-M8M is routed to Teensy pin 35 on the SK Pang board.
// EXTINT header on SK Pang board wired to this Teensy output — rising edge
// wakes the module from backup mode before AIRCR reset.
#define GPS_PPS_PIN       35
#define GNSS_EXTINT_PIN   33  // free GPIO — wire to NEO-M8M EXTINT header on SK Pang board

// CAN transceiver standby — drive HIGH to put all three transceivers into standby during sleep.
// STBY pins lifted from GND on SK Pang board and wired to this pin.
#define CAN_STBY_PIN      32

// TPS131PXQ1EVM-400 active pre-charge enable (TIDA-050082).
// Drive HIGH to enable; LOW to disable. Monitored by check_PreCharge() via IVT U2.
#define PRECHARGE_EN_PIN  34

// Waveshare SIM7080G Cat-M/NB-IoT HAT — cellular modem (LTE-M, T-Mobile)
// UART: Serial4 (TX=17, RX=16) at 115200 baud, 3.3V logic (HAT default).
// PWRKEY: pulse LOW for ~1 s on startup to power on the module.
// SMS via AT+CMGS; data via AT+CNACT / AT+SHCONN (HTTPS).
// Do not connect HAT GNSS antenna — u-blox NEO-M8M already fitted.
#define SIM7080_SERIAL    Serial4
#define SIM7080_BAUD      115200
#define SIM7080_PWRKEY    36    // pulse LOW ~1 s to power on module

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
uint32_t IVTpackCurrent;//1mA resolution
uint32_t IVTpackVoltage;//1mV resolution
uint32_t IVTpreChargeV;//1mV resolution
uint32_t IVTvoltage3;//1mV resolution
uint32_t IVTtemp;
uint32_t IVTpower;
uint32_t IVTcoulombCounter;
uint32_t IVTenergyCounter;
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

// BMW i4/i5/i7/iX Changeover Valve 64119462114 — LIN slave on Serial3
// Serial3 hardware pins on Teensy 4.1: TX3=pin14 (A0), RX3=pin15 (A1)
// SK Pang board GNSS UART is on Serial2 (pins 7/8) — Serial3 is free for LIN.
// TODO: confirm LIN node ID and full frame spec from BMW service docs
#define LIN_BAUD      19200     // LIN 2.x
#define LIN_VALVE_ID  0x10      // TODO: confirm BMW LIN node address
// #define LIN_EN_PIN ?         // TODO: assign LIN transceiver enable pin
uint8_t  valvePosition = 0;     // last commanded position (encoding TBD)
uint8_t  valveStatus   = 0;     // last reported status byte
bool     valveOnline   = false; // true once first valid response received

// EMP WP29-12V-CV-A Smart Flow Water Pump (CAN2 @ 500kbps)
// Command (VCU → pump): EMP proprietary Motor Command Message
//   TX ID = 0x18EF{pump_addr}{vcu_addr} — byte 0: On/Off+PowerHold; bytes 1-2: 0xFFFF;
//   byte 3: %speed × 2 (0.5 %/bit); bytes 4-7: 0xFF. Must send ≥ 1 Hz.
//   On/Off byte: 0xFD = Motor On (forward) + DNC Power Hold; 0xFC = Motor Off + DNC
// Status (pump → VCU): Motor Status Message 2 @ 0x18FF23{pump_addr} (1 Hz)
//   byte 0: bits[1:0]=direction, bits[5:2]=controller_status, bits[7:6]=command_src
//   bytes 1-2: measured speed (little-endian uint16, 0.5 rpm/bit)
//   bytes 3-4: external temp (little-endian int16, 0.03125 °C/bit, offset −273)
//   bytes 5-6: motor power (little-endian uint16, 0.5 W/bit)
//   byte 7: bits[1:0]=service_indicator, bits[3:2]=operation_status
// Motor Status Message 3 @ 0x18FF24{pump_addr} (100 ms): voltage, current, HVIL
// Source addresses: confirm pump address via CAN sniffer or DBC file.
#define EMP_WP29_ADDR    0x8Au   // pump J1939 source address — verify via DBC/sniffer
#define VCU_CAN_ADDR     0xA3u   // VCU J1939 source address
#define EMP_WP29_CMD_ID  (0x18EF0000UL | ((uint32_t)EMP_WP29_ADDR << 8) | VCU_CAN_ADDR)
#define EMP_WP29_STATUS1 (0x18FF0300UL | EMP_WP29_ADDR)  // Motor Status Message 1 (1 Hz) — confirmed via DBC
#define EMP_WP29_STATUS3 (0x18FF2400UL | EMP_WP29_ADDR)  // Motor Status Message 3 (100 ms)
uint16_t pumpActualSpeed;  // raw measured speed (0.5 rpm/bit — divide by 2 for RPM)
uint8_t  pumpStatus;       // controller status nibble (byte 0 bits[5:2] of Status Msg 2)
uint8_t  pumpFaults;       // service indicator [1:0] + operation status [3:2] (byte 7)
uint8_t  pumpSetpoint;     // commanded speed 0–100 %
uint8_t keypadStatus;// 0x01 = Park, 0x02 = Reverse, 0x03 = Neutral, 0x04 = Drive, 0x05 = Ignition, 0x06 = SpeedMode, 0x07 = AUX, 0x08 = DriveMode
uint16_t rpm = 0;
uint16_t power = 0;
uint16_t throttle = 0;  // 0–100 %, written by readThrottle(), consumed by displayStatus()

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
uint16_t throttlePot1Raw;             // ADS1115 AIN0 count, track 1 (updated in loop())
uint16_t throttlePot2Raw;             // ADS1115 AIN1 count, track 2 (updated in loop())
int16_t  brakeRaw = 0;                // ADS1115 AIN2 count (updated in loop())
bool     throttlePlausibility = true; // false = tracks disagree → throttle forced to 0
bool     brakePedal           = false;// true when brake pedal is pressed
bool     regenActive          = false;// true when LDU torque is negative and motor is spinning
bool     IVTfaultActive       = false;// set in can2Sniff on overcurrent / overvoltage
uint32_t preChargeStartTime   = 0;   // millis() when PreCharge state was entered
bool     chargeMode           = false;// true when pre-charge was triggered by EVCC, not KL15
bool     reducedPowerActive   = false;// true when BMS temp fault limits available power
uint16_t groundSpeed = 0;
uint32_t GPSaltitude = 0;
uint8_t fixType;
bool KL17state;
bool KL15state = false;  // true when keypad button 5 (KL15/Ignition) is active

uint16_t debounceDelay = 250;//debounce time (ms)
uint8_t button_0x01_state;
uint8_t button_0x02_state;
uint8_t button_0x04_state;
uint8_t button_0x08_state;
uint8_t button_0x10_state;
uint8_t button_0x20_state;
uint8_t button_0x40_state;
uint8_t button_0x80_state;
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

void displayStatus();

void can1Sniff(const CAN_message_t);
void can2Sniff(const CAN_message_t);
void can3Sniff(const CAN_message_t);

void initCAN (int, int, int);
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

byte buf[8];

unsigned int kpa = 992; // 99.2
unsigned int tps = 965; // 96.5
unsigned int clt = 80;  // 80 - 100
unsigned int textCounter = 0;
//uint16_t packV = 0;
//uint16_t packA = 0;

