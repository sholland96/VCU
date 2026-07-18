#pragma once
/* vcu.h
 * Extern declarations for objects and constants defined in main.cpp.
 * Include this in any module that needs access to CAN buses, timers, FSM, or
 * the main.cpp file-scope variables. Also includes defines.h so callers only
 * need one include.
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <TeensyTimerTool.h>
#include "Fsm.h"
#include "pMBB32.h"
#include "defines.h"

// ── Hardware pins (defined in main.cpp) ──────────────────────────────────────
#define LED_PIN    13
#define KL15R_PIN  2

// ── FSM event tokens (unique integers required by arduino-fsm) ───────────────
#define KL15_ON          1
#define KL15_OFF         2
#define DRIVE_ON         3
#define DRIVE_OFF        4
#define CHARGE_ON        5
#define CHARGE_OFF       6
#define PRECHARGE_OK     7
#define PRECHARGE_CHARGE 8
#define PRECHARGE_FAIL   9
#define FAULT_EV         10
#define FAULT_CLEAR      11
#define TEMP_LOW         12
#define TEMP_HIGH        13
#define TEMP_OK          14
#define EXT_WAKE         15
#define AC_CHARGE_START  16

// ── Sleep magic ───────────────────────────────────────────────────────────────
#define SLEEP_MAGIC_CAN_WAKE 0xC4A8B3E1UL
#define KL30C_SLEEP_TIMEOUT_MS 60000UL

// ── CAN buses (defined in main.cpp) ──────────────────────────────────────────
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
extern FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;

// ── CAN message buffers (defined in main.cpp) ─────────────────────────────────
extern CAN_message_t msg1;
extern CAN_message_t msg2;
extern CAN_message_t msg3;
extern CAN_message_t PDUmsg1;
extern CAN_message_t PDUmsg2;
extern CAN_message_t LDUmsg;

// ── Timers (defined in main.cpp) ──────────────────────────────────────────────
extern IntervalTimer t0;
extern TeensyTimerTool::PeriodicTimer t1;
extern TeensyTimerTool::PeriodicTimer t2;
extern TeensyTimerTool::PeriodicTimer t3;

// ── FSM states and machine (defined in main.cpp) ─────────────────────────────
extern State state_Off;
extern State state_PreCharge;
extern State state_Idle;
extern State state_Drive;
extern State state_Charge;
extern State state_HeatPack;
extern State state_CoolPack;
extern State state_Fault;
extern State state_KL30C;
extern Fsm   fsm;

// ── Main-scope globals (defined in main.cpp) ──────────────────────────────────
extern bool          extWakePending;
extern uint32_t      lastExtActivityMs;
extern bool          kl30cKL15Rstate;
extern bool          evccIsACSession;
extern bool          acReadyToDeliver;
extern VCUStateEnum  VCUstate;
extern volatile uint32_t sleepMagic;
