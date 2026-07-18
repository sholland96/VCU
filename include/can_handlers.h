#pragma once
/* can_handlers.h — CAN receive callbacks and bus initialisation. */
#include <FlexCAN_T4.h>

// Ghost SA / frame-type coverage flags — written by can1Sniff ISR,
// read and reset by callback_t2 (also ISR context via GPT1).
extern volatile bool     pMBB32ghostSA;
extern volatile uint16_t pMBB32ftSeen[3];

void can1Sniff(const CAN_message_t &msg);
void can2Sniff(const CAN_message_t &msg);
void can3Sniff(const CAN_message_t &msg);
void initCAN(int CAN1baud, int CAN2baud, int CAN3baud);
