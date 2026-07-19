#pragma once

void can1Sniff(const CAN_message_t &msg);
void can2Sniff(const CAN_message_t &msg);
void can3Sniff(const CAN_message_t &msg);
void initCAN(int CAN1baud, int CAN2baud, int CAN3baud);
