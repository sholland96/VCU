/* defines.h
*
*/
CAN_message_t msg1;
CAN_message_t msg2;
CAN_message_t msg3;
CAN_message_t PDUmsg;

uint8_t counter;
uint32_t temp;
uint8_t ON = 0xFF;// duty cycle - 100/0.392157 = 0xFF = 100%
uint8_t OFF = 0;
uint32_t packCurrent;
uint32_t packV;
uint32_t preChargeV;
uint32_t U3V;
uint8_t keypadStatus;// 0x01 = Park, 0x02 = Reverse, 0x03 = Neutral, 0x04 = Drive, 0x05 = Ignition, 0x06 = SpeedMode, 0x07 = AUX, 0x08 = DriveMode
uint8_t minCell;
uint8_t maxCell;
uint8_t minModule;
uint8_t maxModule;
uint16_t minCellV;
uint16_t maxCellV;
float ADCres = 0.000076293945;
uint8_t pMBB32stale1;
uint8_t pMBB32stale2;
uint8_t pMBB32stale3;

uint8_t who;

void displayStatus();
void callback_cells_pdu();
void callback_cell_sample();
void callback5000ms();
void can1Sniff(const CAN_message_t);
void can2Sniff(const CAN_message_t);
void can3Sniff(const CAN_message_t);
void initCAN (int, int, int);
void wakepMBB32();
void shutdownpMBB32();
void displayStatus();