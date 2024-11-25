#include <Arduino.h>
#include <FlexCAN_T4.h>



void SendCANFramesToEth(EthernetClient& client);
void initCANframes(CAN_message_t *, CAN_message_t *);



