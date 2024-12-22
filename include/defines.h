/* defines.h
*
*/

#define pMBB32powerOn 0xFF;
#define pMBB32powerOff 0;

#define ON 0xFF;// duty cycle - 100/0.392157 = 0xFF = 100%
#define OFF 0;

// Set the static IP to something other than INADDR_NONE (all zeros)
// to not use DHCP. The values here are just examples.
IPAddress localIP(192, 168, 1, 101);
IPAddress targetIP(192, 168, 1, 102);
IPAddress subnetMask{255, 255, 0, 0};
IPAddress gateway{192, 168, 1, 1};
uint16_t targetPort = 35000; // Port number on the target device

EthernetServer server(targetPort);
EthernetClient connectedClient;
//EthernetUDP udpServer;

uint8_t pMBB32stale1;
uint8_t pMBB32stale2;
uint8_t pMBB32stale3;
uint8_t pMBB32staleMax;

uint8_t counter = 0;

extern byte mac[];

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
uint32_t IVTpackCurrent;
uint32_t IVTpackVoltage;
uint32_t IVTpreChargeV;
uint32_t IVTvoltage3;
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
uint16_t batteryVoltage;
uint8_t keypadStatus;// 0x01 = Park, 0x02 = Reverse, 0x03 = Neutral, 0x04 = Drive, 0x05 = Ignition, 0x06 = SpeedMode, 0x07 = AUX, 0x08 = DriveMode
uint32_t callback_cell_sample_start;
uint32_t callback_cell_sample_finish = 0;
uint32_t callback_main_loop_start;
uint32_t callback_main_loop_finish = 0;
uint16_t rpm = 0;
uint16_t power = 0;
uint16_t throttle = 0;
uint16_t groundSpeed = 0;
uint16_t GPSaltitude = 0;

char ReplyBuffer[] = "acknowledged";
uint8_t displayBuffer[16];

unsigned int digitalPins = 0;
int analogPins[7] = {0};

typedef struct {
  long id;
  byte rtr;
  byte ide;
  byte dlc;
  byte dataArray[8]; // Adjust the size to match your CAN messages
} packet_t;

void displayStatus();
void callback_cells_pdu();
void callback_cell_sample();
void callback1000ms();

void can1Sniff(const CAN_message_t);
void can2Sniff(const CAN_message_t);
void can3Sniff(const CAN_message_t);

void teensyMAC(uint8_t *mac);
void startEthernet();
byte mac[] = {0x04, 0xE9, 0xE5, 0x17, 0xD2, 0x9E};

void initCAN (int, int, int);
void wakepMBB32();
void shutdownpMBB32();

void ReadDigitalStatuses();
void ReadAnalogStatuses(); 

void SendCANFrameToClient(unsigned long canFrameId);
//void SendTextExtensionFrameToEth(unsigned long canFrameId, const char* text);
void sendTestData();

void SendCANFramesToEth(EthernetClient& client);
void forwardAsRD44Frame(packet_t *packet, EthernetClient client);
void initCANframes(CAN_message_t *, CAN_message_t *);

void displayStatus();

byte buf[8];

unsigned int kpa = 992; // 99.2
unsigned int tps = 965; // 96.5
unsigned int clt = 80;  // 80 - 100
unsigned int textCounter = 0;
//uint16_t packV = 0;
//uint16_t packA = 0;

