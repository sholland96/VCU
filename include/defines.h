/* defines.h
*
*/







EthernetServer server(80);
EthernetUDP udpServer;

// Set the static IP to something other than INADDR_NONE (all zeros)
// to not use DHCP. The values here are just examples.
IPAddress localIP(192, 168, 1, 101);
IPAddress targetIP(192, 168, 1, 102);
IPAddress subnetMask{255, 255, 0, 0};
IPAddress gateway{192, 168, 1, 1};
int targetPort = 35000; // Port number on the target device

uint8_t pMBB32stale1;
uint8_t pMBB32stale2;
uint8_t pMBB32stale3;

uint8_t counter = 0;

extern byte mac[];

float ADCres = 0.000076293945;

uint8_t serialBlockTag[] = {0x44,0x33,0x22,0x11};
uint8_t frameID[] = {0x0c,0x08};

uint8_t who;
uint32_t temp;
uint8_t minCell;
uint8_t maxCell;
uint8_t minModule;
uint8_t maxModule;
uint16_t minCellV;
uint16_t maxCellV;
uint32_t packCurrent;
uint32_t packVoltage;
uint32_t preChargeV;
uint16_t batteryVoltage;
uint8_t keypadStatus;// 0x01 = Park, 0x02 = Reverse, 0x03 = Neutral, 0x04 = Drive, 0x05 = Ignition, 0x06 = SpeedMode, 0x07 = AUX, 0x08 = DriveMode
uint32_t U3V;

uint16_t rpm = 0;
uint16_t power = 0;
uint16_t throttle = 0;
uint16_t groundSpeed = 0;
uint16_t GPSaltitude = 0;

char ReplyBuffer[] = "acknowledged";
uint8_t displayBuffer[16];

unsigned int digitalPins = 0;
int analogPins[7] = {0};

void displayStatus();
void callback_cells_pdu();
void callback_cell_sample();
void callback500ms();

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

void displayStatus();

byte buf[8];

unsigned int kpa = 992; // 99.2
unsigned int tps = 965; // 96.5
unsigned int clt = 80;  // 80 - 100
unsigned int textCounter = 0;
//uint16_t packV = 0;
//uint16_t packA = 0;

