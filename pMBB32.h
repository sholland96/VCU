/* pMBB32.h
*
*/
//#define ADCres 0.000076293945;

// 0x009Fxx00 report status command, response frames 0x18FF0Fyy (low IC) and 0x18FF10yy (high IC) byte definitions - status registers for each IC
#define deviceStatus = 0;
#define systemFault = 1;
#define faultSummaryLow = 2;
#define faultSummaryHigh = 3;
#define chipFaultLow = 4;
#define chipFaultHigh = 5;

// 0x009Fxx00 report status command, response frames 0x18FF11yy (low IC) and 0x18FF12yy (high IC) byte definitions - status registers for each IC
#define cellOVlowByte = 0;
#define cellOVhighByte = 1;
#define cellUVlowByte = 2;
#define cellUVhighByte = 3;
#define comparatorOVlowByte = 4;
#define comparatorOVhighByte = 5;
#define comparatorUVlowByte = 6;
#define comparatorUVhighByte = 7;

// 0x00AFxx00 set mode command, no response frame - command byte 0
#define wakeup 0x01
#define sendCommClear 0x02
#define sendCommReset 0x03
#define sendOVUVthresholds 0x04
// bytes 1 and 2, cell OV threshold: 4.2V/5/2^16 = 55050 = 0xD70A (default is 4.199V = 0xD709)
// bytes 3 and 4, cell UV threshold: 2.5V/5/2^16 = 32768 = 0x8000 (default)
// byte 5, comparator OV threshold: default is 4.3V = 0xB8
// byte 6, comparator UV threshold: default is 2.4V = 0x88, 2.8V = 0xA8
#define sendCANid 0x0F
#define contReportingEnable 0x10
#define contReportingDisable 0x11
#define shutdown 0x55

// 0x00CFxx00 min/max cells command, response frame 0x18FF0Eyy byte definitions
#define minCellNum 0
#define maxCellNum 1
#define maxCellLowByte 2
#define maxCellHighByte 3
#define minCellLowByte 4
#define minCellHighByte 5
#define minmaxDeltaVLowByte 7
#define minmaxDeltaVHighByte 6

// 0x00FF0000 start of measurement command and cell voltage reports (broadcast)
// response frames 0x18FF01yy (low IC) and 0x18FF07yy (high IC) byte definitions
#define cell1lowByte 0
#define cell1highByte 1
#define cell2lowByte 2
#define cell2highByte 3
#define cell3lowByte 4
#define cell3highByte 5
#define cell4lowByte 6
#define cell4highByte 7
// response frames 0x18FF02yy (low IC) 0x18FF08yy (high IC) byte definitions
#define cell5lowByte 0
#define cell5highByte 1
#define cell6lowByte 2
#define cell6highByte 3
#define cell7lowByte 4
#define cell7highByte 5
#define cell8lowByte 6
#define cell8highByte 7
// response frames 0x18FF03yy (low IC) 0x18FF09yy (high IC) byte definitions
#define cell9lowByte 0
#define cell9highByte 1
#define cell10lowByte 2
#define cell10highByte 3
#define cell11lowByte 4
#define cell11highByte 5
#define cell12lowByte 6
#define cell12highByte 7
// response frames 0x18FF04yy (low IC) 0x18FF0Ayy (high IC) byte definitions
#define cell13lowByte 0
#define cell13highByte 1
#define cell14lowByte 2
#define cell14highByte 3
#define cell15lowByte 4
#define cell15highByte 5
#define cell16lowByte 6
#define cell16highByte 7
// response frames 0x18FF05yy (low IC) 0x18FF0Byy (high IC) byte definitions
#define aux1lowByte 0
#define aux1highByte 1
#define aux2lowByte 2
#define aux2highByte 3
#define aux3lowByte 4
#define aux3highByte 5
#define aux4lowByte 6
#define aux4highByte 7
// response frames 0x18FF06yy (low IC) 0x18FF0Cyy (high IC) byte definitions
#define PCBlowByte 0
#define PCBhighByte 1
#define analogDielowByte 2
#define analogDiehighByte 3

//Internal Digital Die Temperature °C = (VADC – 2.287) × 131.944
//Internal Analog Die Temperature °C = (VADC – 1.8078) × 147.514
//VMODULE = ([(2 × VREF) / 65535] × READ_ADC_VALUE) × 25
//VAUX = [(2 × VREF) / 65535] × READ_ADC_VALUE