/* display.cpp — RealDash dashboard feed over CAN3 (IDs 0xc80-0xc83), also mirrored over the
 * VCU-to-Odroid Ethernet link (see realdash_tcp.cpp) for RealDash to consume directly. */
#include <Arduino.h>
#include "defines.h"
#include "display.h"
#include "realdash_tcp.h"

void displayStatus() {
  digitalWriteFast(6, HIGH);

  rpm += 100;
  if(rpm >= 13000)
    rpm = 0;

  IVTpackVoltage = 3840;

  power += 1;
  IVTpackCurrent = power * 1000000 / IVTpackVoltage;
  if(power > 45)
    power = 0;

  throttle = 100;
  batteryVoltage = 1255;

  // groundSpeed (mph) and GPSaltitude (ft) are updated live by printPVTdata() in init.cpp,
  // not here — already-converted units, safe to send as-is below.

  msg3.flags.extended = 1;
  msg3.id = 0xc80;
  msg3.len = 8;
  msg3.buf[0] = rpm;//RPM = V
  msg3.buf[1] = rpm>>8;
  msg3.buf[2] = power/10;//power*100*10*134/10000;//kW = V/10
  msg3.buf[3] = (power/10)>>8;//(power*100*10*134/10000)>>8;
  msg3.buf[4] = 21;//°C = V-100
  msg3.buf[5] = 0;
  msg3.buf[6] = throttle*10;//TPS = V/10
  msg3.buf[7] = (throttle*10)>>8;
  can3.write(msg3);
  realdashQueueFrame(0xc80, msg3.buf, 8);
  //delay(1);

  msg3.id = 0xc81;
  msg3.len = 8;
  msg3.buf[0] = 0;
  msg3.buf[1] = 0;
  msg3.buf[2] = IVTpackVoltage;
  msg3.buf[3] = IVTpackVoltage>>8;
  msg3.buf[4] = IVTpackCurrent;
  msg3.buf[5] = IVTpackCurrent>>8;
  msg3.buf[6] = batteryVoltage;
  msg3.buf[7] = batteryVoltage>>8;
  can3.write(msg3);
  realdashQueueFrame(0xc81, msg3.buf, 8);
  //delay(1);

  msg3.id = 0xc82;
  msg3.len = 8;
  msg3.buf[0] = highestCellV;
  msg3.buf[1] = highestCellV>>8;
  msg3.buf[2] = lowestCellV;
  msg3.buf[3] = lowestCellV>>8;
  msg3.buf[4] = groundSpeed;
  msg3.buf[5] = groundSpeed>>8; // was hardcoded 0 — truncated speed to a single byte (overflows instantly at real mph)
  msg3.buf[6] = GPSaltitude;
  msg3.buf[7] = GPSaltitude>>8;
  can3.write(msg3);
  realdashQueueFrame(0xc82, msg3.buf, 8);
  //delay(1);

  msg3.id = 0xc83;
  msg3.len = 8;
  msg3.buf[0] = highestCellV - lowestCellV;
  msg3.buf[1] = (highestCellV - lowestCellV)>>8;
  msg3.buf[2] = SIM100MODohmsPerVolt;
  msg3.buf[3] = SIM100MODohmsPerVolt>>8;
  msg3.buf[4] = SIM100MODtemp;
  msg3.buf[5] = SIM100MODtemp>>8;
  msg3.buf[6] = fixType;
  msg3.buf[7] = 0;
  can3.write(msg3);
  realdashQueueFrame(0xc83, msg3.buf, 8);

  digitalWriteFast(6, LOW);
}//end of displayStatus()
