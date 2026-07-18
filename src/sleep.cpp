/* sleep.cpp — VCU sleep/wake and pMBB32 power sequencing. */
#include "vcu.h"
#include "sleep.h"
#include "sdlog.h"

extern "C" uint32_t set_arm_clock(uint32_t frequency); // clockspeed.c

void enterSleep() {
  if (EVCCsessionActive) return;

  Serial.println("KLR off — sleeping");
  sdLogEvent("SLEEP");
  sdFlushAll();
  Serial.flush();

  USBPHY1_PWD  = 0xFFFFFFFF;
  CCM_CCGR6   &= ~CCM_CCGR6_USBOH3(3);

  t0.end();
  t1.stop();
  t2.stop();
  t3.stop();
  digitalWriteFast(LED_BUILTIN, LOW);

#ifdef UBLOX_GNSS
  myGNSS.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0);
  delay(500);
#endif

  digitalWrite(CAN_STBY_PIN, HIGH);

  CCM_CCGR0 &= ~(CCM_CCGR0_CAN1(3) | CCM_CCGR0_CAN1_SERIAL(3) |
                 CCM_CCGR0_CAN2(3) | CCM_CCGR0_CAN2_SERIAL(3));
  CCM_CCGR7 &= ~(CCM_CCGR7_CAN3(3) | CCM_CCGR7_CAN3_SERIAL(3));

  pinMode(CAN2_RX_PIN, INPUT_PULLUP);

  set_arm_clock(16000000);

  CCM_CBCMR = (CCM_CBCMR & ~CCM_CBCMR_PERIPH_CLK2_SEL_MASK)
            | CCM_CBCMR_PERIPH_CLK2_SEL(1);
  while (CCM_CDHIPR & CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY);
  CCM_CBCDR &= ~CCM_CBCDR_PERIPH_CLK2_PODF_MASK;
  CCM_CBCDR |=  CCM_CBCDR_PERIPH_CLK_SEL;
  while (CCM_CDHIPR & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY);
  CCM_ANALOG_PLL_ARM |= CCM_ANALOG_PLL_ARM_BYPASS;
  CCM_ANALOG_PLL_ARM |= CCM_ANALOG_PLL_ARM_POWERDOWN;

  attachInterrupt(digitalPinToInterrupt(KL15R_PIN),  [](){}, RISING);
  attachInterrupt(digitalPinToInterrupt(CAN2_RX_PIN), [](){}, FALLING);
  SYST_CSR &= ~1u;

  CCM_CLPCR = (CCM_CLPCR & ~0x3u) | 0x2u;
  SCB_SCR   |= (1u << 2);

  asm volatile("dsb");
  asm volatile("isb");
  if (!digitalRead(KL15R_PIN)) {
    asm volatile("wfi");
  }
  asm volatile("dsb");

  sleepMagic = digitalRead(KL15R_PIN) ? 0 : SLEEP_MAGIC_CAN_WAKE;

#ifdef UBLOX_GNSS
  digitalWrite(GNSS_EXTINT_PIN, HIGH);
  { uint32_t t = ARM_DWT_CYCCNT; while (ARM_DWT_CYCCNT - t < 150000u); }
#endif

  SCB_AIRCR = 0x05FA0004;
  while (1);
}

void wakepMBB32() {
  msg1.flags.extended = 1;
  static const uint32_t modIds[] = {0xAF0100, 0xAF0200, 0xAF0300};
  for (uint8_t i = 0; i < 3; i++) {
    msg1.id     = modIds[i];
    msg1.len    = 3;
    msg1.buf[0] = wakeup;
    msg1.buf[1] = channelCount16;
    msg1.buf[2] = numberOfDevices;
    can1.write(msg1);
    delay(10);
  }
  msg1.id  = 0xFF0000;
  msg1.len = 0;
  can1.write(msg1);
}

void shutdownpMBB32() {
  msg1.id     = 0xAF0300;
  msg1.len    = 1;
  msg1.buf[0] = 0x55;
  can1.write(msg1); delay(1); can1.write(msg1); delay(1);

  msg1.id = 0xAF0200;
  can1.write(msg1); delay(1); can1.write(msg1); delay(1);

  msg1.id = 0xAF0100;
  can1.write(msg1); delay(5); can1.write(msg1); delay(5);
}
