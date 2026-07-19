/* sleep.cpp — deep-sleep (STOP mode) entry: gates clocks/peripherals,
 * arms KL15R/CAN2 wake sources, then WFI + AIRCR reset on wake.
 */
#include <Arduino.h>
#include "defines.h"
#include "sdlog.h"
#include "sleep.h"

extern "C" uint32_t set_arm_clock(uint32_t frequency); // clockspeed.c

void enterSleep() {
  if (EVCCsessionActive) return;  // active EVCC charge session — stay awake

  Serial.println("KLR off — sleeping");
  sdLogEvent("SLEEP");
  sdFlushAll();
  Serial.flush();

  // 1. Power down USB PHY and gate its clock — restores automatically on AIRCR reset.
  USBPHY1_PWD    = 0xFFFFFFFF;
  CCM_CCGR6     &= ~CCM_CCGR6_USBOH3(3);

  t0.end();   // IntervalTimer uses end()
  t1.stop();  // TeensyTimerTool PeriodicTimer
  t2.stop();
  t3.stop();
  digitalWriteFast(LED_BUILTIN, LOW);

#ifdef UBLOX_GNSS
  // I2C cannot wake NEO-M8M from backup mode; EXTINT0 is the supported source.
  myGNSS.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0);
  delay(500);
#endif

  // Assert CAN transceiver standby — puts all three transceivers into low-power mode.
  digitalWrite(CAN_STBY_PIN, HIGH);

  // Gate FlexCAN peripheral clocks — stops internal CAN controller sampling.
  // AIRCR reset on wake restores all CCM clock gates to boot defaults.
  CCM_CCGR0 &= ~(CCM_CCGR0_CAN1(3) | CCM_CCGR0_CAN1_SERIAL(3) |
                 CCM_CCGR0_CAN2(3) | CCM_CCGR0_CAN2_SERIAL(3));
  CCM_CCGR7 &= ~(CCM_CCGR7_CAN3(3) | CCM_CCGR7_CAN3_SERIAL(3));

  // Reconfigure CAN2 RXD pin as GPIO input (CAN clock already gated).
  // MCP2562 in standby drives RXD low on dominant bus edges — triggers the interrupt below.
  pinMode(CAN2_RX_PIN, INPUT_PULLUP);

  // 2. Drop CPU to ARM PLL minimum (~16.2 MHz, 0.95 V DCDC).
  set_arm_clock(16000000);

  // Switch AHB to 24 MHz crystal, then bypass ARM PLL so the CPU also takes the
  // crystal reference (24 MHz / ARM_PODF ≈ 3 MHz), then power down the PLL VCO.
  // Mirrors the temporary switch inside set_arm_clock() — proven safe sequence.
  // AIRCR reset on wake restores all clock registers to boot state.
  CCM_CBCMR = (CCM_CBCMR & ~CCM_CBCMR_PERIPH_CLK2_SEL_MASK)
            | CCM_CBCMR_PERIPH_CLK2_SEL(1);           // PERIPH_CLK2 = 24 MHz OSC
  while (CCM_CDHIPR & CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY);
  CCM_CBCDR &= ~CCM_CBCDR_PERIPH_CLK2_PODF_MASK;      // ÷1 (no divide on crystal)
  CCM_CBCDR |=  CCM_CBCDR_PERIPH_CLK_SEL;             // AHB from crystal
  while (CCM_CDHIPR & CCM_CDHIPR_PERIPH_CLK_SEL_BUSY);
  CCM_ANALOG_PLL_ARM |= CCM_ANALOG_PLL_ARM_BYPASS;    // CPU: crystal ref / ARM_PODF
  CCM_ANALOG_PLL_ARM |= CCM_ANALOG_PLL_ARM_POWERDOWN; // ARM PLL VCO off

  // 3. GPIO interrupt on KL15R rising edge; disable SysTick 1 ms tick.
  //    Single WFI then sleeps until the actual wake event instead of every 1 ms.
  //    If KL15R is already high when WFI executes, the pending interrupt returns it
  //    immediately — no race condition.
  attachInterrupt(digitalPinToInterrupt(KL15R_PIN),  [](){}, RISING);   // key-on
  attachInterrupt(digitalPinToInterrupt(CAN2_RX_PIN), [](){}, FALLING); // EVCC bus activity
  SYST_CSR &= ~1u;  // disable SysTick (bit 0 = ENABLE) — stops 1 ms wakeups

  // STOP mode — gates more internal domains than WAIT mode.
  // Wake sources: KL15R_PIN rising edge (key-on) or CAN2_RX_PIN falling edge (EVCC).
  // AIRCR reset on wake restores all registers, so no clock restore needed.
  // To revert to WAIT mode: delete the two lines below.
  CCM_CLPCR = (CCM_CLPCR & ~0x3u) | 0x2u;  // LPM = 0b10 (STOP)
  SCB_SCR   |= (1u << 2);                    // SLEEPDEEP — WFI enters STOP not WAIT

  asm volatile("dsb");
  asm volatile("isb");
  if (!digitalRead(KL15R_PIN)) {
    asm volatile("wfi");  // sleep until KL15R rising edge or CAN2 falling edge
  }

  asm volatile("dsb");

  // Set sleepMagic based on which signal woke us.
  // KL15R HIGH = key was turned → normal boot; KL15R still LOW = CAN2 woke us → KL30C.
  sleepMagic = digitalRead(KL15R_PIN) ? 0 : SLEEP_MAGIC_CAN_WAKE;

#ifdef UBLOX_GNSS
  // Rising edge on EXTINT0 wakes the GNSS module for a hot-start while the Teensy resets.
  // Use DWT cycle counter — independent of SysTick and the stale F_CPU_ACTUAL.
  // At ~3 MHz (crystal / ARM_PODF=8), 150 000 cycles ≈ 50 ms.
  digitalWrite(GNSS_EXTINT_PIN, HIGH);
  {
    uint32_t t = ARM_DWT_CYCCNT;
    while (ARM_DWT_CYCCNT - t < 150000u);
  }
#endif

  SCB_AIRCR = 0x05FA0004;
  while (1);
}
