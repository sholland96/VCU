/* sleep.cpp — deep-sleep (STOP mode) entry: gates clocks/peripherals/interrupts,
 * arms KL15R as the wake source, then WFI + AIRCR reset on wake.
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
  // Only if GNSS was actually brought up this boot — skipped entirely on a CAN-wake cycle
  // that never called myGNSS.begin() (see setup()); the library's internal I2C port pointer
  // may be unset otherwise.
  if (gnssInitialized) {
    // I2C cannot wake the module from backup mode; EXTINT0 is the supported source.
    myGNSS.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0);
    delay(500);
  }
#endif

  // Mask the Ethernet interrupt (IRQ_ENET). QNEthernet's driver enables this in the NVIC
  // during normal operation and enterSleep() never disabled it, so a live Ethernet link
  // (RealDash connected and streaming) could wake the CPU via RX/link-status interrupts
  // at any time — confirmed on hardware: RealDash actively connected made wake unreliable,
  // and remained unreliable even with Ethernet.begin() skipped entirely in setup(), since
  // the PHY chip still does link autonegotiation electrically once powered, independent of
  // whether the software driver ever touched it. Masking just the NVIC interrupt (rather
  // than powering down the PHY, which hung the board when tried previously) is safe with
  // nothing to undo — the NVIC enable bit resets to default on the AIRCR reset anyway.
  NVIC_DISABLE_IRQ(IRQ_ENET);

  // Disable the LIN UART (Serial6). Teensy's HardwareSerial RX interrupt stuffs bytes into
  // a ring buffer regardless of whether linReadValve() is actively polling, and — like
  // Ethernet — was never explicitly quiesced before sleep. With no LIN transceiver
  // connected, the RX pin floats and can pick up noise, spuriously firing this interrupt.
  // setup() calls linInit() fresh every boot, so ending it here is safe.
  Serial6.end();

  // Assert CAN transceiver standby — puts all three transceivers into low-power mode.
  digitalWrite(CAN_STBY_PIN, HIGH);

  // Gate FlexCAN peripheral clocks — stops internal CAN controller sampling.
  // AIRCR reset on wake restores all CCM clock gates to boot defaults.
  CCM_CCGR0 &= ~(CCM_CCGR0_CAN1(3) | CCM_CCGR0_CAN1_SERIAL(3) |
                 CCM_CCGR0_CAN2(3) | CCM_CCGR0_CAN2_SERIAL(3));
  CCM_CCGR7 &= ~(CCM_CCGR7_CAN3(3) | CCM_CCGR7_CAN3_SERIAL(3));

  // CAN2 wake re-enabled below for bench testing with a Kvaser dongle driving real, properly
  // terminated bus traffic. CAN3 (wireless gateway) stays disabled: nothing is connected to it
  // right now, and an unterminated/floating CAN bus was confirmed on hardware to pick up noise
  // that the MCP2562 transceiver's RXD output relays as spurious dominant edges, firing the
  // wake interrupt with no real traffic involved. Re-enable the same way once the gateway is
  // back in service: pinMode(..., INPUT_PULLUP) on CAN3_RX_PIN (CAN clock is already gated
  // above) plus attachInterrupt(..., FALLING) in the wake-arming block below.

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
  // CAN2 RXD: MCP2562 in standby still drives this low on dominant bus edges, independent of
  // the (clock-gated) CAN2 controller — see CAN2_RX_PIN in defines.h. Must be reconfigured as
  // a plain GPIO input here since FlexCAN_T4's begin() owns this pin's IOMUX the rest of the
  // time; initCAN() puts it back to CAN2 RX alternate function on the next boot.
  pinMode(CAN2_RX_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CAN2_RX_PIN), [](){}, FALLING); // EVCC/CAN2 activity
  // CAN3 wake source still temporarily disabled — see note above.
  SYST_CSR &= ~1u;  // disable SysTick (bit 0 = ENABLE) — stops 1 ms wakeups

  // STOP mode — gates more internal domains than WAIT mode.
  // AIRCR reset on wake restores all registers, so no clock restore needed.
  // To revert to WAIT mode: delete the two lines below.
  CCM_CLPCR = (CCM_CLPCR & ~0x3u) | 0x2u;  // LPM = 0b10 (STOP)
  SCB_SCR   |= (1u << 2);                    // SLEEPDEEP — WFI enters STOP not WAIT

  asm volatile("dsb");
  asm volatile("isb");
  if (!digitalRead(KL15R_PIN)) {
    asm volatile("wfi");  // sleep until KL15R rising edge
  }

  asm volatile("dsb");

  // Set SNVS_LPGPR based on wake cause. KL15R HIGH = key was turned → normal boot;
  // KL15R still LOW here would mean a non-KL15R wake source (CAN2, and CAN3 once
  // re-enabled). See defines.h for why this is SNVS_LPGPR (offset 0x068), not the
  // similarly-named but unimplemented-on-this-chip SNVS_LPGPR0 (offset 0x100).
  bool klrHighAtWake = digitalRead(KL15R_PIN);
  SNVS_LPGPR = klrHighAtWake ? 0 : SLEEP_MAGIC_CAN_WAKE;

#ifdef UBLOX_GNSS
  // Only if GNSS was actually brought up this boot — nothing to wake otherwise, and the
  // module's state (running or already in backup mode from an earlier cycle) is unaffected.
  if (gnssInitialized) {
    // Rising edge on EXTINT0 wakes the GNSS module for a hot-start while the Teensy resets.
    // Was timed via ARM_DWT_CYCCNT, but confirmed on hardware that the DWT cycle counter does
    // not reliably increment immediately after a STOP-mode wfi returns at this point — that
    // left this loop spinning forever (ARM_DWT_CYCCNT - t stuck at 0), hanging the board on
    // every wake attempt. A plain instruction-counting loop has no such dependency: it only
    // requires the CPU to be executing instructions at all, which it necessarily is here, so
    // it's guaranteed to terminate. Not precisely timed — just needs to be a clearly-visible
    // pulse width for the GNSS module's EXTINT input, slop in either direction is harmless.
    digitalWrite(GNSS_EXTINT_PIN, HIGH);
    for (volatile uint32_t i = 0; i < 300000u; i++) {}
  }
#endif

  // A dsb right after this write is ARM's documented pattern for a software system reset:
  // the store can sit in the write buffer for a few cycles before it actually reaches the
  // register, and without a barrier forcing that flush, execution can fall straight into
  // the while(1) below before the reset is ever actually committed — spinning forever
  // waiting for a reset that was never issued. Previously missing here.
  SCB_AIRCR = 0x05FA0004;
  asm volatile("dsb");
  while (1);
}
