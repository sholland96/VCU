# EK9 EV VCU

Vehicle Control Unit firmware for a Honda EK9 EV conversion, running on a Teensy 4.1.
Built with PlatformIO / Arduino framework.

---

## Hardware

| Item | Detail |
|------|--------|
| Carrier board | SK Pang Electronics Teensy 4.1 Triple CAN Board with ETH and u-blox NEO-M8M GNSS |
| MCU | PJRC Teensy 4.1 (ARM Cortex-M7 @ 600 MHz) |
| CAN | FlexCAN_T4 — two classic CAN + one CAN FD |
| LIN | Serial3 (TX3=pin 14 / A0, RX3=pin 15 / A1) — GNSS UART is on Serial2, no conflict |
| GNSS | u-blox NEO-M8M on I2C0 — Wire (SDA=pin 18, SCL=pin 19) |
| ADC | ADS1115 16-bit 4-ch ADC on I2C0 (addr 0x48, GAIN_ONE ±4.096 V, 860 SPS) |
| DCFC | Advantics ADM-CS-EVCC CCS/AC charge controller (CAN2 @ 500 kbps, Generic PEV protocol v2.x) |
| Throttle | EVWest dual-pot (OEM pedal) — ADS1115 AIN0 (track 1) / AIN1 (track 2) |
| Brake | Brake pressure sensor — ADS1115 AIN2 |

---

## CAN Bus Topology

### CAN1 — 500 kbps

Devices: pMBB32 battery management modules (×3), PDU-8 power distribution unit

**Transmitted (VCU → device)**

| ID | Type | Description |
|----|------|-------------|
| `0xFF0000` | Extended | Start-of-measurement broadcast to all pMBB32s — triggers cell voltage response frames |
| `0xCF0100 / 02 / 03` | Extended | Request min/max cell voltages from pMBB32 #1 / #2 / #3 |
| `0xAF0100 / 02 / 03` | Extended | pMBB32 mode command — see two-command startup sequence below |

**pMBB32 startup sequence (sent once at power-on via `wakepMBB32()`):**

Step 1 — Wake (3 bytes):

| Byte | Value | Meaning |
|------|-------|---------|
| 0 | `0x01` (`wakeup`) | Wake command |
| 1 | `0x10` (`channelCount16`) | Number of cell channels (16) |
| 2 | `0x02` (`numberOfDevices`) | Number of AFE ICs per module (2) |

Step 2 — Enable continuous reporting (1 byte, sent separately to each module):

| Byte | Value | Meaning |
|------|-------|---------|
| 0 | `0x10` (`contReportingEnable`) | Enable continuous cell voltage broadcast |

Step 3 — Send `0xFF0000` to trigger the first measurement cycle.

After startup, `0xFF0000` is sent every 200 ms to keep measurements running.

**Stale recovery** — each module has a counter incremented every 200 ms in `callback_t2()` and reset to 0 whenever a cell-voltage CAN frame arrives. If the counter exceeds 5 ticks (> 1 s without a frame), recovery begins:

1. **Shutdown** — send `0x55` (shutdown) to the stale module.
2. **Wake** — after 2 s, send wake + `contReportingEnable`; decrement retry credit.
3. Steps 1–2 repeat up to 3 times per module.
4. **CH2 power cycle** — if retries are exhausted, `PDUmsg1` CH2 is set to 0 (PDU-8 cuts power to all pMBB32s for 1 s), then restored. After a 3 s boot-settling wait, wake + `contReportingEnable` is sent to all three modules and retry credits reset. The cycle repeats indefinitely until all modules respond.
| `0x0A0620` | Extended | PDU-8 driver settings — PDUmsg1 - channel current limits (sent every 62.5 ms) |
| `0x0A0630` | Extended | PDU-8 driver outputs — PDUmsg2 - channel PWM duty cycles *(disabled, unverified)* |

**PDU-8 `0x0A0620` byte map** — current limit register: A ÷ 0.4 (e.g. 5 A → 0x0D, 2 A → 0x05, 0 = off)

| Byte | Channel | Load | Active value |
|------|---------|------|--------------|
| 0 | CH1 | Negative contactor | 0x0D (5 A) when KL15 on, 0x00 off |
| 1 | CH2 | pMBB32 modules | 0x05 (2 A) always |
| 2 | CH3 | Positive pre-charge relay | 0x05 (2 A) during Idle pre-charge, 0x00 otherwise |
| 3 | CH4 | Positive contactor | 0x0D (5 A) when KL15 on, 0x00 off |
| 4–7 | — | Unused | 0x00 |

**PDU-8 `0x0A0630` byte map** — PWM duty cycle (0x00–0xFF); only needed for current-controlled outputs

| Byte | Channel | Load |
|------|---------|------|
| 0 | CH1 | Negative contactor |
| 1 | CH2 | pMBB32 modules (init value 0xFE) |
| 2 | CH3 | Positive pre-charge relay |
| 3 | CH4 | Positive contactor |
| 4–7 | — | Unused |

**Received and decoded**

| ID | Description |
|----|-------------|
| `0x18FF0E01 / 02 / 03` | pMBB32 #1 / #2 / #3 min/max cell voltage report |

**Received — defined in pMBB32.h, not yet decoded**

| ID Pattern | Description |
|------------|-------------|
| `0x18FF01yy / 0x18FF07yy` | Cells 1–4 voltages (low IC / high IC) |
| `0x18FF02yy / 0x18FF08yy` | Cells 5–8 voltages |
| `0x18FF03yy / 0x18FF09yy` | Cells 9–12 voltages |
| `0x18FF04yy / 0x18FF0Ayy` | Cells 13–16 voltages |
| `0x18FF05yy / 0x18FF0Byy` | Aux channel readings |
| `0x18FF06yy / 0x18FF0Cyy` | PCB / die temperatures |
| `0x18FF0Fyy / 0x18FF10yy` | Status registers |
| `0x18FF11yy / 0x18FF12yy` | Cell OV/UV fault status |

*(yy = module number 01, 02, 03)*

---

### CAN2 — 500 kbps

Devices: IVT-MOD, SIM100MOD, CAN keypad, OpenInverter Tesla LDU (v5 board), EMP WP29-12V-CV-A water pump, Advantics ADM-CS-EVCC DC fast charge controller

**Transmitted (VCU → device)**

| ID | Type | Rate | Description |
|----|------|------|-------------|
| `0x412` | Standard | on demand | IVT-MOD command (SET_MODE, configure measurements) |
| `0xA100101` | Extended | 200 ms | SIM100MOD isolation poll |
| `0x18EF{pump}{vcu}` | Extended | 200 ms | EMP WP29 pump Motor Command (byte 0: 0xFD=on/0xFC=off; byte 3: %×2) |
| `0x18EF2100` | Extended | on demand | CAN keypad LED colour / mode command |
| `0x201` | Standard | **10 ms** | OpenInverter LDU fixed safety frame (see below) |
| `0x610` | Standard | 62.5 ms | EVCC EV_Information — State_of_Charge (%), Energy_Capacity (kWh × 0.1) |
| `0x612` | Standard | 62.5 ms | EVCC DC_Status1 — Max_Charge_Current, Present_Current, Max_Discharge_Current, Target_Voltage |
| `0x613` | Standard | 62.5 ms | EVCC DC_Status2 — Contactors_Closed, Normal_End_of_Charge, Emergency_Stop, Battery_Voltage (IVT U1), Inlet_Voltage (IVT U3) |

**OpenInverter LDU 0x201 frame — v5.32+ fixed bit-packed layout**

```
data[0] (bytes 0–3, little-endian uint32):
  bits  0–11 : pot          throttle demand 0–4095
  bits 12–23 : pot2         regen channel (0 = unused)
  bits 24–29 : canio        6-bit digital IO field (see below)
  bits 30–31 : ctr1         2-bit sequence counter

data[1] (bytes 4–7, little-endian uint32):
  bits  0–13 : cruisespeed  cruise target (0 = unused)
  bits 14–15 : ctr2         must equal ctr1 every frame
  bits 16–23 : regenpreset  regen % (0 = unused)
  bits 24–31 : crc          optional CRC (set controlcheck=0 to disable)
```

canio bitfield:

| Bit | Mask | Signal |
|-----|------|--------|
| 0 | `0x01` | cruise |
| 1 | `0x02` | start / enable |
| 2 | `0x04` | brake |
| 3 | `0x08` | forward |
| 4 | `0x10` | reverse |
| 5 | `0x20` | bms |

Safety: `ctr1` must equal `ctr2` and must differ from the previous frame's counter value. The inverter shuts down after 5 consecutive invalid frames or 500 ms of silence. The VCU sends every 10 ms.

**One-time inverter terminal setup (run once, then `save`):**
```
potmode      2       enable CAN throttle
potmin       0
potmax       4095
controlid    513     (= 0x201 decimal)
controlcheck 0       disable CRC for now
save
```

**Received (device → VCU)**

| ID | Rate | Description |
|----|------|-------------|
| `0x621` | 20 ms | IVT-MOD current |
| `0x622` | 60 ms | IVT-MOD pack voltage U1 (Pack+) |
| `0x623` | 60 ms | IVT-MOD pre-charge voltage U2 (DC-Link+) |
| `0x624` | 60 ms | IVT-MOD DCFC inlet voltage U3 (DCFC+) |
| `0x625` | 200 ms | IVT-MOD temperature |
| `0x626` | 30 ms | IVT-MOD power |
| `0x627` | 30 ms | IVT-MOD coulomb counter |
| `0x628` | 30 ms | IVT-MOD energy counter |
| `0x18EFFF21` | on event | CAN keypad button press / release |
| `0xA100100` | on request | SIM100MOD isolation state / measurements |
| `0x19A` | — | OpenInverter LDU status *(TODO: confirm ID from inverter `can tx` output)* |
| `0x55A` | — | OpenInverter LDU faults *(TODO: confirm ID)* |
| `0x18FF03{pump}` | 1 Hz | EMP WP29 Motor Status Message 1 (speed, temp, power, controller status) |
| `0x18FF24{pump}` | 100 ms | EMP WP29 Motor Status Message 3 (voltage, current, HVIL status) |
| `0x600` | ~100 ms | EVCC Communication_Stage, Protocol, Pins, Max_Current |
| `0x602` | ~100 ms | EVCC Close_Contactors signal (EVCC drives its own hardware; VCU monitors only) |
| `0x604` | ~1 s | EVCC DC contactor positive/negative feedback, temperatures |
| `0x605` | on event | EVCC diagnostic fault flags |

---

### CAN3 — 1 Mbps

Devices: Wireless gateway, RealDash

**Transmitted (VCU → RealDash / gateway)**

| ID | Description |
|----|-------------|
| `0xC79` | SMS command to wireless gateway |
| `0xC80` | RPM, power, temperature, throttle (every 62.5 ms) |
| `0xC81` | Pack voltage, pack current, 12 V battery voltage |
| `0xC82` | Highest/lowest cell voltage, ground speed, GPS altitude |
| `0xC83` | Cell delta voltage, SIM100MOD isolation, SIM100MOD temperature, GPS fix type |

**Received**

| ID | Description |
|----|-------------|
| `0x1F4` | Received but not yet handled |

---

## LIN Bus — 19200 baud (Serial3)

| Device | Direction | Node ID | Notes |
|--------|-----------|---------|-------|
| BMW i4/i5/i7/iX Changeover Valve 64119462114 | Slave response | `0x10` *(TBD)* | Byte map TBD from BMW ISTA docs |

Uses `gicking/LIN master portable` library (`LIN_Master_HardwareSerial`).

---

## Throttle Pipeline

`readThrottle()` is called from `callback_t0()` every 10 ms. Five stages:

| Stage | Action |
|-------|--------|
| **1 Read** | ADS1115 AIN0 (track 1) and AIN1 (track 2) — polled non-blocking in `loop()` at 860 SPS, time-gated at 1300 µs per channel to share I2C0 with GNSS |
| **2 Verify** | Cross-check tracks within 5 %; mismatch → throttle = 0 |
| **3 Arbitrate** | Brake pedal pressed → 0; IVT or SIM fault active → clamp to 20 % |
| **4 Map** | Linear 1:1 pedal % → `LDUtorqueSetpoint` (0–100); zero outside Drive state |
| **5 Transmit** | `callback_t0()` scales to 12-bit `pot` and packs into 0x201 frame |

Key constants (`defines.h`):

| Constant | Value | Purpose |
|----------|-------|---------|
| `THROTTLE_PLAUSIBILITY_PCT` | 5 | Max allowed % gap between track 1 and track 2 |
| `THROTTLE_FAULT_LIMIT` | 20 | Max throttle % when IVT or SIM fault active |
| `THROTTLE_POT1/2_MIN` | 800 | ADS1115 counts at idle — **bench calibrate** |
| `THROTTLE_POT1/2_MAX` | 31200 | ADS1115 counts at full pedal — **bench calibrate** |
| `BRAKE_THRESHOLD` | 1600 | ADS1115 counts at which brake pedal is considered pressed — **bench calibrate** |

---

## Pin Assignments

| Pin | Function |
|-----|----------|
| 0 | CAN2 RXD wake input (`CAN2_RX_PIN`) — MCP2562 drives this low on bus activity; falling-edge interrupt wakes VCU from STOP mode for EVCC charge sessions |
| 2 | KLR input (`KLR_PIN`) — key position 1, wakes hardware |
| 3 | Loop timing debug output |
| 4 | CAN1 RX timing debug output |
| 5 | CAN2 RX timing debug output |
| 6 | `displayStatus()` timing debug output |
| 14 (TX3 / A0) | LIN bus TX |
| 15 (RX3 / A1) | LIN bus RX |
| 13 | Built-in LED (1 Hz heartbeat) |
| 18 (SDA) | GNSS I2C data — Wire / I2C0 |
| 19 (SCL) | GNSS I2C clock — Wire / I2C0 |
| 16 (RX4) | SIM7080G UART RX (`SIM7080_SERIAL` = Serial4) |
| 17 (TX4) | SIM7080G UART TX |
| 36 | SIM7080G PWRKEY (`SIM7080_PWRKEY`) — pulse LOW ~1 s to power on module |
| 32 | CAN transceiver standby (`CAN_STBY_PIN`) — driven HIGH during sleep to put all three transceivers into standby. STBY pins lifted from GND on SK Pang board and wired to this pin. |
| 33 | GNSS EXTINT (`GNSS_EXTINT_PIN`) — wire to SK Pang GNSS EXTINT header; pulsed HIGH before reset to wake module from backup |
| 34 | TPS131PXQ1EVM-400 active pre-charge enable (`PRECHARGE_EN_PIN`) — driven HIGH during PreCharge state |
| 35 (D35) | GNSS 1PPS (`GPS_PPS_PIN`) — blue LED indicator on SK Pang board |
| 18 (SDA) / 19 (SCL) | ADS1115 AIN0: throttle track 1; AIN1: throttle track 2; AIN2: brake pressure (I2C0, shared with GNSS) |

---

## State Machine

The VCU uses two top-level regions separated by the physical key switch:

### KLR region (key position 1)

Turning the key to position 1 (KLR) powers up the Teensy, display and all controllers. The VCU boots in the **Off** state and waits for the KL15 start button (keypad button 5).

Turning the key off (KLR low) while in the Off state triggers `enterSleep()` after a 500 ms debounce (KLR must be continuously low for 500 ms). The debounce gives the EVCC five broadcast periods to announce itself after a CAN2 wake before sleep is re-entered. `enterSleep()` returns immediately if `EVCCstage > 0` (active DCFC session), keeping the VCU awake without KLR.

1. All four timers stop; heartbeat LED is forced off.
2. USB PHY is powered down (`USBPHY1_PWD = 0xFFFFFFFF`) and its CCM clock gated.
3. GNSS is put into backup mode via `UBX-RXM-PMREQ` (`powerOffWithInterrupt`, EXTINT0 wake source, ~15 µA).
4. FlexCAN1/2/3 peripheral clocks are gated off via `CCM_CCGR0` / `CCM_CCGR7` — stops internal CAN controller sampling.
5. `CAN_STBY_PIN` (pin 32) is driven HIGH — all three CAN transceivers enter standby mode.
6. CPU clock is reduced to ~16.2 MHz (ARM PLL minimum; DCDC core voltage drops to 0.95 V), then AHB is switched to the 24 MHz crystal, ARM PLL bypass is enabled (CPU runs at crystal / ARM_PODF ≈ 3 MHz), and the ARM PLL VCO is powered down.
7. CAN2 RXD pin (pin 0) is reconfigured as GPIO input with pull-up. Rising-edge interrupt on `KLR_PIN` and falling-edge interrupt on `CAN2_RX_PIN` are attached as dual wake sources; SysTick is disabled. The MCP2562 transceiver drives RXD low on dominant bus edges even in standby, so EVCC CAN traffic wakes the VCU without KLR.
8. `CCM_CLPCR[LPM]` is set to STOP (0b10) and `SCB_SCR[SLEEPDEEP]` is set — a single `wfi` then enters IMXRT1062 STOP mode, gating internal power domains beyond what WAIT mode achieves.

On wake, a rising edge is asserted on pin 33 (EXTINT0) via DWT cycle-counter delay to start the GNSS hot-start before the Teensy resets; `SCB_AIRCR` resets the chip so `setup()` re-initialises all peripherals (including clock restoration) cleanly.

**Measured sleep current: ~4 mA at 12 V** (external 90–95 % efficient 12 V → 5 V switcher + Teensy onboard 3.3 V LDO). Down from ~61 mA before sleep optimisations — a 93 % reduction.

### On State (KL15 active)

Pressing keypad button 5 (KL15) fires `KL15_ON` and initiates the drive-enable sequence. The entire On State is exited by pressing the **Park** button while the vehicle is stationary (`LDUrpm == 0`), which fires `KL15_OFF` and disables the contactors.

```
  [KLR / Off] ──── KL15_ON (btn 5) ────> [PreCharge]
       ^                                      │ IVT U2 ≥ 95% U1 within 2 s
       │ KL15_OFF                             ├──────────────────────> [Idle]
       │ (Park btn, speed = 0)                │ EVCC chargeMode = true
       │ or FAULT_CLEAR                       ├──────────────────────> [Charge]
       │ (Park btn, speed = 0)                │ timeout or FAULT_EV
       │                                      └──────────────────────> [Fault]
       │
       │          DRIVE_ON (btn 8)      DRIVE_OFF (btn 1, speed = 0)
       │          [Idle] ──────────────> [Drive] ──────────────────> [Idle]
       │
       │          TEMP_LOW / TEMP_HIGH                    TEMP_OK
       │          [Idle/Drive/Charge] ──────> [HeatPack / CoolPack] ──> [Idle]
       │
       └── KL15_OFF (Park btn, speed = 0) from any On state ──────> [Off]
```

### State table

| State | Entry action | Exit condition |
|-------|-------------|----------------|
| Off | All contactors off; awaiting KL15 | Button 5 pressed → `KL15_ON` |
| PreCharge | Negative contactor on; pre-charge relay on; IVT U2 monitored | U2 ≥ 95% U1 → `PRECHARGE_OK`; timeout → `PRECHARGE_FAIL` |
| Idle | Positive contactor on; keypad amber blink; "KL15 on" SMS | Button 8 (Drive) → `DRIVE_ON`; Button 1 (Park) + speed=0 → `KL15_OFF` |
| Drive | LDU enabled; direction from keypad N/R/D | Button 1 (Park) + speed=0 → `DRIVE_OFF` (→ Idle) |
| Charge | EVCC-initiated; contactors remain closed | EVCC stop request → `CHARGE_OFF`; Button 1 + speed=0 → `KL15_OFF` |
| HeatPack | Pump on; heater on *(TODO)* | BMS temp in range → `TEMP_OK`; Button 1 + speed=0 → `KL15_OFF` |
| CoolPack | Pump on; AC exchanger on *(TODO)* | BMS temp in range → `TEMP_OK`; Button 1 + speed=0 → `KL15_OFF` |
| Fault | All contactors off; keypad red blink; fault SMS | Button 1 (Park) + speed=0 → `FAULT_CLEAR` (→ Off) |

`reducedPowerActive` flag (set by BMS temp fault during Drive/Charge) clamps throttle to `THROTTLE_FAULT_LIMIT` without leaving Drive state.

---

## CAN Keypad Button Assignment

Keypad: RX `0x18EFFF21`, TX `0x18EF2100` (CAN2 @ 500 kbps)

Key Contact state message format (PKP2400SI J1939 §6, event-driven by default):

| Byte | Value | Meaning |
|------|-------|---------|
| 0 | `0x04` | Header |
| 1 | `0x1B` | Header |
| 2 | `0x01` | Key Contact state command |
| 3 | `1`–`8` | Key number |
| 4 | `0x00` / `0x01` | Released / Pressed |
| 5 | `0x21` | Keypad identifier |

| Button | Label | Function |
|--------|-------|----------|
| 1 | Park | In Drive: return to Idle (speed = 0 required). In Idle/Charge/HeatPack/CoolPack/Fault: exit On State (speed = 0 required). |
| 2 | Reverse | Set LDU direction to reverse |
| 3 | Neutral | Set LDU direction to neutral |
| 4 | Drive | In Idle: enter Drive state; set LDU direction to forward |
| 5 | KL15 / Start | Enter On State (fires `KL15_ON`); one-shot — Park exits |
| 6 | Speed Mode | *(undefined — reserved)* |
| 7 | Auxiliary | *(undefined — reserved)* |
| 8 | Drive Mode | *(undefined — reserved)* |

---

## Periodic Tasks

| Timer | Hardware | Period | Work |
|-------|----------|--------|------|
| **t0** | PIT (IntervalTimer) | **10 ms** | Throttle pipeline (read/verify/arbitrate/map); assemble and send LDU 0x201 safety frame |
| t1 | RTC | 62.5 ms | PDU-8 driver settings; pMBB32 min/max cell poll (round-robin); RealDash CAN3 update; EVCC battery status (0x610/0x612/0x613) |
| t2 | GPT1 | 200 ms | Send `0xFF0000` measurement trigger; invalidate stale module data; stale module recovery (wake + contReportingEnable); SIM100MOD isolation poll; LIN valve poll |
| t3 | GPT2 | 1000 ms | Heartbeat LED toggle |
| main loop | — | free-running | CAN event dispatch; GNSS processing; FSM step |

t0 runs at the highest ARM Cortex-M7 NVIC priority (`priority(0)`) and preempts all other work.

---

## Building

Requires [PlatformIO](https://platformio.org/). Open the project folder in VS Code and click **Build (✓)** in the PlatformIO toolbar.

### Dependencies (auto-installed by PlatformIO)

| Library | Purpose |
|---------|---------|
| `luni64/TeensyTimerTool` | Periodic timer callbacks (RTC, GPT1, GPT2) |
| `sparkfun/SparkFun u-blox GNSS Arduino Library` | GNSS / GPS |
| `jonblack/arduino-fsm` | Finite state machine |
| `gicking/LIN master portable` | LIN master on Serial3 |
| `adafruit/Adafruit ADS1X15` | ADS1115 16-bit ADC driver |

---

## Placeholders / TODOs

- **OpenInverter LDU v5** — run `can tx` in inverter terminal to confirm actual RX CAN IDs for status/fault frames; wire up `can2Sniff()` cases for `LDUrpm`, `LDUtorque`, `LDUmotorTemp`, etc.
- **OpenInverter CRC** — implement `crc_calculate_block` equivalent and set `controlcheck 1` on inverter once formula is confirmed from stm32-sine source
- **Brake calibration** — bench-calibrate `BRAKE_THRESHOLD` (ADS1115 counts) against actual sensor output
- **Throttle calibration** — bench-calibrate `THROTTLE_POT1/2_MIN/MAX` (ADS1115 counts; current values are ×8 approximations of old 12-bit readings)
- **EVCC calibration** — set `EVCC_CELL_V_EMPTY` / `EVCC_CELL_V_FULL` (pMBB32 raw counts) for actual cell chemistry; set `EVCC_MAX_CHARGE_A` and `EVCC_TARGET_V` for pack limits; set `EVCC_PACK_ENERGY_X10` for actual pack capacity
- **EMP WP29 pump** — confirm pump J1939 source address (`EMP_WP29_ADDR` in defines.h, currently `0x8A`) via CAN sniffer or DBC file; remove CH3 passive pre-charge relay command from `PreCharge_enter()` once active pre-charge board is fitted
- **BMW LIN valve** — confirm LIN node address (`LIN_VALVE_ID`) and frame spec from BMW ISTA docs; assign `LIN_EN_PIN`
- **Pre-charge / contactor sequencing** — Idle state entry currently has a fixed 5 s delay; implement voltage-based pre-charge completion check using IVT-MOD U2 (pre-charge voltage)
- **Regen braking** — implement `pot2` / `regenpreset` fields in the LDU frame; wire to brake pressure or paddle
- **pMBB32 individual cell voltages** — broadcast frames received but not decoded in `can1Sniff()`
- **IVT fault detection** — populate `IVTfaultActive` in `can2Sniff()` when pack current or voltage is out of safe range
