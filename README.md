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
| Throttle | EVWest dual-pot (OEM pedal) on A14 / A15 |
| Brake | Brake pressure sensor on A17 |

---

## CAN Bus Topology

### CAN1 — 500 kbps

Devices: pMBB32 battery management modules (×3), PDU-8 power distribution unit

**Transmitted (VCU → device)**

| ID | Type | Description |
|----|------|-------------|
| `0xFF0000` | Extended | Start-of-measurement broadcast to all pMBB32s |
| `0xCF0100 / 02 / 03` | Extended | Request min/max cells from pMBB32 #1 / #2 / #3 |
| `0xAF0100 / 02 / 03` | Extended | Set mode / wakeup / shutdown to pMBB32 #1 / #2 / #3 |
| `0x0A0620` | Extended | PDU-8 driver settings (channel current limits) |
| `0x0A0630` | Extended | PDU-8 driver outputs (PWM duty cycles) |

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

Devices: IVT-MOD, SIM100MOD, CAN keypad, OpenInverter Tesla LDU (v5 board), EMP WP29-12V-CV-A water pump

**Transmitted (VCU → device)**

| ID | Type | Rate | Description |
|----|------|------|-------------|
| `0x412` | Standard | on demand | IVT-MOD command (SET_MODE, configure measurements) |
| `0xA100101` | Extended | 200 ms | SIM100MOD isolation poll |
| `0x7A00` | Extended | 200 ms | EMP WP29 pump speed setpoint *(stub)* |
| `0x18EF2100` | Extended | on demand | CAN keypad LED colour / mode command |
| `0x201` | Standard | **10 ms** | OpenInverter LDU fixed safety frame (see below) |

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
| `0x622` | 60 ms | IVT-MOD pack voltage U1 |
| `0x623` | 60 ms | IVT-MOD pre-charge voltage U2 |
| `0x624` | 60 ms | IVT-MOD voltage U3 |
| `0x625` | 200 ms | IVT-MOD temperature |
| `0x626` | 30 ms | IVT-MOD power |
| `0x627` | 30 ms | IVT-MOD coulomb counter |
| `0x628` | 30 ms | IVT-MOD energy counter |
| `0x18EFFF21` | 100 ms | CAN keypad button status |
| `0xA100100` | on request | SIM100MOD isolation state / measurements |
| `0x19A` | — | OpenInverter LDU status *(TODO: confirm ID from inverter `can tx` output)* |
| `0x55A` | — | OpenInverter LDU faults *(TODO: confirm ID)* |
| `0xFBFE` | — | EMP WP29 pump actual speed / status *(stub)* |

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
| **1 Read** | `analogRead(A2)` and `analogRead(A3)` |
| **2 Verify** | Cross-check tracks within 5 %; mismatch → throttle = 0 |
| **3 Arbitrate** | Brake pedal pressed → 0; IVT or SIM fault active → clamp to 20 % |
| **4 Map** | Linear 1:1 pedal % → `LDUtorqueSetpoint` (0–100); zero outside Drive state |
| **5 Transmit** | `callback_t0()` scales to 12-bit `pot` and packs into 0x201 frame |

Key constants (`defines.h`):

| Constant | Value | Purpose |
|----------|-------|---------|
| `THROTTLE_PLAUSIBILITY_PCT` | 5 | Max allowed % gap between track 1 and track 2 |
| `THROTTLE_FAULT_LIMIT` | 20 | Max throttle % when IVT or SIM fault active |
| `THROTTLE_POT1/2_MIN` | 100 | ADC count at idle — **bench calibrate** |
| `THROTTLE_POT1/2_MAX` | 3900 | ADC count at full pedal — **bench calibrate** |

---

## Pin Assignments

| Pin | Function |
|-----|----------|
| 2 | KL15 input (ignition sense) |
| 3 | Loop timing debug output |
| 4 | CAN1 RX timing debug output |
| 5 | CAN2 RX timing debug output |
| 6 | Display timing debug output |
| 14 (TX3 / A0) | LIN bus TX |
| 15 (RX3 / A1) | LIN bus RX |
| 13 | Built-in LED (1 Hz heartbeat) |
| 18 (SDA) | GNSS I2C data — Wire / I2C0 |
| 19 (SCL) | GNSS I2C clock — Wire / I2C0 |
| 35 (D35) | GNSS 1PPS (`GPS_PPS_PIN`) — blue LED indicator on SK Pang board |
| A14 (pin 38) | Throttle pot 1 (EVWest dual-pot) |
| A15 (pin 39) | Throttle pot 2 (EVWest dual-pot) |
| A17 (pin 41) | Brake pedal pressure sensor (`BRAKE_PIN`) |

---

## State Machine

```
         KL15 ON              DRIVE_ON
  [Off] ---------> [Idle] -------------> [Drive]
    ^                |  \
    |    KL15 OFF    |   \ CHARGE_ON
    +----------------+    +-----------> [Charge]
```

| State | Entry action |
|-------|-------------|
| Off | Contactors disabled |
| Idle | Pre-charge relay enabled; keypad LED amber blink; "KL15 on" SMS sent |
| Drive | LDU start/forward/reverse controlled by keypad P/R/N/D buttons |
| Charge | *(TBD)* |

---

## Periodic Tasks

| Timer | Hardware | Period | Work |
|-------|----------|--------|------|
| **t0** | PIT (IntervalTimer) | **10 ms** | Throttle pipeline (read/verify/arbitrate/map); assemble and send LDU 0x201 safety frame |
| t1 | RTC | 62.5 ms | PDU-8 driver settings; pMBB32 min/max cell poll (round-robin); RealDash CAN3 update |
| t2 | GPT1 | 200 ms | pMBB32 measurement broadcast; stale module wakeup; SIM100MOD isolation poll; LIN valve poll |
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
| `ssilverman/QNEthernet` | Ethernet (reserved, not currently active) |
| `jonblack/arduino-fsm` | Finite state machine |
| `gicking/LIN master portable` | LIN master on Serial3 |

---

## Placeholders / TODOs

- **OpenInverter LDU v5** — run `can tx` in inverter terminal to confirm actual RX CAN IDs for status/fault frames; wire up `can2Sniff()` cases for `LDUrpm`, `LDUtorque`, `LDUmotorTemp`, etc.
- **OpenInverter CRC** — implement `crc_calculate_block` equivalent and set `controlcheck 1` on inverter once formula is confirmed from stm32-sine source
- **Brake calibration** — bench-calibrate `BRAKE_THRESHOLD` (currently 200 / 4095 ADC counts) against actual sensor output
- **Throttle calibration** — bench-calibrate `THROTTLE_POT1/2_MIN/MAX` constants
- **EMP WP29 pump** — confirm byte map from datasheet; implement speed setpoint loop
- **BMW LIN valve** — confirm LIN node address (`LIN_VALVE_ID`) and frame spec from BMW ISTA docs; assign `LIN_EN_PIN`
- **Pre-charge / contactor sequencing** — Idle state entry currently has a fixed 5 s delay; implement voltage-based pre-charge completion check using IVT-MOD U2 (pre-charge voltage)
- **Regen braking** — implement `pot2` / `regenpreset` fields in the LDU frame; wire to brake pressure or paddle
- **pMBB32 individual cell voltages** — broadcast frames received but not decoded in `can1Sniff()`
- **IVT fault detection** — populate `IVTfaultActive` in `can2Sniff()` when pack current or voltage is out of safe range
