# EK9 EV VCU

Vehicle Control Unit firmware for a Honda EK9 EV conversion, running on a Teensy 4.1.
Built with PlatformIO / Arduino framework.

---

## Hardware

| Item | Detail |
|------|--------|
| MCU | PJRC Teensy 4.1 (ARM Cortex-M7 @ 600 MHz) |
| CAN | FlexCAN_T4 — three independent buses |
| LIN | Serial3 (RX3=pin 7, TX3=pin 8) via external transceiver |
| GNSS | u-blox module on I2C (400 kHz) |
| Throttle | EVWest dual-pot (OEM pedal) on A2 / A3 |

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

Devices: IVT-MOD, SIM100MOD, CAN keypad, OpenInverter Tesla LDU V2 *(placeholder)*, EMP WP29-12V-CV-A water pump *(placeholder)*

**Transmitted (VCU → device)**

| ID | Type | Description |
|----|------|-------------|
| `0x412` | Standard | IVT-MOD command (SET_MODE, configure measurements) |
| `0xA100101` | Extended | SIM100MOD request command |
| `0x7A00` | Extended | EMP WP29 pump speed setpoint |
| `0x18EF2100` | Extended | CAN keypad LED color / mode command |
| `0x19B` | Standard | OpenInverter LDU torque command *(stub, not yet active)* |

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
| `0x19A` | — | OpenInverter LDU status (speed, torque, temperature) *(placeholder)* |
| `0x55A` | — | OpenInverter LDU faults *(placeholder)* |
| `0xFBFE` | — | EMP WP29 pump actual speed / status *(placeholder)* |

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

## Pin Assignments

| Pin | Function |
|-----|----------|
| 2 | KL15 input (ignition sense) |
| 3 | Loop timing debug output |
| 4 | CAN1 RX timing debug output |
| 5 | CAN2 RX timing debug output |
| 6 | Display timing debug output |
| 7 (RX3) | LIN bus RX |
| 8 (TX3) | LIN bus TX |
| 13 | Built-in LED (1 Hz heartbeat) |
| A2 | Throttle pot 1 (EVWest dual-pot) |
| A3 | Throttle pot 2 (EVWest dual-pot) |
| I2C | u-blox GNSS module |

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
| Drive | — *(drive logic TBD)* |
| Charge | — *(charge logic TBD)* |

---

## Periodic Tasks

| Timer | Period | Work |
|-------|--------|------|
| t1 (RTC) | 62.5 ms | PDU-8 driver settings; pMBB32 min/max cell poll (round-robin); RealDash CAN3 update |
| t2 (GPT1) | 200 ms | pMBB32 measurement broadcast; stale module wakeup; SIM100MOD isolation poll; LIN valve poll |
| t3 (GPT2) | 1000 ms | Heartbeat LED toggle |
| main loop | free-running | CAN event dispatch; GNSS processing; throttle ADC read; FSM step |

---

## Building

Requires [PlatformIO](https://platformio.org/). Open the project folder in VS Code and click **Build (✓)** in the PlatformIO toolbar, or use the PlatformIO terminal:

```
pio run
```

### Dependencies (auto-installed by PlatformIO)

| Library | Purpose |
|---------|---------|
| `luni64/TeensyTimerTool` | Periodic timer callbacks |
| `sparkfun/SparkFun u-blox GNSS Arduino Library` | GNSS / GPS |
| `ssilverman/QNEthernet` | Ethernet (reserved, not currently active) |
| `jonblack/arduino-fsm` | Finite state machine |
| `gicking/LIN master portable` | LIN master on Serial3 |

---

## Placeholders / TODOs

- **OpenInverter LDU V2** — confirm CAN IDs from device configuration page; implement torque command
- **EMP WP29 pump** — confirm byte map from datasheet; implement speed setpoint loop
- **BMW LIN valve** — confirm LIN node address and frame spec from BMW ISTA; implement write frame
- **Throttle calibration** — bench-calibrate `THROTTLE_POT1/2_MIN/MAX` constants in `defines.h`
- **LIN transceiver enable pin** — assign `LIN_EN_PIN` once hardware is finalised
- **Pre-charge / contactor sequencing** — Idle state entry currently has a fixed 5 s delay; implement voltage-based pre-charge completion check
- **pMBB32 individual cell voltages** — broadcast frames are received but not yet decoded in `can1Sniff()`
