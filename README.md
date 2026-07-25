# EK9 EV VCU

Vehicle Control Unit firmware for a Honda EK9 EV conversion, running on a Teensy 4.1.
Built with PlatformIO / Arduino framework.

---

## Hardware

### Core electronics

| Item | Detail |
|------|--------|
| Carrier board | SK Pang Electronics Teensy 4.1 Triple CAN Board with ETH and u-blox NEO-M8M GNSS |
| MCU | PJRC Teensy 4.1 (ARM Cortex-M7 @ 600 MHz) |
| CAN transceivers | 3× on SK Pang board (CAN1/2/3); STBY pins lifted from GND and wired to pin 32 — driven HIGH in sleep to put all three into standby |
| LIN transceivers | 2× MCP2003B *(placeholder — TBD)* — bus 1 on Serial6 (TX6=pin 24, RX6=pin 25); bus 2 *(planned)* on Serial7 (TX7=pin 29, RX7=pin 28) |
| GNSS | u-blox NEO-M8M on I2C0 — Wire (SDA=pin 18, SCL=pin 19) |
| ADC | ADS1115 16-bit 4-ch ADC on I2C0 (addr 0x48, GAIN\_ONE ±4.096 V, 860 SPS) |

### Powertrain

| Item | Detail |
|------|--------|
| Current/voltage sensor | Isabellenhuette IVT-S-1K-U3-I-CAN1-12V — CAN2 @ 500 kbps; measures pack current, pack voltage U1, pre-charge voltage U2, DCFC inlet voltage U3, temperature, power, Ah/Wh counters |
| Auxiliary DC-DC | Chery New Energy combo OBC+DCDC unit — HV → 12 V auxiliary supply half of the unit; CAN2 @ 500 kbps (planned); see [Charging](#charging) for the on-board charger half and CAN protocol |
| Active pre-charge | Texas Instruments TPS131PXQ1EVM-400 evaluation board — enable via pin 34 (3.3 V HIGH); passive pre-charge relay (PDU-8 CH3) used in parallel until active board is commissioned |
| Isolation monitor | SIM100MOD — CAN2 @ 500 kbps; reports isolation resistance (Rp kΩ) and temperature |

### Charging

| Item | Detail |
|------|--------|
| DCFC controller | Advantics ADM-CS-EVCC CCS / CHAdeMO — CAN2 @ 500 kbps; Generic Power Modules extended-ID protocol + v2.5 standard-ID AC handshake |
| OBC | Chery New Energy combo OBC+DCDC unit, AC on-board charger half — CAN2 @ 500 kbps (planned); DBC supplied by vendor, decoded below. **Supersedes** the earlier Elcon UHF-CAN-312 placeholder; VCU AC contactor path (unchanged) is implemented, command/telemetry handling *(TODO)* |

### Thermal management

| Item | Detail |
|------|--------|
| Battery cooling pump | EMP WP29-12V-CV-A — CAN1 @ 500 kbps; Motor Command `0x18EF{pump}{vcu}` every 200 ms (byte 0: 0xFD=on / 0xFC=off; byte 3: speed %×2) |
| Inverter cooling pump | EMP WP29-12V-CV-A — CAN2 @ 500 kbps; same frame layout |
| Coolant changeover valves | 2× BMW 64119462114 — LIN slaves; node address *(TBD from BMW ISTA docs)* |

### Driver interface

| Item | Detail |
|------|--------|
| Throttle | EVWest dual-pot (OEM pedal) — ADS1115 AIN0 (track 1) / AIN1 (track 2) |
| Brake | Brake pressure sensor — ADS1115 AIN2 |

---

## CAN Bus Topology

### CAN1 — 500 kbps

Devices: pMBB32 battery management modules (×3), PDU-8 power distribution unit, EMP WP29-12V-CV-A battery cooling pump

**Transmitted (VCU → device)**

| ID | Type | Description |
|----|------|-------------|
| `0xFF0000` | Extended | Start-of-measurement broadcast to all pMBB32s — triggers cell voltage response frames |
| `0xCF0100 / 02 / 03` | Extended | Request min/max cell voltages from pMBB32 #1 / #2 / #3 |
| `0xAF0100 / 02 / 03` | Extended | pMBB32 mode command — see two-command startup sequence below |

**pMBB32 startup sequence (sent once at power-on via `wakepMBB32()`, after a 500 ms PDU CH2 settle delay):**

Wake command (3 bytes, sent to each module's SA address):

| Byte | Value | Meaning |
|------|-------|---------|
| 0 | `0x01` (`wakeup`) | Wake and initialise BMS ASICs |
| 1 | `0x10` (`channelCount16`) | Number of cell channels (16) |
| 2 | `0x02` (`numberOfDevices`) | Number of AFE ICs per module (2) |

After wake, `0xFF0000` is sent every 200 ms to trigger measurement cycles (trigger-based polling). Continuous auto-broadcast mode (enabled by sending `[0x10]` to each module's SA) is not used — polling gives the VCU explicit control over measurement timing and simplifies stale detection.

**Stale recovery** — each module has a counter incremented every 200 ms in `callback_t2()` and reset to 0 whenever a cell-voltage CAN frame (ft=01..0C) arrives. A 10 s startup grace period suppresses recovery while modules complete initialisation. After the grace period, if the counter exceeds 5 ticks (> 1 s without a frame), recovery begins:

1. **Shutdown** — send `0x55` (shutdown) to the stale module.
2. **Wake** — after 500 ms, send the wake command; decrement retry credit.
3. Steps 1–2 repeat up to 3 times per module.
4. **CH2 power cycle** — if retries are exhausted, `PDUmsg1` CH2 is set to 0 (PDU-8 cuts power to all pMBB32s for 1 s), then restored. After a 1 s boot-settling wait, wake is sent to all three modules and retry credits reset. The cycle repeats indefinitely until all modules respond.

**Corruption detection** — two silent failure modes are detected and corrected automatically:

- **Ghost SA** (`TOTAL_ICS` corruption): if a pMBB32 module's `TOTAL_ICS` is corrupted during initialisation it broadcasts on SA+0..SA+7, flooding the bus with up to 96 frames per trigger while the legitimate SA=1..3 frames still reset the stale counter. Detected by watching for `modNum ≥ 4` in `0x18FFxxxx` frames. If sustained for 2 s (debounced), a CH2 power cycle is triggered.
- **Absent ft=03** (`numChannels` corruption): if `numChannels` is corrupted to 0, cells 9–32 are silently omitted (the module only broadcasts ft=01/02) while stale counters continue to reset normally. Detected every 5 s by checking whether ft=01 arrived without ft=03 in the same window. If detected, the affected module's stale counter is forced to 255 to trigger shutdown→wake recovery; the check is then suppressed for 30 s to allow recovery to complete.

Both detections are cleared when a CH2 power cycle completes.
| `0x0A0620` | Extended | PDU-8 driver settings — PDUmsg1 - channel current limits (sent every 62.5 ms) |
| `0x0A0630` | Extended | PDU-8 driver outputs — PDUmsg2 - channel PWM duty cycles *(disabled, unverified)* |
| `0x18EF{pump}{vcu}` | Extended | EMP WP29 battery cooling pump Motor Command (sent every 200 ms via CAN1; byte 0: 0xFD=on/0xFC=off; byte 3: %×2) |

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
| `0x18FF03{pump}` | EMP WP29 battery cooling pump Motor Status 1 (1 Hz — speed, temp, power, controller status) |
| `0x18FF24{pump}` | EMP WP29 battery cooling pump Motor Status 3 (100 ms — voltage, current, HVIL) |

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

Devices: Isabellenhuette IVT-S-1K-U3-I-CAN1-12V, SIM100MOD, CAN keypad, OpenInverter Tesla LDU (v5 board), EMP WP29-12V-CV-A inverter cooling pump, Advantics ADM-CS-EVCC DC fast charge controller

**Transmitted (VCU → device)**

| ID | Type | Rate | Description |
|----|------|------|-------------|
| `0x412` | Standard | on demand | IVT-S command (SET_MODE, configure measurements) |
| `0xA100101` | Extended | 200 ms | SIM100MOD isolation poll |
| `0x18EF{pump}{vcu}` | Extended | 200 ms | EMP WP29 inverter cooling pump Motor Command (CAN2; byte 0: 0xFD=on/0xFC=off; byte 3: %×2) |
| `0x18EF2100` | Extended | on demand | CAN keypad LED colour / mode command |
| `0x201` | Standard | **10 ms** | OpenInverter LDU fixed safety frame (see below) |
| `0x60010` | Extended | 62.5 ms | EVCC Power_Modules_Status — Present_Voltage (IVT U1, 0.1 V), Present_Current (IVT I, 0.1 A signed), System_Enable, Insulation_R (SIM100MOD Rp, 2 kΩ/bit) |
| `0x60011` | Extended | 62.5 ms | EVCC Power_Modules_Limits — Max_Voltage (0.1 V), Max_Current (0.1 A signed) |
| `0x60012` | Extended | 62.5 ms | EVCC Sequence_Control — Start_Charge_Authorisation, CHAdeMO_Start_Button, CCS_Auth_Done/Valid, Charge_Parameters_Done, User_Stop_Button |
| `0x611` | **Standard** | 62.5 ms | EVCC AC_Status (v2.5) — `Ready_To_Charge` bit 0; set while VCU is in Charge state, EVCC has granted delivery (`acReadyToDeliver`), and battery is not full; cleared immediately on `Charge_exit()` |
| `0x0CE97FC4` | Extended | *(TODO)* | Chery OBC+DCDC — `VCU_DCDC_Command`: DCDC_Enable, DCDC_TargetVoltage (see below) |
| `0x104CE8DC` | Extended | *(TODO)* | Chery OBC+DCDC — `VCU_OBC_Command`: OBC_TargetVoltage, OBC_TargetCurrent, OBC_ChargeControl (see below) |

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
| `0x621` | 20 ms | IVT-S current (mA, signed) |
| `0x622` | 60 ms | IVT-S pack voltage U1 — Pack+ (mV) |
| `0x623` | 60 ms | IVT-S pre-charge voltage U2 — DC-Link+ (mV) |
| `0x624` | 60 ms | IVT-S DCFC inlet voltage U3 — DCFC+ (mV) |
| `0x625` | 200 ms | IVT-S temperature (°C, signed) |
| `0x626` | 30 ms | IVT-S power (W, signed) |
| `0x627` | 30 ms | IVT-S coulomb counter (As, signed) |
| `0x628` | 30 ms | IVT-S energy counter (Wh, signed) |
| `0x18EFFF21` | on event | CAN keypad button press / release |
| `0xA100100` | on request | SIM100MOD isolation state / measurements |
| `0x19A` | — | OpenInverter LDU status *(TODO: confirm ID from inverter `can tx` output)* |
| `0x55A` | — | OpenInverter LDU faults *(TODO: confirm ID)* |
| `0x18FF03{pump}` | 1 Hz | EMP WP29 inverter cooling pump Motor Status 1 (speed, temp, power, controller status) |
| `0x18FF24{pump}` | 100 ms | EMP WP29 inverter cooling pump Motor Status 3 (voltage, current, HVIL status) |
| `0x600` | **Standard** | 100 ms | EVCC EVSE_Information (v2.5) — Stage (b0), Protocol (b1), Pins (b2: 1=CCS_AC, 2=CCS_AC_1PH, 3=CCS_AC_3PH, 4=CCS_DC_Core, 5=CCS_DC_Extended, 6=MCS), Max_Current (b3:4 signed A), RCD (b5.0); AC session started when Pins 1–3 and no session active |
| `0x601` | **Standard** | on event | EVCC AC_Control (v2.5) — `Ready_To_Deliver_Power` bit 0; latched as `acReadyToDeliver`; enables AC_Status Ready response |
| `0x68001` | on plug-in | EVCC New_Charge_Session — Communication_Protocol, Plug_and_pins (0=CCS_DC_Core, 1=CCS_DC_Extended, 2=CHAdeMO; any other value → AC), EV_Max_Voltage/Current, Battery_Capacity, SoC |
| `0x68002` | on demand | EVCC Insulation_Test — informational |
| `0x68003` | on demand | EVCC Precharge — informational |
| `0x68004` | on demand | EVCC Charge_Status_Change — Vehicle_Ready_for_Charging |
| `0x68005` | during charge | EVCC Charging_Loop — Target_Voltage, Target_Current, SoC |
| `0x68006` | on event | EVCC Emergency_Stop — Origin; clears EVCCsessionActive / EVCCsystemEnable |
| `0x68007` | on event | EVCC Charge_Session_Finished; clears EVCCsessionActive / EVCCsystemEnable |
| `0x68009` | 200 ms | EVCC Advantics_Controller_Status — State (heartbeat; absence implies EVCC fault) |
| `0x0CE982A4` | *(TODO)* | Chery OBC+DCDC — `DCDC_Telemetry`: HV input V, LV output V, LV output I, temperature (see below) |
| `0x103F34A4` | *(TODO)* | Chery OBC+DCDC — `OBC_Telemetry`: AC input V, HV output V, HV output I, status flags (see below) |

**Chery New Energy OBC+DCDC frames** — DBC supplied by vendor (`dbc/Chery_New_Energy_OBC_DCDC.dbc`); bus assignment (CAN2) and baud rate are provisional pending hardware arrival. All four messages are extended-ID, byte-aligned big-endian (Motorola) fields.

| Message | ID | Bytes | Field | Scale | Range | Notes |
|---------|-----|-------|-------|-------|-------|-------|
| `VCU_DCDC_Command` (TX) | `0x0CE97FC4` | 0 | `DCDC_Enable` | 1 | 0/1 | 0 = Disable_Standby, 1 = Enable |
| | | 1–2 | `DCDC_TargetVoltage` | 0.1 V/bit | 0–20 V | |
| `VCU_OBC_Command` (TX) | `0x104CE8DC` | 0–1 | `OBC_TargetVoltage` | 0.1 V/bit | 0–500 V | |
| | | 2–3 | `OBC_TargetCurrent` | 0.1 A/bit | 0–50 A | |
| | | 4 | `OBC_ChargeControl` | 1 | 0/1 | 0 = Charge_ON_Run, 1 = Charge_OFF_Stop |
| `DCDC_Telemetry` (RX) | `0x0CE982A4` | 0–1 | `DCDC_HV_InputVoltage` | 0.1 V/bit | 0–500 V | |
| | | 2–3 | `DCDC_LV_OutputVoltage` | 0.1 V/bit | 0–20 V | |
| | | 4–5 | `DCDC_LV_OutputCurrent` | 0.1 A/bit | 0–250 A | |
| | | 6 | `DCDC_Temperature` | 1, offset −40 | −40–120 °C | |
| `OBC_Telemetry` (RX) | `0x103F34A4` | 0–1 | `OBC_AC_InputVoltage` | 1 V/bit | 0–300 V | |
| | | 2–3 | `OBC_HV_OutputVoltage` | 0.1 V/bit | 0–500 V | |
| | | 4–5 | `OBC_HV_OutputCurrent` | 0.1 A/bit | 0–50 A | |
| | | 6 | `OBC_StatusFlags` | 1 | 0–255 | bitfield, meanings undefined in vendor DBC |

---

### CAN3 — 1 Mbps

Devices: Wireless gateway (Arduino Portenta H7 + Quectel 4G module — handles SMS/cellular), RealDash

**Transmitted (VCU → RealDash / gateway)**

| ID | Description |
|----|-------------|
| `0xC79` | SMS command to wireless gateway (byte 0 = message code, see below) |
| `0xC80` | RPM, power, temperature, throttle (every 62.5 ms) |
| `0xC81` | Pack voltage, pack current, 12 V battery voltage |
| `0xC82` | Highest/lowest cell voltage, ground speed, GPS altitude |
| `0xC83` | Cell delta voltage, SIM100MOD isolation, SIM100MOD temperature, GPS fix type |
| `0xC85` | Status response to gateway status request (see below) |

**RealDash-over-Ethernet feed** — the same `0xC80`-`0xC83` data is also mirrored over a direct
Ethernet link to the Odroid M2 display (`realdash_tcp.cpp`), so RealDash can consume it directly
over TCP instead of needing a physical CAN-to-USB adapter. VCU runs a TCP server on
`192.168.10.10:35000` (Odroid at `192.168.10.1`, direct point-to-point cable, static IPs, no
DHCP); RealDash connects as the client and streams RealDash's native "44" frame format (4-byte
tag `0x44,0x33,0x22,0x11` + 4-byte little-endian CAN ID + 8-byte payload). CAN definition file
for RealDash to import: `dbc/realdash_vcu.xml`. Skipped on CAN-wake (`KL15C`) boots along with
GNSS, to keep that path's response time unaffected. Confirmed on hardware; `groundSpeed`
(mph) and `GPSaltitude` (ft) are converted from the GNSS module's raw mm/s and mm units in
`printPVTdata()` (`init.cpp`) before being packed into `0xC82`. RPM, power, motor/pack temp,
pack voltage/current, and 12 V battery in `displayStatus()` are still placeholder/test values,
not real sensor data — TODO.

**`0xC79` SMS message codes** (byte 0, 1 byte total) — the gateway sends a canned text for each code:

| Code | Message | Sent from |
|------|---------|-----------|
| 0 | "KL15R on" | `Idle_enter()` |
| 1 | "KL15C on" | *(not yet called)* |
| 2 | "Pre-charge failed..." | `Idle_enter()` — **note:** fires unconditionally on every Idle entry, which only happens after a successful pre-charge (`PRECHARGE_OK`); likely a placeholder rather than intentional, worth revisiting |
| 3 | "Something happened..." | `Fault_enter()` |
| 4 | "Charging stopped..." | *(not yet called)* |
| 5 | "Temperature warning..." | *(not yet called)* |
| other | "Invalid request..." | — |

**Wake-on-CAN3 status request/response** — the gateway can wake the VCU from sleep to ask for
current status. CAN3's RX pin (pin 30) is armed as a third `enterSleep()` wake source (alongside
KL15R and CAN2), the same way EVCC/CAN2 activity already wakes the VCU — the CAN controller is
clock-gated during sleep, so only a raw GPIO edge is detected; the actual request frame is not
decoded until after the wake-and-reboot cycle completes. **The gateway retries the request every
500 ms for up to 20 s** until it gets a response, since the frame that triggers the wake is itself
lost.

Ideally the VCU recognises this as a CAN wake (via `SNVS_LPGPR0`, see below) and lands directly in
`KL15C` standby, which skips GNSS bring-up entirely (`extWakePending` gates the GNSS block in
`setup()` — see `gnssInitialized`) for a much faster turnaround, and only re-sleeps after 60 s of
CAN inactivity. **Known issue:** this detection currently sometimes fails even with `KL15R`
confirmed low, landing in plain `Off` instead (full GNSS/ADS1115 reinit, slower) — root cause not
yet found; a debounce attempt around the post-`wfi` `KL15R` read did not fix it and briefly hung
the board, so it was reverted (see `feedback_teensy41_noinit` memory / git history for the wfi
timing risk). The behaviour below is written to work correctly regardless of which state it
lands in, so this is a performance issue, not a functional one.

The response is held until `highestCellV` reflects a real pMBB32 reading rather than the
zero-initialized post-reset default — every wake is a full reset, so cell voltage data isn't valid
until the modules have answered at least one `0xFF0000` measurement trigger. `callback_t2()`
re-checks every 200 ms. While a request is pending, the KLR auto-sleep debounce in `loop()` is
held off entirely (`gatewayStatusRequestPending`) so the VCU can't re-sleep before answering, no
matter how long pMBB32 takes; once the response is sent, `loop()` sleeps immediately
(`gatewayResponseSent`) rather than waiting out the normal 500 ms debounce.

| ID | Description |
|----|-------------|
| `0xC84` | Status request from wireless gateway (any payload; sets a pending flag, answered from `callback_t2()`) |

**`0xC85` status response** (8 bytes, sent from `callback_t2()` when a `0xC84` request is pending):

| Bytes | Field | Scale | Notes |
|-------|-------|-------|-------|
| 0-1 | Pack voltage | 0.1 V/bit | from `IVTpackVoltage` |
| 2-3 | Highest cell voltage | raw ADC counts | `highestCellV` |
| 4-5 | Lowest cell voltage | raw ADC counts | `lowestCellV` |
| 6 | VCU state | `VCUStateEnum` value | |
| 7 | Reserved | — | 0 |

**Received**

| ID | Description |
|----|-------------|
| `0x1F4` | Received but not yet handled |

---

## LIN Buses — 19200 baud

| Device | Bus | Direction | Node ID | Notes |
|--------|-----|-----------|---------|-------|
| BMW i4/i5/i7/iX Changeover Valve 64119462114 (#1) | Serial6 (TX6=pin 24, RX6=pin 25) | Slave response | `0x10` *(TBD)* | Byte map TBD from BMW ISTA docs; implemented in `lin.cpp` |
| BMW i4/i5/i7/iX Changeover Valve 64119462114 (#2) | Serial7 (TX7=pin 29, RX7=pin 28) *(planned)* | Slave response | *(TBD)* | Not yet wired or implemented |

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
| 30 | CAN3 RXD wake input (`CAN3_RX_PIN`) — same mechanism as pin 0, wakes VCU from STOP mode for wireless gateway status requests |
| 2 | KL15R input (`KL15R_PIN`) — key position 1, wakes hardware and is sampled in `enterSleep()` to distinguish CAN-wake from key-on wake |
| 3 | Loop timing debug output |
| 4 | CAN1 RX timing debug output |
| 5 | CAN2 RX timing debug output |
| 6 | `displayStatus()` timing debug output |
| 13 | Built-in LED (1 Hz heartbeat) |
| 18 (SDA) | GNSS I2C data — Wire / I2C0 |
| 19 (SCL) | GNSS I2C clock — Wire / I2C0 |
| 24 (TX6) | LIN bus 1 TX |
| 25 (RX6) | LIN bus 1 RX |
| 28 (RX7) | LIN bus 2 RX *(planned — second BMW changeover valve)* |
| 29 (TX7) | LIN bus 2 TX |
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

Turning the key off (KLR low) while in the Off state triggers `enterSleep()` after a 500 ms debounce (KLR must be continuously low for 500 ms). The debounce gives the EVCC time to send `New_Charge_Session` (0x68001) after a CAN2 wake before sleep is re-entered. `enterSleep()` returns immediately if `EVCCsessionActive` is true (active charge session), keeping the VCU awake without KLR.

1. All four timers stop; heartbeat LED is forced off.
2. USB PHY is powered down (`USBPHY1_PWD = 0xFFFFFFFF`) and its CCM clock gated.
3. GNSS is put into backup mode via `UBX-RXM-PMREQ` (`powerOffWithInterrupt`, EXTINT0 wake source, ~15 µA).
4. FlexCAN1/2/3 peripheral clocks are gated off via `CCM_CCGR0` / `CCM_CCGR7` — stops internal CAN controller sampling.
5. `CAN_STBY_PIN` (pin 32) is driven HIGH — all three CAN transceivers enter standby mode.
6. CPU clock is reduced to ~16.2 MHz (ARM PLL minimum; DCDC core voltage drops to 0.95 V), then AHB is switched to the 24 MHz crystal, ARM PLL bypass is enabled (CPU runs at crystal / ARM_PODF ≈ 3 MHz), and the ARM PLL VCO is powered down.
7. CAN2 and CAN3 RXD pins (pins 0 and 30) are reconfigured as GPIO inputs with pull-up. Rising-edge interrupt on `KLR_PIN` and falling-edge interrupts on `CAN2_RX_PIN`/`CAN3_RX_PIN` are attached as three wake sources; SysTick is disabled. The MCP2562 transceivers drive RXD low on dominant bus edges even in standby, so EVCC (CAN2) or wireless gateway (CAN3) traffic wakes the VCU without KLR.
8. `CCM_CLPCR[LPM]` is set to STOP (0b10) and `SCB_SCR[SLEEPDEEP]` is set — a single `wfi` then enters IMXRT1062 STOP mode, gating internal power domains beyond what WAIT mode achieves.

On wake, a rising edge is asserted on pin 33 (EXTINT0) via DWT cycle-counter delay to start the GNSS hot-start before the Teensy resets; `SCB_AIRCR` resets the chip so `setup()` re-initialises all peripherals (including clock restoration) cleanly.

**Measured sleep current: ~4 mA at 12 V** (external 90–95 % efficient 12 V → 5 V switcher + Teensy onboard 3.3 V LDO). Down from ~61 mA before sleep optimisations — a 93 % reduction.

### On State (KL15 active)

Pressing keypad button 5 (KL15) fires `KL15_ON` and initiates the drive-enable sequence. The entire On State is exited by pressing the **Park** button while the vehicle is stationary (`LDUrpm == 0`), which fires `KL15_OFF` and disables the contactors.

```
  [KLR / Off] ──── KL15_ON (btn 5) ────> [PreCharge]
       ^ ^                                    │ IVT U2 ≥ 95% U1 within 2 s
       │ │ KL15_OFF                           ├──────────────────────> [Idle]
       │ │ (Park btn, speed = 0)              │ EVCC chargeMode = true
       │ │ or FAULT_CLEAR                     ├──────────────────────> [Charge]
       │ │ (Park btn, speed = 0)              │ timeout or FAULT_EV
       │ │                                    └──────────────────────> [Fault]
       │ │
       │ │        DRIVE_ON (btn 8)      DRIVE_OFF (btn 1, speed = 0)
       │ │        [Idle] ──────────────> [Drive] ──────────────────> [Idle]
       │ │
       │ │        TEMP_LOW / TEMP_HIGH                    TEMP_OK
       │ │        [Idle/Drive/Charge] ──────> [HeatPack / CoolPack] ──> [Idle]
       │ │
       │ └── KL15_OFF (Park btn, speed = 0) from any On state ──> [Off]
       │
       │  EXT_WAKE (CAN2 wake, KL15R low)
       └──────────────────────────────────── [KL15C] ─── KL15_ON ──> [Off]
                                                │   └── AC_CHARGE_START ─> [PreCharge]
                                                └── 60 s inactivity ──> sleep
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
| KL15C | Minimal CAN-wake standby (no HV); EVCC heartbeat monitored; KL15R LED edge-detected | KL15 pressed → `KL15_ON` (→ Off); AC plug-in → `AC_CHARGE_START` (→ PreCharge); 60 s inactivity → `enterSleep()` |

`reducedPowerActive` flag (set by BMS temp fault during Drive/Charge) clamps throttle to `THROTTLE_FAULT_LIMIT` without leaving Drive state.

### KL15C and the wake-cause mechanism

When the EVCC (CAN2) or wireless gateway (CAN3) wakes the VCU with the key out, `enterSleep()` needs to communicate that fact to the `setup()` that runs after the subsequent AIRCR reset — there is no call stack or return address through a software reset.

The wake cause is carried in `SNVS_LPGPR0`, one of the IMXRT1062's battery-backed low-power general-purpose registers — purpose-built for exactly this (surviving resets and power-down), unlike plain SRAM.

**In `enterSleep()`, just before the AIRCR reset:**
```cpp
SNVS_LPGPR0 = digitalRead(KL15R_PIN) ? 0 : SLEEP_MAGIC_CAN_WAKE;
```
- KL15R HIGH (key was turned while the VCU slept) → `SNVS_LPGPR0 = 0` → normal boot, Off state.
- KL15R still LOW (CAN2/CAN3 woke us, key never moved) → `SNVS_LPGPR0 = SLEEP_MAGIC_CAN_WAKE` → KL15C.

**In `setup()`:**
```cpp
if (SNVS_LPGPR0 == SLEEP_MAGIC_CAN_WAKE && !digitalRead(KL15R_PIN))
    extWakePending = true;
SNVS_LPGPR0 = 0;  // clear immediately — cold-boot after this point must not misfire
```
If `extWakePending` is set, `fsm.trigger(EXT_WAKE)` fires before the main loop, placing the VCU in KL15C instead of Off. The same flag also gates GNSS bring-up in `setup()` — skipped entirely on a CAN wake, since it isn't needed to answer a CAN3 gateway status query and its retry loop is otherwise uncapped (see the CAN3 section above for the known issue where this detection doesn't always fire correctly).

**Why not a DMAMEM/OCRAM variable?** That was the first approach (`DMAMEM volatile uint32_t sleepMagic`, relying on OCRAM surviving an AIRCR software reset — ordinary SRAM isn't cleared by that kind of reset, and the CRT startup's BSS-zeroing loop only covers `.bss` in DTCM, not `.bss.dma` in OCRAM). It compiled and ran without faulting, but on real hardware the value reliably came back as **the same garbage pattern** after a real STOP-mode sleep cycle — reproducible on both a plain KL15R wake and a CAN-triggered wake, so it wasn't a CAN-specific bug. Most likely cause: this project's STOP-mode config drops the CPU to ~3 MHz off the crystal and the DCDC core rail to 0.95 V, which is apparently below OCRAM's safe retention threshold. `SNVS_LPGPR0` sidesteps the question entirely by using a register domain that isn't subject to core RAM power-down at all.

(Earlier still: `.noinit`, the conventional section name for this pattern on AVR/STM32, doesn't exist in the Teensyduino linker script for this chip — GNU ld orphans the symbol into the external-PSRAM ERAM region, which hardfaults immediately on a board without PSRAM fitted.)

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
| t1 | RTC | 62.5 ms | PDU-8 driver settings; pMBB32 min/max cell poll (round-robin); RealDash CAN3 update; EVCC Power_Modules_Status / Limits / Sequence_Control (0x60010 / 0x60011 / 0x60012); EVCC AC_Status (0x611, standard-ID) |
| t2 | GPT1 | 200 ms | Send `0xFF0000` measurement trigger; invalidate stale module data; stale module recovery (wake + contReportingEnable); SIM100MOD isolation poll; LIN valve poll; inverter pump command (CAN2); battery pump command (CAN1) |
| t3 | GPT2 | 1000 ms | Heartbeat LED toggle |
| main loop | — | free-running | CAN event dispatch; GNSS processing; FSM step |

t0 runs at the highest ARM Cortex-M7 NVIC priority (`priority(0)`) and preempts all other work.

---

## Source Layout

`main.cpp` started as a single 2200+ line file and has been split incrementally, one function/module at a time, into `src/`:

| File | Contents |
|------|----------|
| `main.cpp` | Global objects (timers, CAN message structs, FSM `State` objects), `loop()`, and the small pMBB32 helpers (`wakepMBB32`, `shutdownpMBB32`, `ReadDigitalStatuses`, `ReadAnalogStatuses`) |
| `init.cpp` | `setup()` and its GNSS `printPVTdata` auto-PVT callback |
| `callbacks.cpp` | `callback_t0` / `t1` / `t2` / `t3` — the periodic timer callbacks (see [Periodic Tasks](#periodic-tasks)) |
| `can_handlers.cpp` | `can1` / `can2` / `can3` `FlexCAN_T4` objects, `can1Sniff` / `can2Sniff` / `can3Sniff`, `initCAN()` |
| `fsm_states.cpp` | FSM state enter/exit/check callbacks and `on_trans_*` transition callbacks |
| `sleep.cpp` | `enterSleep()` — STOP-mode sleep sequence (see [KLR region](#klr-region-key-position-1)) |
| `display.cpp` | `displayStatus()` — RealDash CAN3 update, also mirrors frames to the Ethernet link via `realdash_tcp.cpp` |
| `realdash_tcp.cpp` | RealDash-over-Ethernet TCP server (`realdashInit`, `realdashService`, `realdashQueueFrame`) — see CAN3 section |
| `sdlog.cpp` | SD card logging (`sdInit`, `sdLogData`, `sdQueueEventISR`, `sdDrainEvents`) |
| `lin.cpp` | LIN valve I/O (`linInit`, `linReadValve`, `linWriteValve`) |
| `throttle.cpp` | `readThrottle()` — the throttle pipeline (see [Throttle Pipeline](#throttle-pipeline)) |
| `globals.cpp` | Remaining global variable definitions with initializers |
| `pMBB32.h` | pMBB32 protocol constants (`wakeup`, `channelCount16`, `numberOfDevices`, frame IDs) |

`include/defines.h` holds macros, types, and `extern` declarations for everything shared across these files; each split-out module also gets its own small prototype header (`callbacks.h`, `can_handlers.h`, etc.) included from wherever it's called.

**Gotcha for future splits:** `FlexCAN_T4`'s constructor and `.begin()` both touch file-scope `static` routing pointers with internal linkage — if a `can1`/`can2`/`can3` object is *constructed* in one `.cpp` file but `.begin()` is *called* from another, the real interrupt vector ends up pointing at a trampoline that reads an unset pointer, and the board hangs on first CAN traffic (indistinguishable from a hardware fault). Keep construction and `.begin()`/`.onReceive()`/`.enableFIFOInterrupt()` calls for a given CAN object in the same translation unit — currently `can_handlers.cpp`.

---

## Building

Requires [PlatformIO](https://platformio.org/). Open the project folder in VS Code and click **Build (✓)** in the PlatformIO toolbar.

### Environments

| Environment | Command | Purpose |
|-------------|---------|---------|
| `teensy41` (default) | `pio run` | Main VCU firmware |

### Dependencies (auto-installed by PlatformIO)

| Library | Purpose |
|---------|---------|
| `luni64/TeensyTimerTool` | Periodic timer callbacks (RTC, GPT1, GPT2) |
| `sparkfun/SparkFun u-blox GNSS Arduino Library` | GNSS / GPS |
| `jonblack/arduino-fsm` | Finite state machine |
| `gicking/LIN master portable` | LIN master on Serial6 |
| `adafruit/Adafruit ADS1X15` | ADS1115 16-bit ADC driver |

---

## Placeholders / TODOs

- **OpenInverter LDU v5** — run `can tx` in inverter terminal to confirm actual RX CAN IDs for status/fault frames; wire up `can2Sniff()` cases for `LDUrpm`, `LDUtorque`, `LDUmotorTemp`, etc.
- **OpenInverter CRC** — implement `crc_calculate_block` equivalent and set `controlcheck 1` on inverter once formula is confirmed from stm32-sine source
- **Brake calibration** — bench-calibrate `BRAKE_THRESHOLD` (ADS1115 counts) against actual sensor output
- **Throttle calibration** — bench-calibrate `THROTTLE_POT1/2_MIN/MAX` (ADS1115 counts; current values are ×8 approximations of old 12-bit readings)
- **EVCC calibration** — set `EVCC_CELL_V_EMPTY` / `EVCC_CELL_V_FULL` (pMBB32 raw counts) for actual cell chemistry; set `EVCC_MAX_VOLTAGE_x10` and `EVCC_MAX_CURRENT_x10` for pack charge limits
- **EVCC AC charging** — AC session detection and EVCC handshake implemented: `New_Charge_Session` (0x68001) `Plug_and_pins` ≥ 3 or `EVSE_Information` (0x600) Pins 1–3 → `evccIsACSession`; `AC_Control` (0x601) → `acReadyToDeliver`; `AC_Status` (0x611) `Ready_To_Charge` sent every 62.5 ms; VCU closes main contactors via KL15C → PreCharge → Charge. Still pending: confirm actual Pins value sent by EVCC on AC plug-in by CAN sniff; split `VCU_STATE_CHARGE` into `VCU_STATE_DCFC` and `VCU_STATE_AC_CHARGE`; implement `CHARGE_OFF` transition from EVCC session-end event
- **Chery New Energy OBC+DCDC** — hardware ordered, not yet fitted; protocol decoded from vendor DBC (see [CAN2](#can2--500-kbps)) but not yet wired into `can2Sniff()` / `callback_t2()`. Once fitted: confirm actual bus/baud rate, send `VCU_DCDC_Command` (always-on `DCDC_Enable`, target 13.8–14.4 V) and `VCU_OBC_Command` (forward target V/I and start/stop from EVCC `AC_Control`/`Charging_Loop`), decode `DCDC_Telemetry` / `OBC_Telemetry` into new globals, replaces the earlier Elcon UHF-CAN-312 OBC placeholder
- **EMP WP29 pumps** — confirm pump J1939 source address (`EMP_WP29_ADDR`, currently `0x8A`) matches both pumps via CAN sniffer; remove CH3 passive pre-charge relay command from `PreCharge_enter()` once active pre-charge board is fitted
- **BMW LIN valve** — confirm LIN node address (`LIN_VALVE_ID`) and frame spec from BMW ISTA docs; assign `LIN_EN_PIN`; wire and implement the second valve's LIN bus on Serial7 (TX7=29, RX7=28)
- **Pre-charge / contactor sequencing** — Idle state entry currently has a fixed 5 s delay; implement voltage-based pre-charge completion check using IVT-S U2 (pre-charge voltage)
- **Regen braking** — implement `pot2` / `regenpreset` fields in the LDU frame; wire to brake pressure or paddle
- **pMBB32 individual cell voltages** — broadcast frames (ft=01..0C) received but not decoded in `can1Sniff()`; only min/max summary (ft=0E) is currently used
- **Cell balancing** — pMBB32 modules perform autonomous balancing when `maxCellDelta > 5 mV`; add VCU-directed balancing commands (`0x00DFxx00`) during charging near top of charge for tighter cell matching
- **IVT fault detection** — populate `IVTfaultActive` in `can2Sniff()` when pack current or voltage is out of safe range
