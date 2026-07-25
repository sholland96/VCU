# Development Handoff

Snapshot of where this VCU firmware stands, for picking up development on another machine.
See `README.md` for the full hardware/protocol reference — this file is about *process*: what
just happened, what's confirmed working, and what's still open.

## RealDash-over-Ethernet feed — confirmed working on hardware

Drafted overnight (autonomous, with the user's explicit go-ahead), then flashed and verified
together the next day. Frame data confirmed flowing correctly over TCP (captured raw bytes on
the Odroid side — tag, CAN IDs, and payloads all check out, e.g. `44 33 22 11 80 0c 00 00 ...`
= tag + `0xC80` little-endian). RealDash itself is now configured and confirmed working:
adapter type is **Adapters (CAN/LIN) -> RealDash CAN -> WiFi**, IP `192.168.10.10` port `35000`
(NOT SocketCAN — that's for a real kernel `can0` device, not a WiFi/TCP feed like this one), CAN
description file `dbc/realdash_vcu.xml` imported via RealDash's file browser (needed `yad`
installed on the Odroid — RealDash's Qt file dialog has no fallback without one of
kdialog/yad/Xdialog on a bare Openbox setup). RealDash's own "configuration frame" (CAN
speed/mode, sent RealDash -> device) is intentionally left disabled — irrelevant here since this
device doesn't read anything back from the socket. Standard `targetId`-based fields (RPM,
throttle/TPS) show up on default layouts automatically; the custom `name="VCU: ..."` fields need
gauges manually added under RealDash's "ECU Specific" input category to be visible at all.

**Bug found and fixed during bring-up:** the first version called `realdashSendFrame()` (a
direct TCP write) straight from `displayStatus()`, which runs inside `callback_t1()` — a
hardware-timer ISR, same category as `callback_t2()` (where SD-card I/O was already known to be
ISR-unsafe, see the CAN3 gateway-wake bug list above). QNEthernet's TCP stack is likewise not
ISR-safe: the connection worked fine (TCP handshake completed, `nc -zv` succeeded) but zero
bytes ever actually arrived — writes were silently no-oping from ISR context, no crash, no
error. Fixed the same way this codebase already fixes this class of bug: `displayStatus()` now
calls `realdashQueueFrame()` (ISR-safe, only buffers 4×8 bytes into a fixed array), and the
actual `EthernetClient::write()` calls happen in `realdashService()` from `loop()` instead. If
another ISR-context feature is added here later, don't repeat this — anything touching
QNEthernet's client/server objects must run from `loop()`, not a `TeensyTimerTool` callback.

**Goal:** feed RealDash on the Odroid M2 display over the Teensy-Odroid Ethernet link (the same
direct point-to-point cable brought up and confirmed working earlier this session with the
`ethernet_test` sketch), instead of requiring a physical CAN-to-USB adapter. This is additive —
the existing physical CAN3 feed to RealDash/the wireless gateway is untouched.

**Protocol (researched, not guessed):** RealDash's native network transport is a TCP server on
the device (RealDash connects as the client) on port 35000, streaming "44" frames: 4-byte tag
`0x44,0x33,0x22,0x11` + 4-byte little-endian CAN ID + up to 8 payload bytes, no CRC. Notably,
`src/globals.cpp` already had `uint8_t serialBlockTag[] = {0x44,0x33,0x22,0x11};` sitting
unused — this is exactly RealDash's tag. Confirmed with the user: this dates back to an earlier
attempt at RealDash-over-Ethernet, back when the Odroid M2 ran Android and Ethernet couldn't be
gotten working there at all. The Odroid runs Armbian now (switched earlier this session during
the display/touchscreen bring-up), which is presumably what actually unblocked it this time.
Reused the dormant constant.

**What was added:**
- `include/realdash_tcp.h` / `src/realdash_tcp.cpp` — `EthernetServer` on port 35000 (QNEthernet),
  single-client (a new connection replaces the old one). `realdashQueueFrame(id, payload, len)`
  is the public, ISR-safe entry point — see the bug note above for why the send itself had to
  move to `realdashService()`.
- `src/display.cpp` — one `realdashQueueFrame(...)` call added after each of the four existing
  `can3.write(msg3)` calls in `displayStatus()`, reusing the exact same `msg3.buf` already built
  for CAN3. No change to what's computed, only where it's also sent.
- `src/init.cpp` — `realdashInit()` (static IP `192.168.10.10`/`.1`, matches the proven
  `ethernet_test` addresses) called from `setup()`, guarded by `if (!extWakePending)` — same
  reasoning as the existing GNSS guard: skip anything non-essential on the CAN-wake fast-response
  path so it can't add latency or risk a stall there.
- `src/main.cpp` — `realdashService()` added to `loop()` (accepts new/replaced clients, flushes
  queued frames to the socket).
- `platformio.ini` — added `ssilverman/QNEthernet` to the main `teensy41` env's `lib_deps`
  (previously only in the standalone `ethernet_test` env).
- `dbc/realdash_vcu.xml` — RealDash CAN-definition file to import on the Odroid, mapping all
  four frames' bytes to gauges/custom ECU-specific inputs. Researched RealDash's actual XML
  schema (janimm/RealDash-extras on GitHub) rather than guessing the format.

**Flashed and confirmed on hardware.** Frame bytes captured on the Odroid side via `nc`/`od`
matched the protocol exactly. Nothing committed yet.

**Bug found and fixed after the initial bring-up: GPS altitude/speed truncation.** Once RealDash
was actually configured and gauges added, altitude showed ~16,000-17,000 ft (obviously wrong).
Root cause was two stacked bugs in a code path unrelated to this feature's own changes:
`printPVTdata()` in `init.cpp` (the GNSS PVT callback) set `GPSaltitude` directly from `hMSL`,
which is **millimeters**, not feet — and `display.cpp` only ever sent the low 16 bits of that
32-bit value, so real altitude above ~215 ft wrapped around. Ground speed had the same class of
bug, worse: `groundSpeed` was set directly from `gSpeed` (mm/s, not mph), and `display.cpp`
hardcoded the high byte to `0`, truncating it to a single byte (overflows past ~0.6 mph). Fixed
both at the source in `printPVTdata()` (mm→ft, mm/s→mph, same conversion factors the codebase
already had sitting in dead/commented-out lines) and fixed the ground-speed packing bug in
`display.cpp`. Flashed and confirmed — altitude now reads correctly.

**Remaining known gap, NOT fixed, pre-dates this feature:** `displayStatus()` still sends mostly
placeholder/test data for everything else — `rpm` is a free-running test ramp (`rpm += 100`),
`IVTpackVoltage`/`batteryVoltage` are hardcoded constants (3840 / 1255), pack current is derived
from a test power counter, the motor/pack temp byte is a hardcoded `21`, throttle is hardcoded to
`100`. RealDash will show fake/frozen numbers for those fields until `displayStatus()` is wired
to the real `IVTpackVoltage`/`IVTpackCurrent`/etc. variables that already exist and are already
populated correctly elsewhere in the codebase. Cell voltages, ground speed, and GPS altitude/fix
type are the exceptions — those are real, live data now.

**Still open — not yet done:**
1. Confirm a plain KL15R (key-on) boot still reaches `RealDash: TCP server listening...` at the
   same point GNSS normally comes up (i.e. this didn't silently break the existing
   GNSS-skip-on-CAN-wake timing) — not explicitly checked, only inferred from normal-looking
   `pMBB32`/GNSS heartbeat prints after flashing.
2. Confirm a CAN3 gateway wake (`0xC84`/`0xC85`) still behaves exactly as before —
   `realdashInit()` is skipped on that path, so it shouldn't be affected, but worth confirming
   given the CAN-wake path's history of subtle regressions this session.

## What just happened (this session)

1. **Incremental `main.cpp` split** — the original 2226-line monolith is now split into
   `main.cpp`, `init.cpp`, `callbacks.cpp`, `can_handlers.cpp`, `fsm_states.cpp`, `sleep.cpp`,
   `display.cpp`, `sdlog.cpp`, `lin.cpp`, `throttle.cpp`, `globals.cpp` — see README's
   "Source Layout" section. Along the way, fixed a genuine hardware bug: the RTC `PI_FREQ`
   formula was inverted (`period = 2^PI_FREQ`, not `2^(15-PI_FREQ)`), which had been flooding
   CAN1/CAN2 at 128x normal rate and was misdiagnosed for a while as a keypad hardware fault.
2. **SIM7080G cellular modem** — wired directly to the Teensy (Serial7, pins 28/29, PWRKEY 27),
   brought up over AT commands, but never registered on the network (signal permanently
   undetectable, likely a power-supply brownout during transmit bursts). **Abandoned** — all
   direct wiring removed from this repo. SMS/cellular is now the job of a separate **Portenta H7
   + Quectel 4G module**, acting as the "wireless gateway" on CAN3.
3. **LIN buses reassigned** — bus 1 (existing BMW valve) moved from Serial3 to Serial6
   (pins 24/25); bus 2 (second valve, not yet wired) planned on Serial7 (pins 28/29), freed up
   by dropping the SIM7080G.
4. **CAN3 wake-on-gateway-status feature** (`0xC84` request / `0xC85` response) — the big one
   this session. See README's CAN3 section for the protocol. Built and debugged end-to-end on
   real hardware; several real bugs found and fixed along the way (see below).
5. **`KL30C` renamed to `KL15C`** — pure naming fix, no functional change (`KL30` is the
   always-hot rail, not a switch position; the standby state itself is CAN-wake-triggered, which
   maps better to `KL15`-adjacent naming). A broader terminology cleanup (giving `KL15R` its own
   distinct state, separate from the current `Idle`) was discussed but **not implemented** —
   flagged as a possible follow-up, not started.

## Bugs found and fixed (CAN3 wake feature bring-up)

All confirmed fixed on real hardware except where noted:

- **`klrLowSince` debounce bug** — was a function-local `static uint32_t = 0` in `main.cpp`'s
  `loop()`, zero-initialized once at first boot. Since `setup()` takes real time, `millis() - 0`
  was already `> 500ms` by the first post-boot `loop()` call, causing near-instant re-sleep with
  no grace period whenever a wake landed in `Off` state. Fixed: promoted to a global, reset in
  `Off_enter()`.
- **ISR debug-print overload** — a `Serial.printf()` in `can3Sniff()`'s `default:` case (logging
  every unhandled CAN3 frame) ran from interrupt context and made the board unresponsive under
  real traffic. Removed.
- **`sleepMagic` (wake-cause flag) unreliable in DMAMEM/OCRAM** — the original implementation
  (`DMAMEM volatile uint32_t sleepMagic`) came back as a consistent garbage value after a real
  STOP-mode sleep cycle, reproducible on both CAN-wake and plain KL15R wake — not random noise,
  not CAN-specific. Believed cause: this project's STOP-mode config (CPU at ~3 MHz, DCDC core at
  0.95 V) is below OCRAM's safe retention threshold. **Fixed** by switching to `SNVS_LPGPR0`, a
  battery-backed SNVS register built for exactly this. See `feedback_teensy41_noinit` memory.
- **Stale/zero pMBB32 data in the `0xC85` response** — every wake is a full reset, so
  `highestCellV`/`lowestCellV` start zero-initialized until the modules answer a `0xFF0000`
  trigger. Fixed: `callback_t2()` now holds the response until `highestCellV > 0`.
- **VCU could re-sleep before answering, or wait too long after answering** — two related fixes
  in `loop()`'s KLR debounce: won't sleep while `gatewayStatusRequestPending` is true (no matter
  how long pMBB32 takes), and sleeps immediately (skipping the 500 ms debounce) once
  `gatewayResponseSent` is set, rather than waiting on timing coincidence.
- **GNSS made conditional on wake cause** — `setup()`'s GNSS bring-up (an uncapped
  `while (myGNSS.begin() == false) delay(500);` retry loop) is now skipped entirely when
  `extWakePending` is true (CAN wake), gated by a new `gnssInitialized` flag that also guards the
  two GNSS touch-points in `enterSleep()`. Motivation was as much robustness as speed — if GNSS
  ever hung on a CAN-wake cycle, no gateway retry timeout could fix it, since `setup()` would
  never reach the response code at all.

## Still open / not fixed

- **Wake-cause detection (`SNVS_LPGPR0`/`extWakePending`) intermittently wrong.** Even with
  `KL15R` confirmed low via a fresh debug print, `SNVS_LPGPR0` sometimes reads back `0` (normal
  boot) instead of the CAN-wake magic value — meaning it lands in `Off` instead of `KL15C`.
  Consistently reads back as a *clean* `0`, not garbage (ruling out the old DMAMEM-style
  corruption), which suggests `digitalRead(KL15R_PIN)` itself is reading HIGH at the exact point
  `enterSleep()` decides the value — cause unknown. **A debounce fix around that read (sampling
  `KL15R_PIN` twice via the DWT cycle counter) was attempted and reverted** — it caused the board
  to stop waking entirely (most likely a `while (ARM_DWT_CYCCNT - ...)` loop that never
  terminated, since it's unconfirmed whether the DWT counter is reliably running immediately
  after a STOP-mode `wfi` returns, at that point in the function). **Do not re-attempt a busy-wait
  fix here without a safer verification method** — this cost a real hang on physical hardware.
  Functionally this is now a performance-only issue, not a correctness one: the flag-based fixes
  above (`gatewayStatusRequestPending`/`gatewayResponseSent`) make the feature work correctly
  regardless of which state it lands in, just slower via `Off` (full GNSS/ADS1115 reinit) than it
  would be via `KL15C`.
- **Broader FSM state naming** — user described wanting `KL15R` (key position 1) to map to its
  own distinct state (BMS enabled, display updated), separate from the current `Idle` (which
  currently means "post-pre-charge, ready for Drive"). Not implemented — only the `KL30C`→`KL15C`
  rename was done this session.
- **SMS code table inconsistency** — `Idle_enter()` sends SMS code `2` ("Pre-charge failed...")
  unconditionally on every entry into `Idle`, which only happens after a *successful* pre-charge.
  Flagged as likely unintentional (README's SMS code table notes this), not fixed — user was
  mid-edit on the SMS labels when this was found and didn't confirm the intended fix.

## Gateway-side protocol (separate project)

The Portenta H7 + Quectel 4G gateway needs to implement the `0xC84`/`0xC85` retry protocol
described in README's CAN3 section (500 ms retry, 20 s window). A standalone protocol doc was
handed to that project's Claude session directly (not saved in this repo).
