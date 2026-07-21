# Development Handoff

Snapshot of where this VCU firmware stands, for picking up development on another machine.
See `README.md` for the full hardware/protocol reference — this file is about *process*: what
just happened, what's confirmed working, and what's still open.

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
