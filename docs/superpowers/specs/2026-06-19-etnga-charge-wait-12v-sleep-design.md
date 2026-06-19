# e-TNGA CHARGE_WAIT 12V-drain fix — tiered slow-poll + sleep

**Date:** 2026-06-19
**Component:** `vehicle/OVMS.V3/components/vehicle_toyota_etnga/`
**Branch:** `feature/etnga-charge-wait-12v-sleep`
**Status:** Design approved, pending spec review

## Problem

On 2026-06-18 the module was plugged in at SOC 87% with a scheduled (delayed) AC charge.
The state machine entered `CHARGE_WAIT` and polled the Plug-In Control System ECU
continuously (~3 ISO-TP req/resp per second) for ~3.5 hours while waiting. This kept the OBD
gateway and PICS ECU awake and drained the vehicle 12V from 12.6V (ref) to 10.0V:

```
18:03  CHARGE_WAIT entered (scheduled wait, AC Op at Stop)
21:29  powermgmt: 12V battery critical: 11.0V (ref=12.6V) -> "shutdown in 30 minutes"
21:34  12V shutdown: 10.0V -> boot: Shutting down for DeepSleep
~06:10 reboot (12V recovered when the car woke); by 06:21 SOC=100%
```

The module **slept through the entire overnight scheduled charge** (SOC 87→100% happened
while in protective DeepSleep). The 18:02 charge session was never cleanly closed — it was
terminated by the reboot — and no charge report was generated.

This 12V critical/shutdown was the **only** one in ~36h of logs; every normal parked
SLEEP/AWAKE cycle held 12V fine. The drain is isolated to `CHARGE_WAIT`.

### Root cause

`CHARGE_WAIT` is the only long-lived poll state with **no idle backoff**. Unlike `AWAKE`
(which forces sleep after 300s of no activity), `HandleChargeWaitState()` only exits to
sleep on `!ms_v_env_awake`. But `env_awake` is set true by *any* CAN2 frame
(`IncomingFrameCan2` → `SetAwake(true)`), and our own poll **replies** keep generating
frames — so `env_awake` never goes stale and the sleep path never fires. The continuous
poll is self-sustaining.

The 1s cadence on `0x1684` (AC-op) / `0x1666` (HLC) / `0x10D1` (ctrl-mode) exists to "catch
AC/DC engage promptly," but a scheduled charge can be hours away, where 1s responsiveness
buys nothing and costs the 12V battery.

Note: the powermgmt **protective DeepSleep** (a hard ESP32 deep sleep needing 12V to
physically recover) is a different and worse beast than the firmware's **poll-state SLEEP**
(firmware keeps running, listening on CAN, wakes on frames). Had the module entered
poll-state SLEEP earlier, 12V would not have collapsed and the protective DeepSleep would
not have triggered.

## Goals / non-goals

**Goals**
- Stop `CHARGE_WAIT` from draining the 12V during long (e.g. scheduled-charge) waits.
- Reliably catch the scheduled charge actually starting (don't sleep through it).
- Preserve a charge session and its energy counters across a wait-induced sleep.

**Non-goals (YAGNI for v1)**
- Timer-backed periodic wake-check (passive bus-wake only; see Risk 2).
- New `[instance]` config params (hardcoded constants only, matching `SLEEP_COOLDOWN_SECS`).
- Any change to AC/DC/HANDSHAKE charging behavior beyond the WAIT cadence.

## Design — tiered wait

Files: `etnga_poll_states.cpp` (state handlers/transitions), `vehicle_toyota_etnga.cpp`
(poll table `obdii_polls_charge[]`), `vehicle_toyota_etnga.h` (members/constants).

### Tier 1 — slow the engage-watch (immediate, on entering CHARGE_WAIT)

In the `CHARGE_WAIT` column of `obdii_polls_charge[]` (the 2nd value of each `{HS,WAIT,AC,DC}`
tuple), slow the three fast engage-watch PIDs from 1s to **10s**:

| PID (on Plug-In Control System ECU) | WAIT now | WAIT new |
|---|---|---|
| `0x1684` AC charging op status | 1 | 10 |
| `0x1666` HLC state | 1 | 10 |
| `0x10D1` control-system mode | 1 | 10 |

All other WAIT polls are already sparse (`0x1669` PISW @30s, `0x1F46` @30s, `0x1688` @10s,
`0x1667` @30s) and are unchanged. This cuts WAIT CAN2 traffic ~10× and is the only behavior
that matters for **brief** waits (an AC pause/resume mid-session that never reaches Tier 2):
worst-case resume latency becomes ≤10s.

### Tier 2 — sleep after a sustained wait

Track elapsed time in `CHARGE_WAIT` without charge engaging. `TransitionToChargeWaitState()`
already stamps `m_charge_state_entry`. In `HandleChargeWaitState()`, after the existing
engage/cable checks, add a sleep threshold that is **short after a prior wait-sleep** and
long on first entry (so we give a fresh wait a responsive window but re-sleep quickly when
oscillating):

```
int sleep_after = m_charge_wait_slept ? CHARGE_WAIT_RESLEEP_SECS : CHARGE_WAIT_SLEEP_SECS;
if (monotonic - m_charge_state_entry >= sleep_after) {
    // Sustained wait (e.g. scheduled charge hours away) — stop polling so the bus
    // (and we) can idle down and the 12V recovers. Reuse the existing sleep machinery;
    // the charge session stays open and resumes on wake.
    ESP_LOGI(TAG, "CHARGE_WAIT idle %ds — sleeping to protect 12V", sleep_after);
    m_charge_wait_slept = true;
    TransitionToSleepState();
    return;
}
```

`m_charge_wait_slept` distinguishes a first wait (full `CHARGE_WAIT_SLEEP_SECS` responsive
window) from a re-entry after a wait-sleep (short `CHARGE_WAIT_RESLEEP_SECS`, so the
oscillation duty cycle stays low: poll ~`RESLEEP` seconds, sleep up to the cooldown cap).
It is **set** here and **cleared** only when the wait genuinely ends: when charge actually
engages (`TransitionToChargeAcState` / `TransitionToChargeDcState`) or when a new session
opens / the session closes (`TransitionToChargeHandshakeState` `!in_session` block /
`TransitionToAwakeState` session-close). It is **not** cleared on the SLEEP→AWAKE→WAIT
resume itself.

`TransitionToSleepState()` is reused unchanged: it stops all polling (SLEEP poll column is
all-zeros), arms the escalating cooldown, and seeds the 12V edge latch. The charge session
(`m_charge_session`, `in_session`) is **not** touched by the sleep transition, so it is
preserved.

### Resume — passive wake via existing machinery

No new wake path. When the scheduled charge starts, the OBC powers up → CAN2 frames (and/or
the DC-DC raises 12V → the existing rising-edge wake in `HandleSleepState`) → wake → `AWAKE`:

- **Cable still seated** (`PISW >= 0x02`): `HandleAwakeState` re-enters the charge
  sub-machine. `in_session` is still true, so `TransitionToChargeHandshakeState`'s
  `if (!in_session)` open-guard prevents re-init — the session and energy counters
  (`ms_v_bat_energy_*`, `ms_v_charge_kwh*`, `m_v_env_hvac_kwh`) are preserved. State
  resolves to AC/DC as the real signals arrive.
- **Cable removed during sleep** (fresh `PISW == 0x00`): the existing wake-reconcile in
  `HandleAwakeState` finalizes the session ("done") and writes the report.

### Anti-oscillation

If the car keeps the bus chattering while waiting (so we keep waking), the existing
escalating cooldown (`SLEEP_COOLDOWN_SECS` 10→30→60→120→300s) throttles re-wake. Worst case
degrades from today's continuous 100% poll to a low duty-cycle poll. Two requirements keep
that worst case cheap:

1. **Resume straight into CHARGE_WAIT, re-sleep quickly** — avoid dwelling 60s in HANDSHAKE
   on every resume. In `HandleAwakeState`, when `PISW >= 0x02`, branch on `in_session`:
   `in_session == true` (resuming an existing session that slept) → `TransitionToChargeWaitState()`
   directly; `in_session == false` (a genuinely fresh plug-in) → `TransitionToChargeHandshakeState()`
   as today. Tier 2 then re-sleeps after the threshold. (AC/DC are still reached promptly on
   resume because the WAIT engage-watch and the direct AC/DC checks in `HandleChargeWaitState`
   fire as soon as the charge actually engages.)
2. **Do not `ResetSleepBackoff()` on resume** — only a genuinely new plug-in resets the
   cooldown. Move the `ResetSleepBackoff()` in `TransitionToChargeHandshakeState()` inside
   the `if (!in_session)` block so resuming an existing session keeps the escalated backoff.

## Risks & error handling

**Risk 1 — false session-close on the OBC post-wake PISW transient (primary).**
The code documents a ~30s window after OBC wake where `0x1669` (PISW on PICS) can briefly
read `0x00` before the OBC fully wakes. Today's wake-reconcile guards only on `!IsStale()`,
so a fresh `0x00` *during* the transient would wrongly finalize a still-plugged session.
This design exercises that path routinely.
**Mitigation:** require `PISW == 0x00` to persist across **2 consecutive AWAKE polls**
(≥~6–10s at the AWAKE @5s cadence) before treating it as cable-removed — add a small
debounce counter (`m_pisw_zero_count`), reset on any non-zero PISW. Hardens an already-latent
edge rather than introducing a new one.

**Risk 2 — sleeping through a charge that is electrically quiet on CAN2.**
Passive-wake assumes a scheduled charge start puts frames on CAN2. The 12V rising-edge wake
is an independent second net (DC-DC coming on). If on-vehicle testing shows a miss, the
fallback is a timer-backed wake-check — deferred, not built now.

**Risk 3 — losing the report if 12V still collapses (weak battery).**
Tier 2 makes this far less likely (we stop being the load) but can't prevent a dying 12V
from triggering the hard powermgmt DeepSleep. Same outcome as today. Out of scope.

**Risk 4 — DC re-engage latency after a DC phase ends.**
DC exits to WAIT (`hlc == 0xFF`); Tier-1 10s `0x1666` means ≤10s to catch a DC re-engage.
DC is attended and re-engage is rare. Acceptable.

## Tuning constants (hardcoded, `etnga_poll_states.cpp`, like `SLEEP_COOLDOWN_SECS`)

| Constant | Value | Rationale |
|---|---|---|
| WAIT engage-watch cadence (`0x1684`/`0x1666`/`0x10D1`) | 10s | ~10× traffic cut; ≤10s resume on brief AC pause |
| `CHARGE_WAIT_SLEEP_SECS` (first wait, no charge → sleep) | 600 (10 min) | Long enough not to sleep on a charge starting "soon"; short enough to protect 12V |
| `CHARGE_WAIT_RESLEEP_SECS` (re-sleep after a prior wait-sleep) | 15 | Keeps the oscillation duty cycle low (poll ~15s, sleep up to 300s) |
| PISW cable-removed debounce | 2 consecutive AWAKE reads | Defeats the OBC post-wake `0x00` transient |
| Resume sleep cooldown | reuse existing 10→300s backoff; not reset on resume | Throttles worst-case oscillation |

No new config params in v1. The threshold can be promoted to an `[xte] charge_wait_sleep`
config later if field-tunable behavior is wanted — noted as a follow-up, not built now.

## Testing (on-vehicle — cannot be unit-tested on host)

1. **Scheduled-charge happy path:** plug in at a charger set to delayed start. Confirm:
   Tier-1 slow poll → Tier-2 sleep after 10 min → 12V holds overnight → wakes and charges
   when the schedule fires → **one continuous session + report**, no kWh reset across the
   sleep.
2. **Unplug while asleep:** plug in, let it sleep, unplug during sleep. Confirm the session
   finalizes cleanly (report written, `in_session` cleared, no leak) and the PISW debounce
   did not false-close earlier.
3. **AC pause/resume:** during AC charging, induce a brief pause (`ac_op == 0x00` → WAIT).
   Confirm Tier-1 catches resume within ~10s and the session/kWh survive.
4. **12V trend:** confirm no `powermgmt: 12V battery critical` during a long plugged-in wait.

## changes.txt

User-facing behavior change (charge-wait now sleeps to protect 12V; scheduled charges are no
longer missed) → add a `changes.txt` entry. No new config keys.
