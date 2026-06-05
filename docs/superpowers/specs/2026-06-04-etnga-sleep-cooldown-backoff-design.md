# e-TNGA adaptive sleep cooldown backoff — design

**Date:** 2026-06-04
**Author:** Jerry Kezar
**Component:** `vehicle/OVMS.V3/components/vehicle_toyota_etnga`
**Branch:** `feature/etnga-sleep-cooldown-backoff`

## Problem

Review of the 2026-06-04 drive logs showed the state machine behaving correctly,
but with a high parked-but-bus-noisy **duty cycle**. After a drive the module
repeatedly cycled:

```
AWAKE ~300s (door-watch watchdog) → forced SLEEP → 10s cooldown → CAN frame re-wakes → AWAKE …
```

The forced-sleep cooldown is a fixed 10s (`HandleSleepState`), and after it
expires the next stray CAN2 frame sets `ms_v_env_awake` again and drags the
module back to a full 5-minute AWAKE poll cycle. Observed steady state was
roughly **~77% awake** while parked with an intermittently-live bus — wasted
12V draw with no driving or charging happening.

The natural `env_awake`-stale `AWAKE→SLEEP` path (when the bus genuinely goes
quiet) is *not* cooldown-latched at all, which also produces rapid ~3s
`SLEEP→AWAKE` bounces.

## Goal

Reduce the parked duty cycle by **escalating the sleep cooldown** on consecutive
no-activity sleeps, while keeping the module responsive to a real return via the
existing **12V-rise wake** escape. No change to driving/charging behavior.

This is a 12V-drain mitigation. Actual drain improvement must be confirmed
on-vehicle; the design is bounded so the worst case is "ignore CAN for up to the
current cooldown, ≤5 min, but still wake on a 12V power-up."

## Design

### State

A single shared escalation index selects from a hardcoded cooldown schedule.
All logic stays in `etnga_poll_states.cpp`.

- **Schedule** (file-scope `static const` in `etnga_poll_states.cpp`):
  `SLEEP_COOLDOWN_SECS[] = {10, 30, 60, 120, 300}` (seconds; cap 5 min).
- **New members** (`vehicle_toyota_etnga.h`):
  - `int m_sleep_backoff_idx = 0;` — index into the schedule.
  - `int m_sleep_cooldown_secs = 10;` — the cooldown that applied to the
    *current* sleep (captured at sleep entry, read in `HandleSleepState`).
- **New helper** (`etnga_poll_states.cpp`): `void ResetSleepBackoff()` →
  `m_sleep_backoff_idx = 0;`

### Arming (escalate) — centralized in `TransitionToSleepState()`

`TransitionToSleepState()` becomes the single place that arms the cooldown.
On every sleep it:

1. sets `m_sleep_entry_time = monotonic`
2. captures `m_sleep_cooldown_secs = SLEEP_COOLDOWN_SECS[m_sleep_backoff_idx]`
3. sets `m_allow_wake = false`
4. increments `m_sleep_backoff_idx`, clamped to the last schedule entry

Because every `AWAKE→SLEEP` / `CHARGE_WAIT→SLEEP` edge already routes through
`TransitionToSleepState()`, this uniformly covers all four sleep sites:

- door-watch forced sleep (`HandleAwakeState`, 300s)
- cable-watch forced sleep (`HandleAwakeState`, 900s)
- charge-wait bus-dead sleep (`HandleChargeWaitState`)
- natural `env_awake`-stale sleep (`HandleAwakeState`)

The three forced sites therefore **drop their now-duplicated**
`m_sleep_entry_time = …; m_allow_wake = false;` lines (handled centrally), and
the natural-stale site gains cooldown arming for free — which also damps the
3s `SLEEP→AWAKE` bounces.

### Cooldown expiry — `HandleSleepState()`

Change the fixed check from `> 10` to `> m_sleep_cooldown_secs`, and log the
actual duration:

```cpp
if (!m_allow_wake) {
    if ((monotonic - m_sleep_entry_time) > m_sleep_cooldown_secs) {
        ESP_LOGI(TAG, "Cooling off period ended (%ds), allowing wake", m_sleep_cooldown_secs);
        m_allow_wake = true;
    }
}
```

### Reset (`idx = 0`) — real activity snaps back to responsive

`ResetSleepBackoff()` is called at the four real-activity points:

| Point | Location |
|-------|----------|
| Enter DRIVING | `TransitionToDrivingState()` |
| Enter charge handshake | `TransitionToChargeHandshakeState()` |
| 12V-rise wake | `HandleSleepState()` 12V branch (before `TransitionToAwakeState`) |
| Charge door opens (arm) | `HandleAwakeState()`, the `lid_open` arm block |

Bare CAN-frame wakes that only reach `AWAKE` (no drive/charge) do **not** reset
— that is the mechanism that lets the ramp climb during persistent parked
chatter.

### 12V escape (the responsiveness guarantee)

The existing 12V-rise wake path in `HandleSleepState()`
(`ms_v_bat_12v_voltage > ref + 0.2`) is **not** gated by `m_allow_wake`, so it
already fires during a cooldown. This design preserves that and adds a
`ResetSleepBackoff()` call so a real power-up resumes responsive (10s)
cooldowns. This escape is what makes the long end of the ramp safe: while CAN is
ignored (≤5 min), a genuine return that raises 12V still wakes immediately.

## Expected behavior

- Parked-with-chatter steady-state duty cycle drops from ~77% toward ~50%
  (300s awake watchdog + 300s capped cooldown), ramping over ~4 cycles after
  parking.
- First sleep after any real activity is still 10s — a returning driver is
  caught by CAN while `idx` is low, and by the 12V escape once it has climbed.
- Cleanly-parked car (bus goes quiet, stays asleep) only sleeps once, so `idx`
  stays low — escalation only builds under active cycling, exactly the drain
  scenario.

## Risks

- **12V reliability is load-bearing at the aggressive end.** If a wake powers the
  bus but not the 12V rail above `ref + 0.2`, CAN is ignored for up to the
  current cooldown (≤5 min). The existing code already trusts this 12V signal, so
  no new assumption is introduced — but the backoff leans on it harder.
  On-vehicle measurement is the acceptance gate.
- Behavior change to a 12V-critical loop: validate on the real module before
  merging.

## Validation

No host unit tests (hardware project). Acceptance is on-vehicle:

1. `make` builds clean.
2. Flash; park and let it cycle. Confirm in logs the ramp:
   `Cooling off period ended (10s/30s/60s/120s/300s)`.
3. Confirm a 12V-rise wake logs `Aux 12V has exceeded the threshold` and the
   *next* cooldown is back to 10s (reset worked).
4. Confirm a real drive and a charge plug-in each reset to 10s.
5. Optionally compare `ms_v_bat_12v_voltage` across parked windows for actual
   sag improvement.

## Out of scope (YAGNI)

- Making the schedule config-tunable via `xte` params (revisit only if repeated
  on-vehicle tuning proves necessary).
- Shortening the 300s AWAKE door-watch watchdog (the other duty-cycle lever) —
  separate change, separate validation.

## Bundled change

This branch also carries improvement **#1** (observability): the two watchdog
reason logs promoted `ESP_LOGD → ESP_LOGI`, plus a reason line on the natural
`env_awake`-stale sleep, so production (info-level) logs show *why* the module
slept. This directly supports validating the backoff above.
