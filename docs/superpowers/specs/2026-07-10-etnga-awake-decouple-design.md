# e-TNGA: decouple CAN2 bus-liveness from `v.e.awake`

**Date:** 2026-07-10
**Branch:** `feature/etnga-awake-decouple`
**Scope:** `components/vehicle_toyota_etnga` only. No framework/base-class change. Single-purpose fork PR.

## Problem

The e-TNGA overloads the standard metric `v.e.awake` (documented as "Vehicle is fully
awake (switched on by the user)", `metrics_standard.h:478`) to mean "the CAN2 bus has
recent traffic." `IncomingFrameCan2` calls `SetAwake(true)` on every CAN2 frame
(`etnga_can_processor.cpp:29`), and the metric's built-in ~120 s auto-stale is used to
detect "bus went dead → sleep" (`etnga_metrics.cpp:88-90`). Because the module stays awake
to poll throughout an AC/DC charge, `v.e.awake` stays **true for the whole charge**.

Every framework consumer of `v.e.awake` then reports "on" during a charge:

- `vehicle.cpp:1096-1105` — the idle detector fires `NotifyVehicleIdling()`
  ("Vehicle is idling / stopped turned on") 15 min into the charge, then hourly. This is
  the user-visible symptom: Home Assistant showed the parked, charging car as on/idle.
- `ovms_server_v2.cpp:1597` / `ovms_server_v3.cpp:1370` — the `CarAwake` bit sent to the
  server/app is set true during charge.
- `vehicle.cpp:2019-2024` — the `vehicle.awake` event + `NotifiedVehicleAwake()` fire on
  every charge wake.

**Observed** (logs, night of 2026-07-09→10): AC charge ran 23:00:43 → 06:49:01
(SOC 25%→100%). "Vehicle is idling" alerts fired at 23:15, 00:18, 01:15, 02:17, 03:16,
04:16, 05:18, 06:18 — entirely inside the charge window.

The closest analogue vehicle, Hyundai Ioniq5, sets `ms_v_env_awake = isOn` (car on, false
during charge) and drives charge polling from a dedicated poll state
(`vehicle_hyundai_ioniq5.cpp:661, 850`). That is the fleet norm: `awake` = the user
switched the car on; charging is a separate poll-state concern with the car off.

## Goal

Separate the two concepts currently conflated in `v.e.awake`:

1. **CAN2 bus-liveness** — an internal signal that drives the `SLEEP ↔ AWAKE` state
   transitions. Stays exactly as capable as today.
2. **`v.e.awake` metric** — restored to the standard "switched on" semantic, so framework
   consumers stop reporting on/idle during a charge.

Non-goals: no base-class idle-alert change; no change to the 12 V wake path; no change to
charge polling, sleep cooldown/backoff, or the chargeport-latch logic.

## Design

### 1. Internal bus-liveness (drives `SLEEP ↔ AWAKE`)

Replace the metric-staleness mechanism with an explicit member timestamp.

- New member: `uint32_t m_last_can2_time = 0;` — monotonic **seconds** (from
  `StandardMetrics.ms_m_monotonic->AsInt()`, the same clock the CHARGE_WAIT 12 V-drain
  timer already uses in `etnga_poll_states.cpp`).
- New constant: `static constexpr uint32_t BUS_STALE_SECS = 120;` — equals `SM_STALE_MID`
  (`metrics_standard.h:38`), the auto-stale period `v.e.awake` uses today.
- New helper:
  ```cpp
  bool OvmsVehicleToyotaETNGA::IsBusAlive() const
  {
      if (m_last_can2_time == 0)
          return false;
      uint32_t now = StandardMetrics.ms_m_monotonic->AsInt();
      return (now - m_last_can2_time) <= BUS_STALE_SECS;
  }
  ```

**Faithful-port justification.** `OvmsMetric::IsStale()` is
`m_lastmodified + m_autostale < monotonictime` → *not* stale while
`now - m_lastmodified <= 120`. Every CAN2 frame refreshed `m_lastmodified` because
`OvmsMetricBool::SetValue` calls `SetModified` in **both** the changed and unchanged
branches (`ovms_metrics.cpp`), so today's "alive" means "a frame within the last 120 s."
`IsBusAlive()` reproduces that comparison on the same monotonic-seconds clock.

### 2. `v.e.awake` metric (standard "switched on" semantic)

Make the metric a pure function of poll state, set in the single transition choke point.

- `SetPollState(int state)` is the sole path for every transition (each `TransitionTo*`
  calls it; it is where the "Transitioning from the X to the Y state" log is emitted).
  Add there:
  ```cpp
  SetAwake(state == PollState::AWAKE || state == PollState::DRIVING);
  ```
- Result: `v.e.awake` is **true in AWAKE and DRIVING**, **false in SLEEP and all four
  CHARGE_\* states** (HANDSHAKE / WAIT / AC / DC). This keeps `awake` distinct from and a
  superset of `v.e.on` (Ready), and excludes charging.

`SetAwake(bool)` (`etnga_metrics.cpp:645`) keeps its one-line body; it is now called only
from `SetPollState` and from explicit initialization, never per-frame.

### 3. The forced-sleep liveness reset (critical subtlety)

`TransitionToSleepState` currently ends with `SetAwake(false)` (`etnga_poll_states.cpp:380`).
This is **not** redundant: when sleep is *forced while the bus is still alive* — the
CHARGE_WAIT 12 V-drain protection, and the sleep-cooldown path — forcing awake false makes
the next wake require a **fresh** CAN2 frame (rising-edge), preventing an immediate bounce
back to AWAKE.

To preserve this under the member-based liveness, `TransitionToSleepState` must **reset the
liveness timestamp**: replace `SetAwake(false)` with
```cpp
m_last_can2_time = 0;   // force rising-edge: wake requires a fresh CAN2 frame
```
The `v.e.awake` metric is set false anyway by `SetPollState(PollState::SLEEP)` inside the
same transition, so no separate metric write is needed here.

Without this reset, a forced sleep taken while `m_last_can2_time` is recent would leave
`IsBusAlive()` true, and after the cooldown window the SLEEP handler would wake immediately
instead of waiting for a fresh frame — a behavior change we explicitly avoid.

## Complete change list

| # | Site | Today | Change |
|---|------|-------|--------|
| 1 | `etnga_can_processor.cpp:29` | `SetAwake(true)` per CAN2 frame | `m_last_can2_time = StandardMetrics.ms_m_monotonic->AsInt();` |
| 2 | `etnga_poll_states.cpp:65` (SLEEP: wake if alive) | `ms_v_env_awake->AsBool()` | `IsBusAlive()` |
| 3 | `etnga_poll_states.cpp:168` (AWAKE→sleep if dead) | `!ms_v_env_awake->AsBool()` | `!IsBusAlive()` |
| 4 | `etnga_poll_states.cpp:283` (CHARGE_WAIT→sleep if dead) | `!ms_v_env_awake->AsBool()` | `!IsBusAlive()` |
| 5 | `etnga_poll_states.cpp:380` (`TransitionToSleepState`) | `SetAwake(false)` | `m_last_can2_time = 0;` |
| 6 | `etnga_metrics.cpp:88-90` (awake stale-reset block) | force awake false on stale | **delete** |
| 7 | `SetPollState(...)` (`etnga_metrics.cpp`) | — | add `SetAwake(state == AWAKE \|\| state == DRIVING)` |
| 8 | `vehicle_toyota_etnga.h` | — | declare `uint32_t m_last_can2_time`, `BUS_STALE_SECS`, `bool IsBusAlive() const` |

Unchanged and intentionally kept:
- `SetAwake(false)` at init (`etnga_metrics.cpp:54`, `vehicle_toyota_etnga.cpp:165`) — a
  harmless explicit initial value; `SetPollState(PollState::SLEEP)` at boot also sets it.
- `SetAwake` helper body (`etnga_metrics.cpp:645`).
- The 12 V rising-edge wake path in the SLEEP handler (separate from bus-liveness).
- Charge-state polling, sleep cooldown/backoff, chargeport-latch / `in_session` logic.

## Why it's behavior-preserving for the state machine

- `SLEEP ↔ AWAKE` timing is identical: same 120 s window, same clock, same
  rising-edge-on-forced-sleep behavior (via the `m_last_can2_time = 0` reset).
- Charge polling is unaffected: the charge states never used `awake` to *stay* in charge;
  they only used it (now `IsBusAlive()`) for the bus-dead → sleep guard, and the OBC keeps
  CAN2 alive during a real charge.
- The only intended behavior change is the value of the `v.e.awake` **metric** during
  charge (now false), which is the fix.

## Consequences (the payoff)

- Base idle detector no longer fires during a charge → no more "Vehicle is idling"
  notifications while parked and charging (the reported HA symptom).
- v2/v3 `CarAwake` bit and `vehicle.awake` event now reflect genuine on-state, not charge.
- `v.e.awake` becomes informative (distinct from `v.e.on`) rather than a duplicate of
  bus-liveness.

## Testing / validation

No host unit-test suite exists; firmware is validated by CI build + on-vehicle runtime
(per CLAUDE.md). Validation plan:

1. **CI build** green (firmware `build.yml`) with the e-TNGA selected.
2. **On-vehicle, static checks via `metrics`/logs:**
   - Parked, bus quiet → module enters SLEEP ~120 s after last CAN2 frame (unchanged).
   - Passive CAN wake (open door) → SLEEP→AWAKE; `v.e.awake` becomes true only on the
     transition to AWAKE.
   - 12 V rising-edge wake still works (uncalibrated-ref fallback path unaffected).
3. **On-vehicle, AC charge session:** confirm `v.e.awake == false` throughout; confirm the
   "Vehicle is idling" notification does **not** fire; confirm charge metrics/report still
   populate; confirm no spurious SLEEP↔AWAKE oscillation.
4. **CHARGE_WAIT 12 V-drain path:** plug in with delayed/scheduled charge; confirm the
   tiered slow-poll → forced sleep → passive resume still works and does not immediately
   bounce (validates the `m_last_can2_time = 0` reset).

Items 3–4 require a real vehicle and cannot be verified in this environment.

## Documentation & housekeeping

- `changes.txt`: add an entry — user-noticeable behavior change (no idle alerts while
  charging; `v.e.awake` now excludes charging). No new config keys.
- `components/vehicle_toyota_etnga/docs/state_machine.rst`: update the "Two views of
  'vehicle on'" note and the `env_awake` auto-stale note to describe the new split
  (internal `m_last_can2_time` drives SLEEP↔AWAKE; `v.e.awake` reflects poll state).

## Risks

- **Subtle timing drift** if `ms_m_monotonic` and the metric's `monotonictime` differ by a
  tick. Both advance once per second from the same source; the 120 s window is tolerant of
  a ±1 s difference. Low risk.
- **A missed `awake` reader.** Audit confirmed exactly five `ms_v_env_awake` references and
  seven `SetAwake` references in the component (all in the change list). Re-grep during
  implementation to guard against drift.
