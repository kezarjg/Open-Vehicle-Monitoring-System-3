# e-TNGA: suspend charge logging/accounting during a CAN-stale window (#138) — design

**Date:** 2026-06-24
**Issue:** [#138](https://github.com/kezarjg/Open-Vehicle-Monitoring-System-3/issues/138) — charge CSV + energy accounting filled with stale values during a lock (CAN-quiet)
**Status:** design — pending user review

## Problem

When the car is **locked during an AC charge**, the OBD gateway isolates the diagnostic path from the OBC, so the charge metrics stop updating (CAN goes quiet). But `UpdateChargeSessionStats()` runs every tick (~1 s) and keeps logging + integrating **frozen** values:

- the per-sample **CSV row** is written with stale data (`battery_kw` held at its last value while `pack_a` clears to 0 — physically impossible: kW at 0 A);
- the e-TNGA per-tick energy accumulator **`station_kwh`** (∫ frozen grid power) over-counts by ~0.2 kWh over a ~110 s lock, and **`delivered_ah`** under-counts (`pack_a`=0);
- the **SVG chart** records a flat stale plateau.

Net result: phantom station energy and a nonsensical efficiency (observed: "52% to battery" / "158%"). The state machine itself rides the lock correctly (only a *fresh* `pisw=0`/`ac_op=0`/`hlc=0xFF` ends a phase/session) — this is purely a logged/accumulated **data-quality** bug.

**Not affected (verified):** the headline **`v.c.kwh`** ("delivered") is integrated by e-TNGA *per poll-reply* (`SetBatteryChargingPower`), not per-tick, and `EnergyIntervalHours` already clamps gaps >60 s to 1 s — so a lock contributes no integration during it and a clamped bridge on resume. It is **not** significantly over-counted; no fix needed there. The corruption is entirely the **per-tick** work in `UpdateChargeSessionStats`.

The existing staleness handlers use `IsStale()` (`SM_STALE_MID` = **120 s**) — far too slow to catch a lock.

## Goal

Quickly detect that the CAN poll path has stopped responding and **suspend all per-sample logging + energy accumulation** until it resumes, so a lock leaves an honest **gap** rather than a fabricated plateau, and every energy number reflects only measured charging.

## Design

### Detection — a dedicated "last poll reply" timestamp

Add a member `int m_last_poll_monotonic = 0;`, stamped to `ms_m_monotonic` at the top of **`IncomingPollReply()`** on *every* reply (any ECU). In `UpdateChargeSessionStats()`:

```cpp
const int CHARGE_STALE_SECS = 3;   // charge poll path runs at 1s; 3 missed = clearly stale but quick
int now = StandardMetrics.ms_m_monotonic->AsInt();
bool can_stale = (m_last_poll_monotonic == 0) || (now - m_last_poll_monotonic > CHARGE_STALE_SECS);
```

Rationale for a dedicated stamp rather than a metric's `Age()`: (a) we *zero* the power metrics on stale, so using their Age would be circular; (b) "when did we last hear from the car at all" is the semantically-correct signal and is robust to which ECU/DID — the whole OBD path freezes together under a lock (observed: `pack_a`/`0x7D2` telemetry cleared, not just the OBC `0x745`). The stamp is written on the poller task and read on the vehicle-ticker task; it is a plain `int` like the existing `m_charge_fault_pending` cross-task pattern — a benign unsynchronized scalar (worst case: one tick early/late at the boundary).

### Suspend — early-return branch at the top of `UpdateChargeSessionStats()`

When `can_stale` (and `in_session`):

1. **Zero the live power/current metrics** for display honesty — during the lock these are frozen at their last value (e.g. 6.85 kW, which is what the CSV/app/server would otherwise show); 0 reads as "not measuring":
   ```cpp
   StandardMetrics.ms_v_charge_power->SetValue(0);
   StandardMetrics.ms_v_bat_power->SetValue(0);
   StandardMetrics.ms_v_bat_current->SetValue(0);
   ```
   This is **display-only** — it does not touch the energy integrals (`v.c.kwh` recomputes from pack V×I on the next reply; the per-tick accumulators are suspended below). It does **not** end the charge (gated on `ac_op`/`pisw`), and the existing "keep charge-state fresh" code holds `v.c.state = charging`. It mirrors the existing 120 s `ResetStaleMetrics` power-zeroing, just on the quick charge-stale clock.
2. **Bump the dt baselines** so resume doesn't bridge the gap:
   ```cpp
   m_charge_session.last_sample_monotonic = now;
   m_charge_session.last_svg_monotonic   = now;
   ```
3. **`return;`** — skipping the per-sample CSV row (`AppendChargeCsvRow`), the `station_kwh`/`delivered_ah` accumulation, the SVG sample push, and peak/temp/ambient folding (frozen values add nothing).

The rest of `UpdateChargeSessionStats()` (the normal logging/accumulation) runs only when **not** stale.

### Resume — automatic

The next poll reply advances `m_last_poll_monotonic`; `can_stale` goes false; the next tick resumes normal logging/accumulation. Because the dt baselines were kept current during the gap, the first post-resume `delivered_ah`/`station_kwh` step is a normal ~1 s interval (the existing `dt > 0 && dt <= 10` clamp also covers any longer gap).

## Scope / non-goals

- **In:** the suspension + power-zeroing above. One member + one stamp line in `IncomingPollReply`; one branch + a `CHARGE_STALE_SECS` const in `UpdateChargeSessionStats` (`etnga_charge_report.cpp`).
- **Out:** changing the 120 s `ResetStaleMetrics` handler (leave as-is; the charge-specific quick path lives in `UpdateChargeSessionStats`), the lock-detection state-machine behaviour (already correct), and any report annotation of the gap (the honest numbers are the deliverable; a "telemetry coverage" line is a possible later nicety, not this fix).
- No new polls.

## Files

| File | Change |
|---|---|
| `src/vehicle_toyota_etnga.h` | add `int m_last_poll_monotonic = 0;` |
| `src/etnga_poll_processor.cpp` | stamp `m_last_poll_monotonic = ms_m_monotonic` at the top of `IncomingPollReply` |
| `src/etnga_charge_report.cpp` | `CHARGE_STALE_SECS` const + the `can_stale` early-return branch in `UpdateChargeSessionStats` |
| `vehicle/OVMS.V3/changes.txt` | entry: charge energy accounting stays honest during a lock; `v.c.power` reads 0 during a CAN-stale window |

## Validation

- **Compile:** CI.
- **On-vehicle (the repro):** lock the car mid-AC-charge for >a few seconds. Expect: the CSV stops adding rows during the lock (no frozen-value rows / no `battery_kw>0 @ pack_a=0`), the SVG shows a gap, the report's `station_kwh`/efficiency/delivered-kWh are sensible (no phantom ~0.2 kWh, no >100% efficiency), `v.c.power` reads 0 during the lock and recovers on unlock, and the session/phase is not torn down.
