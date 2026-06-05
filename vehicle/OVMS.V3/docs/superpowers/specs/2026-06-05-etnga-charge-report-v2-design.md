# Toyota e-TNGA — Charge Session Report v2 (design)

**Date:** 2026-06-05
**Status:** Design approved (brainstorming), pending implementation plan.
**Supersedes:** the v1 single-phase report (#86, `etnga_charge_report.cpp`).
**Related:** parent #81; follow-ups #87; decode reference `solterra-can/docs/charging_state_machine_architecture.md` §6 and `ecus/plug-in-charge-control.md` (0x1688 enum).

## Dependencies / base

v2 touches code from several in-flight branches, so it must be developed against a base that contains all of them — i.e. **master once #83/#85/#86/#88 have merged**, or the combined integration branch in the interim:
- **#86** — the v1 report code it extends (`etnga_charge_report.cpp`, `ChargeSessionState`).
- **#88** — the `/xte/report` viewer it extends to serve CSV, and the `/xte/reports` index.
- **#85** — `v.c.kwh.grid.total` and the coulomb infrastructure reused for energy/Ah accounting.

If those have not merged when implementation starts, branch from the combined integration branch (`test/etnga-combined`) rather than master.

## Goal

Make the end-of-charge report substantially more useful across four user goals: diagnose charge performance, energy/cost accounting, a richer per-session record, and battery-health tracking. Presentation is **two-tier**: a confident **Measured** section and a clearly-labelled **Estimates** section.

## Foundational bug fix — charge-power source

The v1 report's "Peak Power" is wrong because `ms_v_charge_power` is set **only** by the `PID_CHARGER_INPUT_POWER` (`0x161D`) handler, which is gated to `CHARGE_AC` (`etnga_poll_processor.cpp:225`):
- DC charge → `ms_v_charge_power` never updates (peak stale/zero).
- AC charge → it is grid-input power, not battery-delivered power.

The real delivered power (`0x10D4`, valid AC+DC, polled 1 s) flows into `SetBatteryChargingPower()` but is only integrated to energy, never stored live.

**Fix:** in `SetBatteryChargingPower()` also `ms_v_charge_power->SetValue(power)` (delivered power, AC+DC). Treat grid-input (`0x161D`) as a distinct signal used for efficiency and the CSV. All of peak/avg/chart/CSV "delivered" then read a correct value.

## Data captured during a session

Extend `ChargeSessionState` and add streaming:
- **Location:** `v.p.latitude` / `v.p.longitude` / `v.p.gpslock` snapshotted at session open (`TransitionToChargeHandshakeState`).
- **Ambient:** `v.e.temp` at open + running min/max.
- **Event log:** `std::vector<{uint32 monotonic, const char* label}>` appended at each state transition (plugged-in / handshake / AC-begin / DC-begin / phase-end / re-arm bounce / unplug) in the `TransitionToCharge*` and `TransitionToAwakeState` functions.
- **Delivered-Ah counter:** today coulomb integrates only while driving; add a charge-side Ah accumulation (integrate pack current during charge) for the implied-capacity estimate.
- **SVG sample buffer:** small in-RAM ring (~200–300 points, interval auto-scaled to session length) for the inline chart — bounded RAM, independent of the CSV.

## CSV (fine-grained, streamed)

- Written **per sample (~1 s)** during the charge, appended straight to the file (not buffered in RAM), so granularity is fine without RAM cost.
- **Columns:** `wallclock_iso, elapsed_s, soc_pct, delivered_kw, pack_v, pack_a, batt_temp_c, ambient_c, state, station_max_kw, station_max_a, station_max_v, car_perm_kw, target_a, grid_kw, present_v, present_a`. Columns not applicable to the current phase (AC vs DC) are left blank.
- Sources are already polled into `xte.v.c.*` metrics (`stamaxp/stamaxi/stamaxv`, `perm`, `tgti`, `acusbl`, `acilim`, present V/A) plus the fixed delivered power — enabling **offered-vs-allowed-vs-delivered** (limiting-side) analysis offline, instead of an in-firmware heuristic.

## HTML report structure

1. **Summary (Measured):** plug-in/unplug time + duration; location (coords + OpenStreetMap link); ambient range; AC/DC; SOC delta; energy delivered + from grid; peak/avg delivered power; battery-temp range; **human-readable outcome** (from the `0x1688` enum table).
2. **Charging power vs time:** **inline SVG** line of delivered power (light SOC overlay), self-contained, no JS/CDN — renders on-module and copied off.
3. **Session event log:** timestamped transition table.
4. **Estimates (labelled):** charging efficiency (delivered kWh ÷ grid kWh) + losses; implied pack capacity (delivered Ah ÷ ΔSOC) and implied SOH vs ~201.1 Ah nominal.
5. **Link to the companion CSV.**

## Storage & retention

- Write to **`/sd/charge-reports/`** when the SD card is mounted (GBs of space; pull the card or scp from `/sd`), **falling back to `/store/charge-reports/`** if no SD.
- HTML + CSV pair share the same `<timestamp>` basename and directory.
- Retention: keep the newest **50 sessions** (delete oldest HTML+CSV pairs together).

## Web UI

- Extend the `/xte/report` viewer (from #88) to serve `.csv` as `text/csv` (download) in addition to `.html`.
- `/xte/reports` index lists each session with links to both its report and CSV; resolves the active directory (`/sd` or `/store`).

## Components / boundaries

- `etnga_charge_report.cpp` — owns report HTML generation (incl. SVG), CSV streaming (open/append/close), directory resolution, and retention. Public entry points: `UpdateChargeSessionStats()` (per-tick: aggregates + appends CSV row + feeds SVG buffer), `GenerateChargeReport()` (session end: writes HTML, closes CSV, prunes).
- `ChargeSessionState` (header) — session aggregates + event log + SVG buffer + open CSV handle/path.
- Transition functions (`etnga_poll_states.cpp`) — append event-log entries; open/close the session.
- `SetBatteryChargingPower()` (`etnga_metrics.cpp`) — the charge-power-source fix.
- Outcome-label lookup — a small static table sourced from the decoded `0x1688` enum.

## Out of scope (remain in #87)

Multi-phase segmentation (v2 treats the session as one continuous timeline), sleep-survival / summary-mode (if the module sleeps mid-charge the CSV/SVG/event-log may be incomplete — documented limitation), in-firmware limiting-side attribution (now an offline CSV analysis), error DID-dump, and the real-time per-phase charge-complete notification/ABRP event.

## Validation

- Compile gate via CI.
- On-vehicle: AC and DC charges each produce an HTML+CSV pair on `/sd`; **peak power now correct on DC**; SVG renders on-module and when the file is copied off; CSV opens in a spreadsheet with offered/allowed/delivered columns populated; outcome label matches the session; estimates plausible; retention prunes at 50.
