# e-TNGA charging monitor: AC/DC views + live chart + state history

**Date:** 2026-06-06
**Component:** `vehicle/OVMS.V3/components/vehicle_toyota_etnga`
**Status:** design approved, ready for implementation plan
**Follow-up to:** charge report v2 (#90) and the report refinements (#95)

## Goal

Improve the **Charging monitor** web page (`/xte/charge`, `WebDispChgMetrics`) so it:

1. Shows **distinct layouts for AC vs DC** charging, auto-selected from the active session.
2. Has a **live, auto-updating chart** carrying the same series as the finished report's chart
   (delivered power, station-offered max [DC], car-permitted limit, SOC over time).
3. Shows a **live "charging state history"** — timestamped state transitions — using the DC HLC
   sequence (0x1666) on DC and the AC Operation Status sequence (0x1684) on AC. The same history
   is persisted in the report's event log.

Non-goals: changing CAN polling behavior beyond reading already-polled metrics; adding new routes;
persisting metric history server-side; changing the report layout other than the event-log additions
described below.

## Background / constraints (from the OVMS web framework)

- **Highcharts 6.0.7** is bundled (`components/ovms_webserver/assets/highcharts.js`) and lazy-loaded
  with the standard idiom: `if (window.Highcharts) init(); else $.ajax({url: charts.js, dataType:
  "script", cache:true, success: init})`. jQuery 1.12.4 is global.
- Live metric values reach the browser over a WebSocket and fire a jQuery `msg:metrics` event on
  `.receiver` elements; the global `metrics` object holds current values. This is the standard live
  feed (used by the BMS cell monitor and the Renault Twizy dyno chart).
- **There is no server-side metric history.** A pure browser-side chart only accumulates from page
  open. **However**, the module keeps the whole-session sample buffer (`m_charge_session.svg`) and
  event log (`m_charge_session.events`) in memory while a session is open — the same data the report
  is built from.
- Page handlers (`WebDispChgMetrics`, …) are **static**. They reach the active vehicle instance via
  `MyVehicleFactory.ActiveVehicle()` cast to `OvmsVehicleToyotaETNGA*`; a static member may read the
  instance's private fields. The page is registered only for e-TNGA vehicles.

## Architecture

`WebDispChgMetrics` (static) is restructured to, at render time:

1. Resolve the active e-TNGA instance and read session state: `in_session`, `is_dc`, the `svg`
   sample buffer and the `events` list.
2. Decide the view:
   - **DC** if charging and `is_dc`
   - **AC** if charging and not `is_dc`
   - **Idle** if not in a session.
3. Emit the chosen layout (metric panels), the chart container, the state-history container, and a
   `<script>` block that:
   - seeds the chart from an embedded JS literal of the current sample buffer
     (`var INIT=[[t_s,kw,soc,sta_max,car_perm], …]`),
   - seeds the state-history table from an embedded literal of the event log
     (`var EVT=[[t_s,"label"], …]`),
   - lazy-loads Highcharts and initializes the chart with axes/series for the view,
   - subscribes to `msg:metrics` and on each update appends a chart point and, when the relevant
     state metric changes, a new history row.

**Backfill without a new endpoint:** because the page is server-rendered on every load, the embedded
`INIT`/`EVT` literals give the full session immediately, even when opened mid-charge. No new route,
no JSON handler.

**View is fixed at page load.** If the charge type changes while the page is open (rare — e.g. a
phase change), a reload re-selects the layout. On live updates the JS compares the live charge type
to the rendered view and, on mismatch, shows a small "charge type changed — reload" hint rather than
trying to re-render in place.

### Files & structure

All changes are in the e-TNGA component; no new files required.

- `etnga_web.cpp`
  - `WebDispChgMetrics` — restructured dispatcher (idle / AC / DC).
  - `WebChgRenderAc(PageContext_t&, instance)` — AC metric panels.
  - `WebChgRenderDc(PageContext_t&, instance)` — DC metric panels.
  - `WebChgChartJs(PageContext_t&, instance, bool dc)` — emits `INIT`/`EVT` literals + chart/history
    JS. Shared by both views; `dc` toggles the station-max series and which state metric is watched.
  - A small JS-side label map for HLC (0x1666) and AC-Op (0x1684) states is emitted inline so the
    live history rows are human-readable without a new string metric.
- `etnga_metrics.cpp`
  - `SetAcOpStatus(int)` — add change-detection that logs transitions via `LogChargeEvent`
    (mirrors the existing `SetHlcState` logic added in #95).
- `etnga_charge_report.cpp`
  - `AcOpStatusLabel(int)` — new decoder: `1→"AC: Startup"`, `2→"AC: Running"`, `3→"AC: Finishing"`,
    `0 (Stop)→""` (skipped, so DC sessions — where AC-Op stays Stop — log nothing from it; the final
    stop is already represented by the outcome / "Charging paused" / "Unplugged" events).
- `vehicle_toyota_etnga.h`
  - declare `AcOpStatusLabel(int)`; add `int last_acop = -1;` to `ChargeSessionState` for AC-Op
    change-detection (auto-reset with the struct between sessions, like `last_hlc`).

## The live chart

Highcharts line/spline, mirroring the report's SVG chart:

- **X axis:** elapsed time. Seeded from `INIT` `t_s` (seconds); live points appended at
  `last_t + (clientNow - loadTime)`. Displayed in minutes.
- **Left Y axis — power (kW):** fixed full scale per type — **AC 0–11, DC 0–150** (same constants as
  the report).
- **Right Y axis — SOC (%):** fixed **0–100**.
- **Series:**
  - delivered power (`v.c.power`) — solid, left axis
  - car-permitted (`|xte.v.c.perm|`) — dashed, left axis (0x16A1 is signed; plot magnitude)
  - station-offered max (`xte.v.c.stamaxp`) — dashed, left axis, **DC only**
  - SOC (`v.b.soc`) — line, right axis
- **Live append:** on each `msg:metrics`, push one point per series from the current `metrics`
  values; cap series length (~600 points) by dropping the oldest.

## The charging state history (live + report)

- **Report:** already a single event log. Adding AC-Op transition logging (via `SetAcOpStatus`) puts
  `AC: Startup/Running/Finishing` rows into AC-session reports, symmetric with the `HLC: …` rows that
  DC sessions already get from `SetHlcState`.
- **Monitor:** a "Charging state history" table seeded from the embedded `EVT` literal (the full
  in-memory event log, including plug-in/started/paused/unplug and the HLC/AC-Op transitions). Live,
  on each `msg:metrics`, the JS watches the relevant state metric — `xte.v.c.hlc` on DC,
  `xte.v.c.acop` on AC — and appends a new `mm:ss` + label row when it changes (using the inline
  label maps; Stop maps to nothing on AC). This reproduces, live, what the finished report shows.

## View contents

**AC view** — header "AC charging" + state (`v.c.state`); SoC progress bar; Charger (delivered power
`v.c.power`, grid input `xte.v.c.gridpower`, V/A `v.c.voltage`/`v.c.current`); Battery (power/V/A/temp);
Session energy (charged `v.c.kwh`, from grid `v.c.kwh.grid`, cabin `xte.v.e.hvac.kwh`); chart
(delivered + car-permitted + SOC); state history.

**DC view** — header "DC fast charging" + live HLC state (`xte.v.c.hlc` → label); Station (max power
`xte.v.c.stamaxp`, max V/A `xte.v.c.stamaxv`/`xte.v.c.stamaxi`, present V/A `v.c.voltage`/`v.c.current`);
Car (permitted `xte.v.c.perm`, target A `xte.v.c.tgti`); Battery (power/V/A/temp); Session energy
(charged `v.c.kwh`, SoC); chart (delivered + station-max + car-permitted + SOC); state history.

**Idle view** — "No active charge session." note + link to the latest report (`/xte/reports`). No
chart/history.

## Error handling / edge cases

- **No active vehicle / wrong type:** `ActiveVehicle()` null or not e-TNGA → render the idle view.
- **Charts.js fails to load:** chart container shows nothing; metric panels and history still work
  (chart init is in the ajax `success` callback only).
- **Empty session buffer** (page opened just as a session starts): `INIT`/`EVT` are `[]`; chart and
  history start empty and accumulate live.
- **Charge type change while open:** handled by the reload-hint described above.
- **Signed/sentinel guards:** car-permitted uses magnitude; station-max is unsigned and 0 on AC
  (not plotted on AC). These mirror the report's already-validated handling.

## Validation

Build is verified on GitHub CI (no local toolchain). On-vehicle validation (next AC and DC charges):

- DC: HLC states roll into the chart's state history live and match the report; station-max +
  car-permitted traces render; power axis 0–150.
- AC: AC-Op states (`Startup/Running/Finishing`) roll into the state history live and appear in the
  report event log; grid power + efficiency shown; power axis 0–11.
- Opening the page mid-charge backfills the full curve and the prior state transitions.
- Idle view shown when parked, with a working link to the latest report.

## Out of scope / deferred

- Re-rendering the layout in place when the charge type changes (reload instead).
- Showing the previous session's chart in the idle view (link to the report only).
- Decoding/showing the secondary 0x1619 charger-op enum.
