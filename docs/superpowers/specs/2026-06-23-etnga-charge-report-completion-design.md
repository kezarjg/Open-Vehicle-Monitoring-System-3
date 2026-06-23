# e-TNGA charge-report completion (issue #81) — design

**Date:** 2026-06-23
**Issue:** [#81](https://github.com/kezarjg/Open-Vehicle-Monitoring-System-3/issues/81) — Toyota e-TNGA: charging session report generation (HTML, multi-phase, sleep-aware)
**Status:** design — pending user review
**Scope:** Complete the five remaining #81 checklist items as a sequence of five single-purpose PRs.

## 1. Background

PR #86 shipped the charge-session report (`etnga_charge_report.cpp`) and PR #122 moved its
file writes onto an async I/O worker (`etnga_charge_io.cpp`). The report already exceeds the
issue's "first increment" framing: a single `ChargeSessionState` tracks plug-in time, GPS,
ambient/battery temp ranges, peak power, coulomb-counted Ah (implied capacity), station kWh,
an event log (HLC / AC-Op transitions), and a 20 s-downsampled SVG chart; a 20-column
per-sample CSV is streamed to disk (~1 Hz, flushed ≤30 s / ≤4 KB) and linked from the report.
Reports + CSVs live at `/sd/charge-reports/` (or `/store/charge-reports/`), capped at the
newest 50, surfaced at the web pages `/xte/reports` (index) and `/xte/report` (viewer + CSV
download).

What remains from the #81 scope, and the dependency that orders it:

1. **Multi-phase `phases[]`** — the single `ChargeSessionState` only models one phase
   (`is_dc` is a scalar). Everything below attaches *per phase*, so this lands first.
2. **Limiting-side attribution** — per-phase "who capped the rate".
3. **Error DID-dump** — on a faulted phase.
4. **Sleep-persisted session state** — serialize the session (incl. `phases[]`) across SLEEP.
5. **Summary-mode report** — degraded report when the module slept through the charge.

This document specs all five and their sequencing. Each increment is its own single-purpose
PR (per `CLAUDE.md`), merged and on-vehicle-validated before the next begins. Several items are
**car-gated** for validation and MUST be built degradable so they ship inert until exercised.

## 2. Current hook points (grounding)

State transitions live in `etnga_poll_states.cpp`:

| Transition fn | Role for this work |
|---|---|
| `TransitionToChargeHandshakeState()` | **Session open** — `in_session=false→true`, resets energy counters, seeds GPS/ambient/start-SOC. |
| `TransitionToChargeAcState()` / `…DcState()` | **Phase open** (INC-1 hook). |
| `TransitionToChargeWaitState()` | **Phase close** (INC-1 hook). |
| `TransitionToAwakeState()` (`oldState >= CHARGE_HANDSHAKE`) | **Session close** → `GenerateChargeReport()` then `m_charge_session = {}`. |
| AWAKE-reconcile PISW=Unconnected path (`m_pisw_zero_count >= 2`) | Alternate **session close** → also calls `GenerateChargeReport()`. |
| `TransitionToSleepState()` | **Sleep-persist hook** (INC-4). |

Report rendering + CSV streaming + session struct live in `etnga_charge_report.cpp`; the
struct `ChargeSessionState` and PID enum are in `vehicle_toyota_etnga.h`. Async file ops go
through `ChargeIoEnqueue()` (`etnga_charge_io.cpp`).

## 3. INC-1 — Multi-phase tracking (foundation)

### 3.1 Data model

Refactor the single `ChargeSessionState` into a **session** that owns a **vector of phases**.
Session-level fields (one per plug-in→unplug) stay on the session; per-active-interval fields
move into `ChargePhase`.

```cpp
struct ChargePhase {
    bool   is_dc = false;
    int    start_monotonic = 0, end_monotonic = 0;
    time_t start_utc = 0;
    int    start_soc = -1, end_soc = -1;
    float  peak_power = 0.0f;          // kW into battery
    float  energy_kwh = 0.0f;          // battery-side, phase-local
    float  delivered_ah = 0.0f;        // coulomb count, phase-local
    bool   temp_seen = false; float temp_min = 0.0f, temp_max = 0.0f;
    int    outcome = -1;               // 0x1688 latched at phase close
    // INC-2 fills these; default-inert until then:
    int    limiting_side = 0;          // enum LimSide; 0 = unknown
    float  limiting_value = 0.0f;
    // INC-3 fills this; empty until then:
    std::vector<std::string> errors;
};

struct ChargeSessionState {
    bool   in_session = false;
    int    start_monotonic = 0; time_t start_utc = 0; int start_soc = -1;
    // session-level GPS + ambient (unchanged):
    bool   has_loc = false; float start_lat = 0, start_lon = 0;
    bool   amb_seen = false; float amb_min = 0, amb_max = 0;
    // session-level energy accounting (unchanged metrics: ms_v_charge_kwh, …, station_kwh):
    float  station_kwh = 0.0f;
    bool   report_written = false;     // idempotency (NEW; defensive)
    std::vector<std::pair<int,const char*>> events;   // session-level event log (unchanged)
    // CSV streaming state (unchanged; gains a phase index — see 3.3):
    std::string base; bool csv_started=false, csv_file_created=false;
    std::string csv_buf; int last_csv_flush=0;
    // SVG chart buffer (unchanged; whole-session, phase boundaries derived from phase times):
    struct Sample { int t_s; float kw; int soc; float sta_max; float car_perm; float station_kw; float hvac_kw; };
    std::vector<Sample> svg; int svg_interval_s = 20; int last_sample_monotonic = 0;
    std::vector<ChargePhase> phases;
    ChargePhase* cur = nullptr;        // points into phases.back() while a phase is active
};
```

Memory note: the per-phase scalars are tiny; the only large buffers (`svg`, `csv_buf`) stay
**session-level and bounded** exactly as today. Phases do NOT each hold a sample vector — the
full timeline stays in the single CSV (3.3). This is the RAM win of layout Option A.

### 3.2 Phase lifecycle

- **Open** a phase on entry to `CHARGE_AC` / `CHARGE_DC` *if no phase is currently open*
  (`cur == nullptr`): push a `ChargePhase`, set `is_dc`, `start_*`, point `cur` at it.
  Add a `LogChargeEvent("Phase N start (AC|DC)")`.
- **Close** the open phase on `TransitionToChargeWaitState()` *if `cur != nullptr`*: set
  `end_monotonic`, `end_soc`, latch `outcome` from `0x1688`, finalize `peak_power`/temp/energy
  deltas, then `cur = nullptr`. Event: `"Phase N complete"`.
- A second `CHARGE_AC`/`DC` entry (post-wait top-off, or a scheduled-wait→engage) opens a
  **new** phase → `phases.size()` grows. AC↔DC mid-session (rare; cable is fixed) simply closes
  one phase and opens another.
- Re-entering `CHARGE_AC`/`DC` while a phase is already open (`cur != nullptr`, e.g. a 1 s
  state flap) is a **no-op** — guards against double-open.
- Session close (`TransitionToAwakeState` / PISW-reconcile): if `cur != nullptr`, close it
  first (defensive — normal exits route through CHARGE_WAIT), then generate the report.

**Phase-local energy/peak/temp:** today these accumulate session-wide on `ChargeSessionState`.
They move to `cur`. Session totals are derived at report time by summing phases (energy) or
min/max-ing (temp). The existing standard metrics (`ms_v_charge_kwh`, `ms_v_bat_temp`, …) keep
their current per-session reset semantics — phase fields are computed by snapshotting at phase
open/close (`phase.energy = ms_v_charge_kwh@close − @open`), so we do not perturb the metric
reset logic that #105 fixed.

### 3.3 CSV — single file + phase column

Keep one CSV per session. Add **one leading column `phase`** (1-based; `0` while plugged but
not in a phase, i.e. WAIT/HANDSHAKE rows if any are written). New header:

```
phase,elapsed_s,soc_pct,bms_soc_pct,station_kw,battery_kw,hvac_kw,pack_v,pack_a,batt_temp_c,
ambient_c,state,station_max_kw,station_max_a,station_max_v,car_perm_kw,car_target_a,
station_grid_kw,station_present_v,station_present_a,obc_kw
```

`phase` = `cur ? (index+1) : 0`. The `state` column already distinguishes AC/DC; `phase` lets
a reader split the single CSV per phase without inferring boundaries.

### 3.4 Report layout (Option A — approved)

`GenerateChargeReport()` becomes:

1. **Summary `<dl>`** — plug-in/unplug/duration, location, ambient, **`Phases: N`**, total SOC
   delta, **summed** energy + grid + accounting, session battery-temp range, **telemetry
   coverage** (active-charging seconds vs total plugged seconds — sets up INC-4's gap line),
   overall outcome (last phase's, or worst).
2. **One session-level SVG** (`RenderPowerSvg`) spanning the whole charge, with **phase-boundary
   markers** (vertical lines at each phase start/end) and shaded gaps for non-charging spans.
3. **Per-phase `<h2>` + `<dl>`** block each: active interval, type, SOC delta, energy, peak/avg
   kW, limiting side (INC-2; omitted while unknown), battery-temp range, outcome, and (INC-3)
   an errors sublist. **No per-phase `<details>` sample tables.**
4. **Session events** table (unchanged).
5. **Estimates** (efficiency, implied capacity) — now summed across phases.
6. **Sleep-coverage gaps** list (populated by INC-4; empty/omitted until then).
7. **CSV download** link (unchanged).

Skip/idempotency: report is written once; skip entirely if `phases.empty()` (plugged then
unplugged, no charge) — replaces today's "no energy delivered" no-op. Set `report_written`.

### 3.5 INC-1 validation

- **Offline:** a 1-phase charge must produce a report substantially equivalent to today's
  (same summary numbers, same SVG, one phase block). CSV gains the `phase` column.
- **On-vehicle (Solterra):** (a) a normal single-phase AC or DC charge → 1 phase; (b) a
  scheduled-wait charge (wait → engage) and/or a top-off after full → **2 phases** with correct
  per-phase SOC/energy/peak and summed totals; CSV `phase` column increments correctly.

## 4. INC-2 — Limiting-side attribution

Per design doc §6.1. At **phase close**, classify who capped the rate and record
`limiting_side` + `limiting_value` on the `ChargePhase`; surface in the per-phase report block
and append to the real-time charge-complete event/notification.

```cpp
enum LimSide { LIM_UNKNOWN=0, LIM_STATION, LIM_CAR, LIM_CABLE, LIM_OBC, LIM_GRID, LIM_THERMAL };
```

**Candidate caps** (already-polled unless noted):
- `station`: `min(0x166A station_max_kw, 0x1679×0x1681 I×V)` (DC).
- `car`: `0x16A1` car-permission power (DC taper). Sub-attribution "cold battery" if battery
  temp `0x1829`/`0x182A` below a threshold at the cap point.
- `cable`: `0x1671 × line_voltage` (AC). **`0x1671` is NOT polled today — new poll** (AC states).
- `obc`: OBC AC max (model constant ~7.2 kW for Solterra).
- `grid`: inferred — AC drawn persistently below both cable and OBC caps.
- `thermal`: power-stage temps `0x1657`/`0x1658` or battery temps crossing derate thresholds
  **and** peak power fell mid-phase. **`0x1657`/`0x1658` new polls** (low-rate, charge states).

**Algorithm:** at phase close, take the phase's delivered peak; the smallest active cap is the
limiting side, with a `thermal` override if peak fell during the phase while a temperature
crossed a known derate threshold. All inputs are phase-tracked min/max/last values — no new
per-sample storage.

**Degradable:** if the required DIDs for a side never answered, that side is simply not a
candidate; `LIM_UNKNOWN` renders as an omitted "Limiting side" row (never a wrong guess).

**New polls:** `0x1671` (AC cable perm current), `0x1657` (PFC temp), `0x1658` (DC/DC temp) on
the OBC (`0x745`), in the AC/DC charge poll states. Thermal battery DIDs `0x1829`/`0x182A` —
check whether already polled; add if not.

**Validation (car-gated):** AC charge → expect `obc` (7.2 kW) or `cable`/`grid`; the L3 DC case
→ expect `car` with cold-battery sub-attribution (the 2026-05-10 canonical example: station
~90 kW, car 42.69 kW, delivered 40.54 kW → `car`).

## 5. INC-3 — Error DID-dump

Per design doc §6.4. When a phase faults (`error_in_phase` — detected via an abnormal `0x1688`
outcome or an `IncomingPollError` during the phase), perform a **one-shot burst of ~30
individual `0x22 XX YY` reads** in `CHARGE_WAIT` (a low-rate state — the serialized burst is a
few seconds and needs no slot management). Append decoded results to the faulted phase's
`errors[]`; log to the OVMS log; include in the report as a per-phase error sublist.

DID set (design doc §6.4 table): state-machine, trip-flag, connector/safety, electrical-at-
fault, thermal — ~30 DIDs across `0x16xx`/`0x18xx`/`0x10D4`.

**Mechanism:** a transient one-shot poll list appended on CHARGE_WAIT entry when
`error_in_phase` is set; results captured in `IncomingPollReply` into a scratch buffer keyed by
DID, formatted into `errors[]`. Cleared after dump.

**Degradable:** if a DID doesn't answer, record `"0xXXXX: no reply"` rather than blocking.

**Validation (car-gated):** requires a faulted charge — rare. Ship inert; validate
opportunistically. Unit-confirm the formatting against a synthetic fault if possible.

## 6. INC-4 — Sleep-persisted session state

Per design doc §6.5. On `TransitionToSleepState()` *when `in_session`* (sleep from CHARGE_WAIT,
or rarely AC/DC under bus-loss), serialize the session to
`/store/charge-session-state.json` via the async worker.

**Contents:** `in_charge_session`, `session_start_monotonic`, `session_start_soc`, the full
`phases[]` array (completed + any in-progress), `phase_active_at_sleep`, `armed_for_charge`,
and a `last_snapshot_at_sleep` block of SOC + key DIDs (`0x1688`, `0x16A0`, `0x1829`, `0x182A`,
lifetime AC counters from `0x1648`) + wall-clock ISO.

**Write throttle:** ≥60 s between writes; if sleep entry is <60 s since the last write, defer to
the next sleep entry (matches the §6.5 anti-thrash rule and the #118 sleep/wake oscillation
concern).

**On wake** (AWAKE resume): if the JSON exists, read the same DIDs, delta-compare to detect
"did a charge happen while asleep", and restore `phases[]` into `m_charge_session` so the
eventual report spans the whole session including slept spans. The **sleep-coverage gaps** list
(report §3.4 item 6) is computed from the wall-clock deltas between active spans.

**Clear:** delete the JSON on session end (after the report is written), and on a clean
non-charge wake that finds no charge happened.

**Serialization:** hand-rolled minimal JSON writer/parser (no cJSON dependency unless already
linked) — the schema is small and fixed. HTML/JSON-escape any string fields.

**Validation:** offline — round-trip serialize/deserialize a multi-phase session. On-vehicle —
the #118 overnight scheduled-charge scenario: module sleeps through the wait, wakes, and the
final report shows the pre-sleep phase(s) plus the gap list. (Wake delta-compare branching is
the car-gated part.)

## 7. INC-5 — Summary-mode report

Per design doc §6.3. Depends on INC-4 (knowing a charge happened while asleep) + the OBC counter
DIDs. When telemetry coverage for a phase is 0% (slept through it), render that phase block from
**computed values** instead of timeline-derived ones: duration from `0x1648` counter delta,
type from which counter incremented, energy from SOC-delta × pack capacity (or elapsed × avg),
temp range from `0x16A0` logged-during-charge max/min, outcome from `0x1688`. The phase block is
flagged `(summary mode, no live telemetry)`, omits any sample reference, and reports `0%`
coverage. Same single HTML document.

**New polls:** `0x1648` (AC lifetime minutes/count), `0x16A0` (max/min temp during charge) on
the OBC — read on wake during the delta-compare, and/or in CHARGE_WAIT.

**Pack capacity** for the energy estimate: use the variant-correct nominal (ties into the
pack-variant work, [[project_etnga_multivariant]]); 201.1 Ah / model nominal for the 96-cell
Solterra.

**Validation (most car-gated):** requires an actually-slept-through charge with the counters
read on both sides. Ship degradable; the non-summary path (INC-4 present, telemetry captured)
must be unaffected.

## 8. Sequencing & PR plan

| PR | Item | New polls | Validation gate |
|----|------|-----------|-----------------|
| 1 | INC-1 multi-phase + Option A report + CSV `phase` col | none | Solterra 1-phase ≈ today; 2-phase scheduled-wait/top-off |
| 2 | INC-2 limiting-side | `0x1671`, `0x1657`, `0x1658` (+ batt thermal if absent) | AC → obc/cable/grid; DC → car (cold-batt) |
| 3 | INC-3 error DID-dump | ~30 one-shot DIDs | opportunistic (faulted charge) |
| 4 | INC-4 sleep-persist JSON | snapshot DIDs incl. `0x1648`/`0x16A0` | #118 overnight scheduled charge |
| 5 | INC-5 summary-mode | (uses INC-4 counters) | slept-through charge |

INC-1 is a hard prerequisite for all others. INC-2 and INC-3 are independent given INC-1 and may
reorder. INC-5 requires INC-4. Each PR: `changes.txt` entry (new user-facing feature), single
vehicle scope, CI-green, on-vehicle validation recorded before the next starts. Car-gated items
(#3, #5, the DC half of #2) ship degradable and are validated opportunistically — the issue
stays open tracking the validation tail even after code merges, consistent with prior e-TNGA
work.

## 9. Risks / open points

- **RAM:** Option A keeps only the bounded `svg` + `csv_buf` buffers; `phases[]` scalars are
  negligible. No regression vs today. INC-4's `phases[]` JSON is small.
- **Metric reset interplay:** phase energy is snapshot-derived from `ms_v_charge_kwh` to avoid
  re-touching the #105 reset fix. Verify no double-reset on pause/resume.
- **`0x1671` AC cable DID** decode unconfirmed on Solterra — validate the scale before trusting
  the `cable` attribution; until then it can be a candidate only when plausibly bounded.
- **Error-trigger definition (INC-3):** what exactly sets `error_in_phase` (which `0x1688`
  codes, or any `IncomingPollError`) needs pinning in the INC-3 plan.
- **JSON library:** confirm whether a JSON writer is already linked; prefer hand-rolled minimal
  if not, to avoid a new dependency in a vehicle component.
- Several validations are genuinely car-gated; per repo norms unvalidated paths ship degradable
  and are not asserted correct until exercised on the vehicle.
