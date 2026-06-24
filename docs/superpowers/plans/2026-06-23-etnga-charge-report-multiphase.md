# e-TNGA charge report — multi-phase (INC-1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the single-phase e-TNGA charge-session report into a multi-phase model (plug-in → unplug may contain multiple AC/DC active phases), render the Option A layout (one session SVG + per-phase summaries), and add a `phase` column to the per-sample CSV.

**Architecture:** `phases[]` is **purely additive** — the existing session-level aggregates (peak/temp/Ah/station_kWh, SVG buffer, CSV stream) are left exactly as they are so the validated Summary block does not regress; a parallel `std::vector<ChargePhase>` is opened on each `CHARGE_AC`/`CHARGE_DC` entry and closed on `CHARGE_WAIT` (and defensively at report time). Per-phase energy is a snapshot delta of `ms_v_charge_kwh`; per-phase peak/temp/Ah are tracked in the per-tick aggregator. The report gains a `Phases: N` summary line, one `<dl>` block per phase, and phase-boundary markers on the existing SVG.

**Tech Stack:** C++ (ESP-IDF 3.3 / older GCC), OVMS standard metrics, the e-TNGA vehicle component (`components/vehicle_toyota_etnga/`). No new dependencies.

## Global Constraints

- **C++ for ESP-IDF 3.3 / older GCC.** Match the surrounding file's existing style exactly (`snprintf` into stack `char b[]`, `ESP_LOGx(TAG, …)`, no exceptions, no STL surprises beyond what the file already uses: `std::vector`, `std::string`, `std::ostringstream`).
- **No host unit-test suite.** This firmware compiles on GitHub CI, not locally; behavior is validated on-device. Per task, the gate is **(a) a code-inspection check** (the shown `grep`/reasoning) and **(b) the CI build**. Trigger CI for this branch with `gh workflow run build.yml --ref feature/etnga-charge-multiphase` (push trigger is master-only; feature branches build via manual dispatch or PR) and confirm green before merge. Do **not** propose a local `make` as the verification step.
- **`phases[]` is additive** — do not remove or repurpose the existing session-level `peak_power`, `temp_*`, `delivered_ah`, `station_kwh`, `svg`, or CSV fields. The Summary block keeps using them.
- **Pointer safety:** track the open phase with an **index** `int cur = -1`, never a `ChargePhase*` (a `push_back` realloc would dangle a pointer).
- **HTML-escape free text** before interpolation (outcome labels already routed through `ChargeOutcomeLabel`; no new free text in INC-1, but keep the rule).
- **Single-purpose PR / `changes.txt`:** this is one vehicle, one feature; add a `changes.txt` entry (new user-facing multi-phase report).
- **Work in the worktree** `/home/devuser/wt-etnga-charge-multiphase` on branch `feature/etnga-charge-multiphase`. All paths below are relative to `vehicle/OVMS.V3/components/vehicle_toyota_etnga/`.

---

## File Structure

| File | Responsibility | Change |
|---|---|---|
| `src/vehicle_toyota_etnga.h` | `ChargeSessionState` struct + method decls | Add `ChargePhase` struct, `phases`/`cur`/`report_written`; declare `OpenChargePhase(bool)`, `CloseChargePhase()`. |
| `src/etnga_poll_states.cpp` | State transitions | Call `OpenChargePhase(is_dc)` in AC/DC transitions, `CloseChargePhase()` in WAIT transition. |
| `src/etnga_charge_report.cpp` | Session stats, CSV, report render, SVG | Phase helpers; per-phase aggregation; CSV `phase` column; multi-phase report; SVG phase markers; report idempotency + defensive close. |
| `changes.txt` (repo: `vehicle/OVMS.V3/changes.txt`) | User-facing changelog | New entry. |

---

## Task 1: Data model — `ChargePhase` + session fields

**Files:**
- Modify: `src/vehicle_toyota_etnga.h:87-124` (the `ChargeSessionState` struct) and the method-declaration block near `:199`.

**Interfaces:**
- Produces: `struct ChargePhase` with fields used by all later tasks; `m_charge_session.phases` (`std::vector<ChargePhase>`), `m_charge_session.cur` (`int`, -1 = none), `m_charge_session.report_written` (`bool`); method decls `void OpenChargePhase(bool is_dc);` and `void CloseChargePhase();`.

- [ ] **Step 1: Add the `ChargePhase` struct + new session fields**

In `src/vehicle_toyota_etnga.h`, inside `struct ChargeSessionState`, immediately before the closing `};` at line 123, add:

```cpp
        // INC-1: per-phase tracking. phases[] is additive — the session-level
        // aggregates above stay authoritative for the Summary block.
        struct ChargePhase {
            bool   is_dc = false;
            int    start_monotonic = 0;
            time_t start_utc = 0;
            int    start_soc = -1;
            int    end_monotonic = 0;
            int    end_soc = -1;
            float  kwh_at_open = 0.0f;   // ms_v_charge_kwh snapshot at phase open
            float  energy_kwh = 0.0f;    // computed at close (delta)
            float  peak_power = 0.0f;    // kW into battery
            float  delivered_ah = 0.0f;  // phase-local coulomb count
            bool   temp_seen = false;
            float  temp_min = 0.0f;
            float  temp_max = 0.0f;
            int    outcome = -1;         // 0x1688 latched at close
        };
        std::vector<ChargePhase> phases;
        int   cur = -1;                  // index of the open phase in phases, -1 = none
        bool  report_written = false;    // idempotency guard for GenerateChargeReport
```

- [ ] **Step 2: Declare the phase helpers**

In `src/vehicle_toyota_etnga.h`, next to the existing `void GenerateChargeReport();` declaration (around line 199), add:

```cpp
    void OpenChargePhase(bool is_dc);   // INC-1: start a new charge phase on AC/DC entry
    void CloseChargePhase();            // INC-1: close the open phase on WAIT / report time
```

- [ ] **Step 3: Inspection check**

Run: `grep -n 'struct ChargePhase\|std::vector<ChargePhase>\|int   cur = -1\|OpenChargePhase\|CloseChargePhase' src/vehicle_toyota_etnga.h`
Expected: the struct, `phases`, `cur`, `report_written`, and both method decls are present. `cur` defaults to `-1`; `report_written` to `false`.

- [ ] **Step 4: Commit**

```bash
git add src/vehicle_toyota_etnga.h
git commit -m "etnga: add ChargePhase model + per-session phases[] (INC-1, #81)"
```

---

## Task 2: Phase lifecycle helpers + transition wiring

**Files:**
- Modify: `src/etnga_charge_report.cpp` (add the two helpers; place them next to `LogChargeEvent`, after line 153).
- Modify: `src/etnga_poll_states.cpp:493-527` (the three transition functions).

**Interfaces:**
- Consumes: `ChargePhase`, `m_charge_session.{phases,cur}` (Task 1).
- Produces: `OpenChargePhase(bool)` / `CloseChargePhase()` definitions; `phases` populated across a session.

- [ ] **Step 1: Implement the helpers**

In `src/etnga_charge_report.cpp`, after the `LogChargeEvent` function (ends line 153), add:

```cpp
// INC-1: open a new charge phase on CHARGE_AC / CHARGE_DC entry. No-op if a phase is
// already open (guards a 1 s state flap from double-opening). Snapshots the session
// kWh meter so the phase energy is a delta at close.
void OvmsVehicleToyotaETNGA::OpenChargePhase(bool is_dc)
{
    if (!m_charge_session.in_session || m_charge_session.cur >= 0)
        return;
    ChargeSessionState::ChargePhase ph;
    ph.is_dc          = is_dc;
    ph.start_monotonic = StandardMetrics.ms_m_monotonic->AsInt();
    ph.start_utc      = StandardMetrics.ms_m_timeutc->AsInt();
    ph.start_soc      = (int) StandardMetrics.ms_v_bat_soc->AsFloat();
    ph.kwh_at_open    = StandardMetrics.ms_v_charge_kwh->AsFloat();
    m_charge_session.phases.push_back(ph);
    m_charge_session.cur = (int) m_charge_session.phases.size() - 1;
    ESP_LOGD(TAG, "Charge phase %d open (%s)", m_charge_session.cur + 1, is_dc ? "DC" : "AC");
}

// INC-1: close the open phase on CHARGE_WAIT entry (or defensively at report time).
// No-op if no phase is open. Energy is the kWh-meter delta since open; outcome latches 0x1688.
void OvmsVehicleToyotaETNGA::CloseChargePhase()
{
    if (m_charge_session.cur < 0)
        return;
    ChargeSessionState::ChargePhase& ph = m_charge_session.phases[m_charge_session.cur];
    ph.end_monotonic = StandardMetrics.ms_m_monotonic->AsInt();
    ph.end_soc       = (int) StandardMetrics.ms_v_bat_soc->AsFloat();
    ph.energy_kwh    = StandardMetrics.ms_v_charge_kwh->AsFloat() - ph.kwh_at_open;
    if (ph.energy_kwh < 0.0f) ph.energy_kwh = 0.0f;
    ph.outcome       = m_v_charge_outcome->AsInt();
    ESP_LOGD(TAG, "Charge phase %d closed (%.2f kWh, %d%%->%d%%)",
             m_charge_session.cur + 1, ph.energy_kwh, ph.start_soc, ph.end_soc);
    m_charge_session.cur = -1;
}
```

- [ ] **Step 2: Open a phase on AC entry**

In `src/etnga_poll_states.cpp`, in `TransitionToChargeAcState()` (line 502), after the existing `LogChargeEvent("AC charging started");` (line 510), add:

```cpp
    OpenChargePhase(false);   // INC-1: AC phase
```

- [ ] **Step 3: Open a phase on DC entry**

In `TransitionToChargeDcState()` (line 513), after `LogChargeEvent("DC charging started");` (line 526), add:

```cpp
    OpenChargePhase(true);    // INC-1: DC phase
```

- [ ] **Step 4: Close the phase on WAIT entry**

In `TransitionToChargeWaitState()` (line 493), after `LogChargeEvent("Charging paused / phase ended");` (line 499), add:

```cpp
    CloseChargePhase();       // INC-1: close the active phase (pause / phase end)
```

- [ ] **Step 5: Inspection check**

Run: `grep -n 'OpenChargePhase\|CloseChargePhase' src/etnga_poll_states.cpp src/etnga_charge_report.cpp`
Expected: definitions in `etnga_charge_report.cpp`; `OpenChargePhase(false)` in the AC transition, `OpenChargePhase(true)` in the DC transition, `CloseChargePhase()` in the WAIT transition. Reason through one session: HANDSHAKE (no phase) → AC entry opens phase 0 → WAIT closes it (`cur=-1`) → AC re-entry opens phase 1. Two phases, correct.

- [ ] **Step 6: Commit**

```bash
git add src/etnga_charge_report.cpp src/etnga_poll_states.cpp
git commit -m "etnga: open/close charge phases on AC/DC<->WAIT transitions (INC-1, #81)"
```

---

## Task 3: Per-phase stat aggregation

**Files:**
- Modify: `src/etnga_charge_report.cpp:158-200` (`UpdateChargeSessionStats`).

**Interfaces:**
- Consumes: `m_charge_session.{cur,phases}`, `ChargePhase` fields (Tasks 1-2).
- Produces: per-phase `peak_power`, `temp_min/max`, `delivered_ah` filled live.

- [ ] **Step 1: Fold per-phase peak/temp into the aggregator**

In `UpdateChargeSessionStats()`, immediately after the existing session ambient block (after line 185, before the `m_charge_session.is_dc = …` line 187), add:

```cpp
    // INC-1: mirror peak power + battery-temp range into the open phase (additive to the
    // session-level aggregates above). cur < 0 between phases (WAIT) — skip then.
    if (m_charge_session.cur >= 0) {
        ChargeSessionState::ChargePhase& ph = m_charge_session.phases[m_charge_session.cur];
        if (p > ph.peak_power) ph.peak_power = p;
        if (!ph.temp_seen) { ph.temp_min = ph.temp_max = t; ph.temp_seen = true; }
        else if (t < ph.temp_min) ph.temp_min = t;
        else if (t > ph.temp_max) ph.temp_max = t;
    }
```

- [ ] **Step 2: Fold per-phase delivered-Ah into the dt block**

In the delivered-Ah block (lines 190-199), inside the `if (dt > 0 && dt <= 10)` body, after the existing `m_charge_session.delivered_ah += …` line (193), add:

```cpp
            if (m_charge_session.cur >= 0)
                m_charge_session.phases[m_charge_session.cur].delivered_ah +=
                    fabsf(StandardMetrics.ms_v_bat_current->AsFloat()) * (dt / 3600.0f);
```

- [ ] **Step 3: Inspection check**

Run: `grep -n 'phases\[m_charge_session.cur\]' src/etnga_charge_report.cpp`
Expected: three references in `UpdateChargeSessionStats` (peak, temp, Ah). Confirm `p` and `t` are already in scope at the insertion point (they are — defined at lines 165 and 169).

- [ ] **Step 4: Commit**

```bash
git add src/etnga_charge_report.cpp
git commit -m "etnga: aggregate peak/temp/Ah per charge phase (INC-1, #81)"
```

---

## Task 4: CSV `phase` column

**Files:**
- Modify: `src/etnga_charge_report.cpp:243-280` (`AppendChargeCsvRow`).

**Interfaces:**
- Consumes: `m_charge_session.cur` (Task 1).
- Produces: CSV header + rows lead with a `phase` column (1-based; `0` = no open phase).

- [ ] **Step 1: Add `phase` to the header**

In `AppendChargeCsvRow`, change the header string (line 244) from starting `"elapsed_s,soc_pct,…"` to lead with `phase,`:

```cpp
        m_charge_session.csv_buf =
            "phase,elapsed_s,soc_pct,bms_soc_pct,station_kw,battery_kw,hvac_kw,pack_v,pack_a,"
            "batt_temp_c,ambient_c,state,station_max_kw,station_max_a,station_max_v,"
            "car_perm_kw,car_target_a,station_grid_kw,station_present_v,station_present_a,obc_kw\n";
```

- [ ] **Step 2: Add the phase value to each row**

In the same function, compute the phase number before the `snprintf(row, …)` (before line 262):

```cpp
    int phase_no = (m_charge_session.cur >= 0) ? (m_charge_session.cur + 1) : 0;
```

Then change the row format string (line 263) to lead with `%d,` and pass `phase_no` first. The format becomes:

```cpp
    snprintf(row, sizeof(row),
        "%d,%d,%.0f,%.1f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%s,%s,"
        "%.2f,%.0f,%.0f,%.2f,%.0f,%.3f,%.0f,%.0f,%.3f\n",
        phase_no,
        elapsed,
```

(The remaining arguments are unchanged; `phase_no` is the new first arg, `elapsed` stays second.)

- [ ] **Step 3: Inspection check**

Run: `grep -n 'phase,elapsed_s\|int phase_no\|phase_no,' src/etnga_charge_report.cpp`
Expected: header leads with `phase,`; `phase_no` computed and passed as the first `snprintf` value. Count the format specifiers (21) against the argument count (21) — the leading `%d,` for `phase_no` plus the original 20.

- [ ] **Step 4: Commit**

```bash
git add src/etnga_charge_report.cpp
git commit -m "etnga: add phase column to charge CSV (INC-1, #81)"
```

---

## Task 5: Multi-phase report rendering + idempotency

**Files:**
- Modify: `src/etnga_charge_report.cpp` — `GenerateChargeReport` head (`:425-438`), Summary `<dl>` (`:472-533`), per-phase insertion after the SVG (`:536`), footer (`:570-571`).

**Interfaces:**
- Consumes: `m_charge_session.{phases,report_written}`, `CloseChargePhase()`, `ChargeOutcomeLabel` (existing), `ChargePhase` fields.
- Produces: report with `Phases: N`, one `<dl>` per phase, multi-phase footer; idempotent + defensive phase close.

- [ ] **Step 1: Idempotency + defensive close at report head**

In `GenerateChargeReport()`, at the very top (before line 427's `const float energy_kwh = …`), add:

```cpp
    if (m_charge_session.report_written)
        return;                 // defensive idempotency
    CloseChargePhase();         // close any still-open phase (direct AC/DC->AWAKE safety net)
```

- [ ] **Step 2: Add the phase count to the Summary**

In the Summary `<dl>`, after the existing `Type` line (the `f << "<dt>Type</dt>…"` at line 495), add a phase-count line:

```cpp
    {
        char pc[48];
        snprintf(pc, sizeof(pc), "%u", (unsigned) m_charge_session.phases.size());
        f << "<dt>Phases</dt><dd>" << pc << "</dd>\n";
    }
```

- [ ] **Step 3: Render per-phase blocks after the SVG**

In `GenerateChargeReport`, immediately after the `RenderPowerSvg(f);` line (536), insert the per-phase loop:

```cpp
    // INC-1: per-phase summary blocks (Option A — no per-phase sample tables; the full
    // timeline stays in the single CSV with its phase column).
    for (size_t i = 0; i < m_charge_session.phases.size(); i++) {
        const ChargeSessionState::ChargePhase& ph = m_charge_session.phases[i];
        int pdur = ph.end_monotonic - ph.start_monotonic; if (pdur < 0) pdur = 0;
        float pavg = (pdur > 0) ? ph.energy_kwh / (pdur / 3600.0f) : 0.0f;
        char pb[96];
        f << "<h2>Phase " << (i + 1) << " &mdash; " << (ph.is_dc ? "DC fast" : "AC") << "</h2>\n<dl>\n";
        if (ph.start_utc > 1000000000) {
            time_t st = (time_t) ph.start_utc, et = st + pdur; struct tm tmv;
            char ps[40], pe[40];
            gmtime_r(&st, &tmv); strftime(ps, sizeof(ps), "%H:%M:%S", &tmv);
            gmtime_r(&et, &tmv); strftime(pe, sizeof(pe), "%H:%M:%S", &tmv);
            snprintf(pb, sizeof(pb), "%s &rarr; %s UTC (%dh %02dm %02ds)", ps, pe,
                     pdur/3600, (pdur%3600)/60, pdur%60);
            f << "<dt>Active</dt><dd>" << pb << "</dd>\n";
        }
        snprintf(pb, sizeof(pb), "%d%% &rarr; %d%% (+%d%%)", ph.start_soc, ph.end_soc,
                 ph.start_soc >= 0 ? ph.end_soc - ph.start_soc : 0);
        f << "<dt>SOC</dt><dd>" << pb << "</dd>\n";
        snprintf(pb, sizeof(pb), "%.2f kWh; %.1f kW peak / %.2f kW avg",
                 ph.energy_kwh, ph.peak_power, pavg);
        f << "<dt>Energy</dt><dd>" << pb << "</dd>\n";
        if (ph.temp_seen) {
            metric_unit_t tu = OvmsMetricGetUserUnit(GrpTemp, Celcius);
            const char* tlabel = OvmsMetricUnitLabel(tu);
            float lo = UnitConvert(Celcius, tu, ph.temp_min), hi = UnitConvert(Celcius, tu, ph.temp_max);
            snprintf(pb, sizeof(pb), "%.0f%s &rarr; %.0f%s", lo, tlabel, hi, tlabel);
            f << "<dt>Battery temp</dt><dd>" << pb << "</dd>\n";
        }
        {
            const char* lbl = ChargeOutcomeLabel(ph.outcome);
            if (lbl[0]) f << "<dt>Outcome</dt><dd>" << lbl << "</dd>\n";
            else if (ph.outcome >= 0) {
                snprintf(pb, sizeof(pb), "0x%02X (raw)", ph.outcome & 0xFF);
                f << "<dt>Outcome</dt><dd>" << pb << "</dd>\n";
            }
        }
        f << "</dl>\n";
    }
```

- [ ] **Step 4: Update the footer text and set `report_written`**

Change the footer note (lines 570-571) from the "Single-phase" wording to multi-phase, and set the idempotency flag just before the job enqueue (before line 573's `etnga_io_job* job = new etnga_io_job;`):

```cpp
    f << "<p class=\"note\">Generated on-module by OVMS (Toyota e-TNGA). Multi-phase; per-phase "
         "avg power is over each active interval. Estimates are provisional.</p>\n</body></html>\n";
    m_charge_session.report_written = true;
```

- [ ] **Step 5: Inspection check**

Run: `grep -n 'report_written\|<dt>Phases</dt>\|Phase " << (i + 1)\|Multi-phase' src/etnga_charge_report.cpp`
Expected: idempotency guard + `CloseChargePhase()` at the head; `Phases` count in the Summary; the per-phase loop after `RenderPowerSvg`; `report_written = true` before enqueue; multi-phase footer. Confirm `ChargeOutcomeLabel`, `OvmsMetricGetUserUnit`, `UnitConvert`, `Celcius`, `GrpTemp` are already used elsewhere in the file (they are — Summary block) so no new includes are needed.

- [ ] **Step 6: Commit**

```bash
git add src/etnga_charge_report.cpp
git commit -m "etnga: render multi-phase charge report (Option A) + idempotency (INC-1, #81)"
```

---

## Task 6: SVG phase-boundary markers

**Files:**
- Modify: `src/etnga_charge_report.cpp:362-366` (inside `RenderPowerSvg`, after the axis lines, before the SOC overlay at line 367).

**Interfaces:**
- Consumes: `m_charge_session.{phases,start_monotonic}`, the SVG geometry locals `PADL/PADT/PW/PH/tmax/W/PADR/H/PADB` already defined in `RenderPowerSvg`.
- Produces: a faint vertical dashed line at each phase start/end on the chart.

- [ ] **Step 1: Draw boundary markers**

In `RenderPowerSvg`, after the three axis-line `snprintf`/`out` statements (line 365 is the bottom-time axis), before the `// SOC overlay` comment (line 367), add:

```cpp
    // INC-1: phase-boundary markers (vertical dashed lines at each phase start/end),
    // mapped from monotonic seconds onto the same time axis the traces use.
    for (size_t i = 0; i < m_charge_session.phases.size(); i++) {
        const ChargeSessionState::ChargePhase& ph = m_charge_session.phases[i];
        int bounds[2] = { ph.start_monotonic - m_charge_session.start_monotonic,
                          ph.end_monotonic   - m_charge_session.start_monotonic };
        for (int k = 0; k < 2; k++) {
            int ts = bounds[k];
            if (ts <= 0 || ts > tmax) continue;   // off-chart / unset end
            float x = PADL + (float)PW * ts / tmax;
            snprintf(b, sizeof(b),
                "<line x1=\"%.1f\" y1=\"%d\" x2=\"%.1f\" y2=\"%d\" stroke=\"#bbb\" "
                "stroke-width=\"0.7\" stroke-dasharray=\"2 2\"/>\n", x, PADT, x, H - PADB);
            out << b;
        }
    }
```

- [ ] **Step 2: Inspection check**

Run: `grep -n 'phase-boundary markers' src/etnga_charge_report.cpp`
Expected: the marker loop sits inside `RenderPowerSvg` after the axis lines and before the SOC polyline. Confirm `b`, `PADL`, `PW`, `tmax`, `PADT`, `H`, `PADB` are all in scope there (they are — declared earlier in the function).

- [ ] **Step 3: Commit**

```bash
git add src/etnga_charge_report.cpp
git commit -m "etnga: mark phase boundaries on the charge SVG (INC-1, #81)"
```

---

## Task 7: changes.txt + CI build + validation handoff

**Files:**
- Modify: `vehicle/OVMS.V3/changes.txt` (repo root-relative; the entry block at the top).

- [ ] **Step 1: Add the changes.txt entry**

At the top of `vehicle/OVMS.V3/changes.txt`, under a dated/author header in the existing format, add:

```
3.3.xxx  2026-06-23  Jerry Kezar
  - Toyota e-TNGA: charge-session report is now multi-phase — a single plug-in-to-unplug
    session that charges, pauses (scheduled wait), then tops off is reported as separate
    phases, each with its own SOC delta, energy, peak/avg power, temperature range and
    outcome. The power chart marks phase boundaries; the per-sample CSV gains a leading
    "phase" column. Single-phase charges are unchanged.
```

(Match the surrounding entry style — version line, two-space-indented `-` bullets. Use the next version number consistent with the existing top entry.)

- [ ] **Step 2: Commit**

```bash
git add vehicle/OVMS.V3/changes.txt
git commit -m "etnga: changes.txt for multi-phase charge report (INC-1, #81)"
```

- [ ] **Step 3: Trigger the CI build and confirm green**

Run: `gh workflow run build.yml --ref feature/etnga-charge-multiphase` then watch with `gh run list --branch feature/etnga-charge-multiphase --limit 1` / `gh run watch <id>`.
Expected: build succeeds (component `vehicle_toyota_etnga` compiles). If it fails, fix the reported file/line and amend the relevant task's commit before proceeding.

- [ ] **Step 4: On-vehicle validation checklist (record results in the PR / memory)**

This is the real gate — INC-1 is not "done" until validated on the Solterra. Verify, on `solterra-ovms`:
1. **Single-phase parity:** a normal AC (or DC) charge → report shows `Phases: 1`, one phase block whose SOC/energy/peak match the Summary; SVG unchanged bar the (absent, single-phase) boundary markers; CSV has the `phase` column = 1 throughout the active span.
2. **Multi-phase:** a scheduled-wait charge (wait → engage) or a top-off after full → `Phases: 2`, correct per-phase SOC/energy/peak, phase boundary lines on the SVG, CSV `phase` increments 1→(0 during WAIT)→2.
3. **No regression:** Summary totals (energy, grid, accounting, implied capacity) match pre-change behavior for a single-phase charge; no new poll errors/crashes.

---

## Self-Review

**Spec coverage (INC-1 section §3 of the design doc):**
- §3.1 data model (session owns `vector<ChargePhase>`, index not pointer) → Task 1. ✓ (additive variant: session aggregates retained per Global Constraints — a deliberate, documented deviation that lowers regression risk.)
- §3.2 phase lifecycle (open on AC/DC, close on WAIT, no double-open, defensive close at session end) → Tasks 2 + 5 Step 1. ✓
- §3.2 phase-local energy = `ms_v_charge_kwh` snapshot delta → Task 2 (`kwh_at_open`) + helper close. ✓
- §3.3 single CSV + `phase` column → Task 4. ✓
- §3.4 Option A report (Summary `Phases:N`, session SVG + boundary markers, per-phase `<dl>`, no sample tables, idempotency, skip when no charge) → Tasks 5 + 6. ✓ (skip-when-empty preserved via the existing `energy < 0.05` guard; per-phase limiting-side/errors rows are INC-2/INC-3, correctly absent here.)
- §3.5 validation (1-phase parity, 2-phase, no regression) → Task 7 Step 4. ✓

**Placeholder scan:** no TBD/TODO; every code step shows complete code; the `changes.txt` version number is the one intentional fill ("next version consistent with the top entry") — unavoidable without reading the live file, and bounded to a single token.

**Type consistency:** `OpenChargePhase(bool)` / `CloseChargePhase()` signatures match between header (Task 1) and definitions/call sites (Task 2). `ChargeSessionState::ChargePhase` is the nested type name used consistently in Tasks 2/3/5/6. `m_charge_session.cur` is `int` (-1 sentinel) everywhere; `phases` indexed by `cur` and by loop `i`. `report_written` set once (Task 5) and read once (Task 5 head). CSV format specifier/argument counts reconciled in Task 4 Step 3.
