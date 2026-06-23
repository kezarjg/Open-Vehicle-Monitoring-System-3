# e-TNGA charge report — limiting-side attribution (INC-2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Attribute *who capped the charge rate* per phase and surface it in the multi-phase charge report — scoped to the confidently-decodable **DC car-vs-station** case (with a cold-battery sub-attribution), deferring the speculative AC/cable/stage-thermal pieces.

**Architecture:** Stacks on INC-1 (multi-phase, branch `feature/etnga-charge-multiphase`). At phase close, classify the DC limiting side by comparing the station's advertised max (`0x166A`, already polled) against the car's minimum permission power (`0x16A1`, already polled, forward-filled) tracked across the phase; record `limiting_side`/`limiting_value`/`cold_battery` on the `ChargePhase` and render a "Limiting" line in the per-phase report block. Uses only measured, decoded DIDs — no magic constants.

**Tech Stack:** C++ (ESP-IDF 3.3 / older GCC), the e-TNGA vehicle component. No new polls, no new dependencies.

## Scope decision (read first)

The #81 design doc §4 / §6.1 lists six limiting sides (`station`, `car`, `cable`, `obc`, `grid`, `thermal`). Per the 2026-06-23 scoping decision ("build confident core, defer fringe"), this PR implements **only** the parts that use confirmed DIDs and decodes:

**IN (this PR):**
- **DC `car` vs `station`** — `0x166A` advertised station max vs `fabsf(0x16A1)` car permission. Both polled, both decoded, no constant.
- **Cold-battery sub-attribution** on a car-limited DC phase — from `temp_min` (battery temp, already tracked per phase in INC-1; decode known).

**DEFERRED (a later PR, needs unconfirmed data — do NOT implement here):**
- **AC `obc`** — needs the Solterra OBC AC-max constant (design doc says ~7.2 kW; unconfirmed; Solterra AC charger may be 6.6 kW). No magic constant in this PR.
- **AC `cable`** — the cable-permission DID is ambiguous in the RE repo (`0x1671` "AC rated current" vs `0x16A2` "cable permission current"); unconfirmed.
- **AC `grid`** — indistinguishable from `cable` without the cable DID.
- **`thermal` as a side** (mid-phase derate detection via `0x1657`/`0x1658` stage temps) — stage-temp decode unconfirmed.
- **Real-time `MyNotify` push** — etnga deliberately does NOT override charge-notify hooks (overriding `NotifyChargeStart` caused the #105 kWh-wipe). Surfacing here is via the report block + `ESP_LOGI`. A notify push is out of scope.

AC phases and any unclassifiable DC phase render **no** "Limiting" line (omitted, never a wrong guess). This is the degradable behavior the design doc mandates.

## Global Constraints

- **C++ for ESP-IDF 3.3 / older GCC.** Match the surrounding file's style (`snprintf` into stack `char b[]`, `ESP_LOGx(TAG, …)`, `fabsf`, no exceptions).
- **No host unit-test suite / no local build.** Per-task gate = code inspection + reasoning; the compile is verified by GitHub CI (`gh workflow run build.yml --ref feature/etnga-charge-limiting-side`). Do NOT propose a local `make`.
- **No new polls, no magic constants.** Classification uses only `m_v_charge_sta_max_p` (`0x166A`), `m_v_charge_perm` (`0x16A1`), and per-phase `temp_min`/`temp_seen` — all already present after INC-1. If a value was never seen during the phase, that candidate is simply absent.
- **Additive & degradable.** New `ChargePhase` fields default to "unknown"; an unclassified phase omits the Limiting line. Nothing in INC-1's behavior changes for phases that don't classify.
- **DC-only classification.** AC phases (`!is_dc`) always resolve to `LIM_UNKNOWN` in this PR.
- **Thresholds are named constants with provisional comments** — they need on-vehicle DC-charge tuning; say so in the comment.
- **Single-purpose PR / `changes.txt`** entry under the existing `????-??-?? ??? ??????? OTA release` pending block (do NOT invent a dated/versioned header).
- **Work in** `/home/devuser/wt-etnga-limiting-side` on branch `feature/etnga-charge-limiting-side` (stacked on `feature/etnga-charge-multiphase`). Paths below are relative to `vehicle/OVMS.V3/components/vehicle_toyota_etnga/`.

## Interfaces from INC-1 (already present — do not re-add)

- `ChargeSessionState::ChargePhase` with `is_dc`, `peak_power`, `temp_seen`, `temp_min`, `temp_max`, `start_soc`, `end_soc`, `energy_kwh`, `outcome`.
- `m_charge_session.cur` (`int`, -1 = no open phase); `m_charge_session.phases`.
- `void OpenChargePhase(bool)`, `void CloseChargePhase()` (in `etnga_charge_report.cpp`); `void UpdateChargeSessionStats()` (already mirrors per-phase peak/temp under an `if (m_charge_session.cur >= 0)` block).
- The per-phase render loop in `GenerateChargeReport()` (after `RenderPowerSvg(f)`), emitting `<h2>Phase i …</h2><dl>…</dl>` with an Energy line.
- Metrics `m_v_charge_sta_max_p` (`0x166A`, kW, DC), `m_v_charge_perm` (`0x16A1`, kW signed; magnitude via `fabsf` is the charge limit, forward-filled, 0x8000 inactive skipped by the poll processor).

## File Structure

| File | Responsibility | Change |
|---|---|---|
| `src/vehicle_toyota_etnga.h` | `ChargePhase` + decls | Add `LimSide` enum, `ChargePhase` fields (limiting_side/value/cold_battery + per-phase cap trackers), declare `ClassifyLimitingSide()` + `LimSideLabel()`. |
| `src/etnga_charge_report.cpp` | stats, classify, render | Track DC caps in `UpdateChargeSessionStats`; implement + call `ClassifyLimitingSide()` in `CloseChargePhase`; render Limiting line; `LimSideLabel()`. |
| `changes.txt` (`vehicle/OVMS.V3/changes.txt`) | changelog | New bullet under the pending block. |

---

## Task 1: Data model — `LimSide` enum + `ChargePhase` fields

**Files:**
- Modify: `src/vehicle_toyota_etnga.h` — the `ChargeSessionState::ChargePhase` struct (add fields) and the method-declaration block near `OpenChargePhase`/`CloseChargePhase`.

**Interfaces:**
- Produces: `enum LimSide { LIM_UNKNOWN=0, LIM_STATION, LIM_CAR };`; `ChargePhase` fields `int limiting_side`, `float limiting_value`, `bool cold_battery`, and per-phase cap trackers `bool cap_car_seen`, `float cap_car_min`, `bool cap_station_seen`, `float cap_station_max`; method decls `void ClassifyLimitingSide();` and `static const char* LimSideLabel(int side);`.

- [ ] **Step 1: Add the `LimSide` enum**

In `src/vehicle_toyota_etnga.h`, immediately before the `struct ChargeSessionState {` line, add:

```cpp
    // INC-2: charge-rate limiting side (who capped the rate). DC-only in this increment;
    // AC/cable/obc/grid/thermal are deferred (unconfirmed DIDs/constants).
    enum LimSide { LIM_UNKNOWN = 0, LIM_STATION, LIM_CAR };
```

- [ ] **Step 2: Add the per-phase fields**

In `struct ChargePhase` (inside `ChargeSessionState`), after the existing `int outcome = -1;` line, add:

```cpp
            // INC-2: limiting-side attribution (DC car-vs-station). Defaults = unknown/inert.
            int    limiting_side = 0;       // LimSide; 0 = unknown / not classified
            float  limiting_value = 0.0f;   // the binding cap (kW)
            bool   cold_battery = false;    // sub-attribution: car-limited while battery cold
            // per-phase cap trackers (DC), filled live in UpdateChargeSessionStats:
            bool   cap_car_seen = false;
            float  cap_car_min = 0.0f;      // min active fabsf(0x16A1) over the phase
            bool   cap_station_seen = false;
            float  cap_station_max = 0.0f;  // max 0x166A advertised station power over the phase
```

- [ ] **Step 3: Declare the helpers**

Next to the `void OpenChargePhase(bool is_dc);` / `void CloseChargePhase();` declarations, add:

```cpp
    void ClassifyLimitingSide();                        // INC-2: set the open phase's limiting side at close
    static const char* LimSideLabel(int side);          // INC-2: LimSide enum -> human text
```

- [ ] **Step 4: Inspection check**

Run: `grep -n 'enum LimSide\|limiting_side\|cap_car_min\|cap_station_max\|ClassifyLimitingSide\|LimSideLabel' src/vehicle_toyota_etnga.h`
Expected: enum with three values; the six new `ChargePhase` fields; both decls. No existing field altered.

- [ ] **Step 5: Commit**

```bash
git add src/vehicle_toyota_etnga.h
git commit -m "etnga: add LimSide model for charge limiting-side (INC-2, #81)"
```

---

## Task 2: Track DC caps per phase

**Files:**
- Modify: `src/etnga_charge_report.cpp` — `UpdateChargeSessionStats`, inside the existing `if (m_charge_session.cur >= 0) { … }` per-phase block (the one that mirrors peak/temp).

**Interfaces:**
- Consumes: `m_charge_session.cur`, `ChargePhase` cap fields (Task 1), `m_v_charge_sta_max_p`, `m_v_charge_perm`, `ph.is_dc`.
- Produces: `cap_car_min`/`cap_station_max` (+seen flags) filled live during DC phases.

- [ ] **Step 1: Add DC cap tracking to the per-phase block**

In `UpdateChargeSessionStats`, inside the existing `if (m_charge_session.cur >= 0) { ChargeSessionState::ChargePhase& ph = …; … }` block (added in INC-1 for peak/temp), after the temp-tracking lines and before the block's closing `}`, add:

```cpp
        // INC-2: track the DC limiting caps. Station advertised max (0x166A) is DC-only and
        // reads ~0 on AC; car permission (0x16A1) is forward-filled (inactive sentinel skipped
        // upstream). Only meaningful while is_dc — AC classification is deferred.
        if (ph.is_dc) {
            float sta = m_v_charge_sta_max_p->AsFloat();        // 0x166A advertised station max kW
            if (sta > 0.1f) {
                if (!ph.cap_station_seen || sta > ph.cap_station_max) ph.cap_station_max = sta;
                ph.cap_station_seen = true;
            }
            float car = fabsf(m_v_charge_perm->AsFloat());      // 0x16A1 magnitude = car charge limit kW
            if (car > 0.1f) {
                if (!ph.cap_car_seen || car < ph.cap_car_min) ph.cap_car_min = car;
                ph.cap_car_seen = true;
            }
        }
```

- [ ] **Step 2: Inspection check**

Run: `grep -n 'cap_station_max\|cap_car_min\|0x166A advertised' src/etnga_charge_report.cpp`
Expected: the DC cap-tracking block sits inside the `cur >= 0` per-phase block, guarded by `ph.is_dc`. Confirm `m_v_charge_sta_max_p` and `m_v_charge_perm` are member metrics (they are) and `fabsf` is already used in this file (it is, in `RenderPowerSvg`).

- [ ] **Step 3: Commit**

```bash
git add src/etnga_charge_report.cpp
git commit -m "etnga: track DC station/car caps per phase (INC-2, #81)"
```

---

## Task 3: Classify at phase close

**Files:**
- Modify: `src/etnga_charge_report.cpp` — add `ClassifyLimitingSide()` + `LimSideLabel()` near the phase helpers; call `ClassifyLimitingSide()` from `CloseChargePhase()` before `cur` is reset.

**Interfaces:**
- Consumes: `ChargePhase` cap fields + `temp_seen`/`temp_min` (Tasks 1-2).
- Produces: `ClassifyLimitingSide()` sets `limiting_side`/`limiting_value`/`cold_battery` on the open phase; `LimSideLabel(int)` returns `"station"`/`"car"`/`""`.

- [ ] **Step 1: Implement `LimSideLabel`**

In `src/etnga_charge_report.cpp`, near the other small helpers (e.g. next to `ChargeOutcomeLabel` if present in this file, else after `CloseChargePhase`), add:

```cpp
// INC-2: LimSide enum -> short human label. Empty string for unknown (caller omits the row).
const char* OvmsVehicleToyotaETNGA::LimSideLabel(int side)
{
    switch (side) {
        case ChargeSessionState::LIM_STATION: return "station";
        case ChargeSessionState::LIM_CAR:     return "car";
        default:                              return "";
    }
}
```

(If `LimSide` is scoped inside `ChargeSessionState`, reference it as `ChargeSessionState::LIM_STATION`. If Task 1 placed the enum at class scope instead, drop the `ChargeSessionState::` qualifier to match. Use whichever matches the header you wrote in Task 1.)

- [ ] **Step 2: Implement `ClassifyLimitingSide`**

Add, near `CloseChargePhase`:

```cpp
// INC-2: classify who capped the DC charge rate for the open phase. DC-only: compares the
// station's advertised max (0x166A) against the car's min permission (0x16A1). The smaller is
// the binding side. AC phases and phases with no caps seen stay LIM_UNKNOWN (report omits the
// row). Thresholds are provisional pending on-vehicle DC-charge tuning.
void OvmsVehicleToyotaETNGA::ClassifyLimitingSide()
{
    if (m_charge_session.cur < 0)
        return;
    ChargeSessionState::ChargePhase& ph = m_charge_session.phases[m_charge_session.cur];
    if (!ph.is_dc)
        return;   // AC attribution deferred (needs unconfirmed OBC-max / cable DID)

    const float LIM_MARGIN_KW = 2.0f;    // provisional: ignore near-equal caps as noise
    const float COLD_BATT_C   = 25.0f;   // provisional: car-limit below this reads as cold-battery derate

    if (ph.cap_car_seen && ph.cap_station_seen) {
        if (ph.cap_car_min < ph.cap_station_max - LIM_MARGIN_KW) {
            ph.limiting_side  = ChargeSessionState::LIM_CAR;
            ph.limiting_value = ph.cap_car_min;
            ph.cold_battery   = (ph.temp_seen && ph.temp_min < COLD_BATT_C);
        } else {
            ph.limiting_side  = ChargeSessionState::LIM_STATION;
            ph.limiting_value = ph.cap_station_max;
        }
    } else if (ph.cap_station_seen) {
        ph.limiting_side  = ChargeSessionState::LIM_STATION;   // station cap seen, car never bound
        ph.limiting_value = ph.cap_station_max;
    } else if (ph.cap_car_seen) {
        ph.limiting_side  = ChargeSessionState::LIM_CAR;       // car cap seen, station unknown
        ph.limiting_value = ph.cap_car_min;
        ph.cold_battery   = (ph.temp_seen && ph.temp_min < COLD_BATT_C);
    }
    // else: neither seen -> remains LIM_UNKNOWN

    if (ph.limiting_side != ChargeSessionState::LIM_UNKNOWN)
        ESP_LOGI(TAG, "Phase %d limited by %s (%.1f kW)%s", m_charge_session.cur + 1,
                 LimSideLabel(ph.limiting_side), ph.limiting_value,
                 ph.cold_battery ? " [cold battery]" : "");
}
```

- [ ] **Step 3: Call it from `CloseChargePhase`**

In `CloseChargePhase()`, after the line that latches `ph.outcome = m_v_charge_outcome->AsInt();` and BEFORE `m_charge_session.cur = -1;`, add:

```cpp
    ClassifyLimitingSide();   // INC-2: attribute the limiting side while cur still points at this phase
```

- [ ] **Step 4: Inspection check**

Run: `grep -n 'ClassifyLimitingSide\|LimSideLabel\|LIM_MARGIN_KW\|COLD_BATT_C' src/etnga_charge_report.cpp`
Expected: both helpers defined; `ClassifyLimitingSide()` called inside `CloseChargePhase` while `cur >= 0` (before the reset). Reason the canonical L3 case: `cap_car_min ≈ 42.7`, `cap_station_max ≈ 90` → `42.7 < 90 - 2` → `LIM_CAR`, value 42.7, cold if temp_min < 25 °C. A normal station-limited DC charge (`car` never below station) → `LIM_STATION`.

- [ ] **Step 5: Commit**

```bash
git add src/etnga_charge_report.cpp
git commit -m "etnga: classify DC limiting side at phase close (INC-2, #81)"
```

---

## Task 4: Render the Limiting line in the per-phase block

**Files:**
- Modify: `src/etnga_charge_report.cpp` — `GenerateChargeReport()`, the per-phase render loop, after the Energy line and before the Battery-temp/Outcome lines.

**Interfaces:**
- Consumes: `ph.limiting_side`/`limiting_value`/`cold_battery`, `LimSideLabel()`.
- Produces: a `<dt>Limiting</dt><dd>…</dd>` row, omitted when `LIM_UNKNOWN`.

- [ ] **Step 1: Add the Limiting row**

In the per-phase loop in `GenerateChargeReport`, after the `<dt>Energy</dt>` block and before the `if (ph.temp_seen)` battery-temp block, add:

```cpp
        if (ph.limiting_side != ChargeSessionState::LIM_UNKNOWN) {
            snprintf(pb, sizeof(pb), "%s &mdash; %.1f kW%s", LimSideLabel(ph.limiting_side),
                     ph.limiting_value, ph.cold_battery ? " (cold battery)" : "");
            f << "<dt>Limiting</dt><dd>" << pb << "</dd>\n";
        }
```

(`pb` is the per-phase `char pb[96]` already declared in the loop body in INC-1.)

- [ ] **Step 2: Inspection check**

Run: `grep -n '<dt>Limiting</dt>' src/etnga_charge_report.cpp`
Expected: one Limiting row inside the per-phase loop, guarded by `!= LIM_UNKNOWN`, using `LimSideLabel` + `limiting_value` (+ cold-battery suffix). AC/unclassified phases omit it. Confirm `pb` is in scope (declared in the loop in INC-1).

- [ ] **Step 3: Commit**

```bash
git add src/etnga_charge_report.cpp
git commit -m "etnga: show DC limiting side in per-phase report block (INC-2, #81)"
```

---

## Task 5: changes.txt + CI build + validation handoff

**Files:**
- Modify: `vehicle/OVMS.V3/changes.txt` — a bullet under the existing `????-??-?? ???  ???????  OTA release` pending block (NOT a new dated header).

- [ ] **Step 1: Add the changes.txt bullet**

Read the current top of `vehicle/OVMS.V3/changes.txt`. Under the existing `????-??-?? ???  ???????  OTA release` placeholder header (alongside the INC-1 multi-phase bullet), add a new `- ` bullet, 4-space continuation indent, matching the neighbors:

```
- Toyota e-TNGA (Subaru Solterra / Toyota bZ4X): the charge report now attributes the DC
    charge-rate limit per phase — whether the car (battery taper, flagged "cold battery" when
    the pack was cold) or the station capped the rate, with the binding kW shown. AC phases are
    not yet attributed.
```

- [ ] **Step 2: Commit**

```bash
git add vehicle/OVMS.V3/changes.txt
git commit -m "etnga: changes.txt for DC limiting-side attribution (INC-2, #81)"
```

- [ ] **Step 3: CI build**

Run: `gh workflow run build.yml --ref feature/etnga-charge-limiting-side`, then watch with `gh run watch <id> --exit-status`.
Expected: green. Fix any compile error and amend the relevant task commit before proceeding.

- [ ] **Step 4: On-vehicle validation checklist (record in PR / memory)**

The real gate — INC-2 is not "done" until validated on a real DC charge:
1. **DC car-limited (canonical):** a DC fast charge where the car tapers below station capability (cold battery or high SOC) → the phase block shows `Limiting: car — <kW>` with `(cold battery)` when the pack was cold; the kW ≈ the observed `0x16A1` plateau.
2. **DC station-limited:** a DC charge on a low-power stall (station max < car permission) → `Limiting: station — <kW>` ≈ the station's advertised `0x166A`.
3. **AC:** AC phases show NO Limiting line (deferred), and INC-1's other report content is unchanged.
4. Tune `LIM_MARGIN_KW` / `COLD_BATT_C` against the observed values if the labels misfire.

---

## Self-Review

**Spec coverage (design doc §4 / §6.1, scoped):**
- DC `car` vs `station` attribution → Tasks 2-3. ✓
- Cold-battery sub-attribution → Task 3 (`COLD_BATT_C`). ✓
- Surface in per-phase report block → Task 4. ✓ (real-time `MyNotify` push intentionally deferred — etnga avoids notify overrides; `ESP_LOGI` provides the live trace.)
- Degradable: AC + unclassified phases omit the row → Tasks 3-4 (`LIM_UNKNOWN`). ✓
- `cable`/`obc`/`grid`/`thermal` sides → explicitly DEFERRED (Scope decision), not implemented. ✓ (documented, not a silent gap.)

**Placeholder scan:** no TBD/TODO; every step has complete code. The two thresholds are real named constants with provisional-tuning comments (intended, not placeholders). The changes.txt version is governed by the pending-block convention (no number to invent).

**Type consistency:** `LimSide` values (`LIM_UNKNOWN`/`LIM_STATION`/`LIM_CAR`) used identically in header, classifier, label, and render. `ClassifyLimitingSide()`/`LimSideLabel(int)` signatures match between header (Task 1) and definitions (Task 3). `cap_car_min`/`cap_station_max` + seen flags written in Task 2, read in Task 3. `limiting_side`/`limiting_value`/`cold_battery` written in Task 3, read in Task 4. `pb[96]` reused from the INC-1 loop. Enum-scope qualifier (`ChargeSessionState::` or class-scope) reconciled in Task 3 Step 1's note against Task 1's placement.
