# e-TNGA fault-dump hardening + human-readable decode (INC-3 follow-up) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task.

**Goal:** Make the charge-fault diagnostic DID dump reliable and useful: fix its delivery (don't lose it on low-energy faults; beat the async race) and labeling (#139), and render **human-readable decoded values** beside the raw hex in the report.

**Architecture:** Continues PR #137 (branch `feature/etnga-charge-error-dump`, now based on merged master). The dump still captures raw bytes via the OnceOffPoll burst; this adds (a) latching the *triggering* fault outcome/phase, (b) deferring report generation until the dump completes so the report contains it, (c) generating a report even on a sub-0.05 kWh faulted session, and (d) a confident-subset decode rendered alongside the raw hex.

**Tech Stack:** C++ (ESP-IDF 3.3 / older GCC), the e-TNGA vehicle component. No new polls.

## Global Constraints

- **C++ for ESP-IDF 3.3 / older GCC.** Match surrounding style. No host tests; CI compiles (`gh workflow run build.yml --ref feature/etnga-charge-error-dump`). Do NOT propose local `make`.
- **Threading discipline (unchanged from #137):** dump callbacks run on the **poller task** and touch only `m_dump_results` (under `m_dump_mutex`) + `m_dump_remaining` (atomic). Report generation, the deferral flag, and `phases[]` are **Events-task only**. Do NOT generate the report or touch `phases[]` from a poller callback.
- **Decode confidence:** only decode DIDs with a confident decode (existing firmware decoders or known temp formulas — see Task 4). Uncertain DIDs render raw-only (decoded column blank). Do NOT invent decodes.
- **Raw is always kept** — the decoded value is shown *in addition to* the raw hex, never replacing it.
- **Single-purpose continuation of #137.** Work in `/home/devuser/wt-etnga-error-dump` on `feature/etnga-charge-error-dump`. Paths relative to `vehicle/OVMS.V3/components/vehicle_toyota_etnga/`.

## Interfaces already in place (INC-3, #137)
- `m_charge_fault_pending` (atomic<bool>), `m_dump_remaining` (atomic<int>), `m_dump_results` (map<uint16_t,string>), `m_dump_mutex`, `m_dump_phase_idx`, `m_dump_outcome`.
- `IsChargeFaultCode(int)`, `MaybeStartChargeFaultDump()`, `IncomingDumpSuccess/Fail(...)` in `etnga_charge_dump.cpp`.
- The `0x1688` handler in `etnga_poll_processor.cpp` sets `m_charge_fault_pending` via `IsChargeFaultCode(outcome_code)`.
- `GenerateChargeReport()` in `etnga_charge_report.cpp` (skip when `energy < 0.05`; renders the dump table under `m_dump_mutex`; clears dump state at report end; session-open clear in `TransitionToChargeHandshakeState`).
- Existing decoders: `ChargeOutcomeLabel(int)`, `HlcStateLabel(int)`, `AcOpStatusLabel(int)`; `CalculateBatteryChargingPower`, `CalculatePISWRaw`, station present V/A calculators.

---

## Task 1: Latch the triggering outcome + phase (labeling fix, #139.2)

**Files:** `src/vehicle_toyota_etnga.h` (members), `src/etnga_poll_processor.cpp` (the `0x1688` handler), `src/etnga_charge_dump.cpp` (`MaybeStartChargeFaultDump`).

- [ ] **Step 1:** Add members in the header near the dump state: `int m_dump_trigger_outcome = -1;` and `int m_dump_trigger_phase = -1;`.
- [ ] **Step 2:** In `etnga_poll_processor.cpp`, where `IsChargeFaultCode(outcome_code)` sets `m_charge_fault_pending = true`, also latch the trigger context at that moment:
```cpp
            if (IsChargeFaultCode(outcome_code)) {
                m_charge_fault_pending = true;
                m_dump_trigger_outcome = outcome_code;                          // the code that IS the fault
                m_dump_trigger_phase   = (m_charge_session.cur >= 0)
                                         ? m_charge_session.cur
                                         : (int) m_charge_session.phases.size() - 1;  // phase whose fault flagged
            }
```
- [ ] **Step 3:** In `MaybeStartChargeFaultDump()`, REPLACE the `m_dump_outcome = m_v_charge_outcome->AsInt();` / `m_dump_phase_idx = phases.size()-1` lines with the latched values:
```cpp
    m_dump_outcome   = m_dump_trigger_outcome;                       // the triggering fault code, not current
    m_dump_phase_idx = (m_dump_trigger_phase >= 0) ? m_dump_trigger_phase
                                                   : (int) m_charge_session.phases.size() - 1;
```
- [ ] **Step 4:** Inspection: the dump label now reflects the *fault* code. Reason the 2026-06-24 case: trigger `0x29` in Phase 1 → label "Phase 1 fault (outcome 0x29 …)", not "Phase 2 fault (0x26)".
- [ ] **Step 5:** Commit `etnga: latch triggering fault outcome+phase for the dump (INC-3, #139)`.

---

## Task 2: Generate a report even on a sub-0.05 kWh faulted session (delivery fix, #139.1a)

**Files:** `src/etnga_charge_report.cpp` (`GenerateChargeReport` skip guard).

- [ ] **Step 1:** Change the `energy < 0.05f` skip so it does NOT skip when a fault dump exists/was triggered. Locate the skip guard and add a dump condition:
```cpp
    bool have_dump = (m_dump_phase_idx >= 0) || (m_dump_remaining.load() > 0);
    if ((energy_kwh < 0.05f || m_charge_session.base.empty()) && !have_dump) {
        // ... existing skip (stub-CSV cleanup) ...
        return;
    }
```
(So a faulted session still produces a report carrying the diagnostic dump, even with ~no energy.)
- [ ] **Step 2:** Inspection: a fault with <0.05 kWh now renders a report; a normal <0.05 kWh plug-blip still skips.
- [ ] **Step 3:** Commit `etnga: don't skip the charge report when a fault dump exists (INC-3, #139)`.

---

## Task 3: Defer report generation until the dump completes (delivery fix, #139.1b — Events-task only)

**Files:** `src/vehicle_toyota_etnga.h` (members), `src/etnga_poll_states.cpp` (`TransitionToAwakeState`, the AWAKE PISW-reconcile in `HandleAwakeState`, and a check in `Ticker1`/`HandleAwakeState`).

**Design:** Report generation stays 100% on the Events task. At session close, if a dump is in flight, set a pending flag and DEFER both the report and the session reset; a per-tick check (Events task) finalizes once the dump completes or a timeout elapses. No cross-task report generation.

- [ ] **Step 1:** Header members: `bool m_report_pending = false;` and `int m_report_pending_deadline = 0;` and a `static const int DUMP_WAIT_SECS = 10;`.
- [ ] **Step 2:** Add a private helper `void FinalizeChargeSession();` (Events task) implemented in `etnga_charge_report.cpp`:
```cpp
// Events-task: write the report and reset the session. Used both for the immediate close
// path and the deferred (wait-for-dump) path.
void OvmsVehicleToyotaETNGA::FinalizeChargeSession()
{
    GenerateChargeReport();
    m_charge_session = ChargeSessionState{};
}
```
- [ ] **Step 3:** In `TransitionToAwakeState()` where it currently does `GenerateChargeReport(); m_charge_session = ChargeSessionState{};` (the `in_session` block), replace with a defer-or-finalize:
```cpp
        if (m_charge_session.in_session) {
            ESP_LOGI(TAG, "Charge session closed");
            LogChargeEvent("Unplugged");
            if (m_dump_remaining.load() > 0) {       // a fault dump is still capturing — wait for it
                m_report_pending = true;
                m_report_pending_deadline = monotonic + DUMP_WAIT_SECS;
                ESP_LOGI(TAG, "Report deferred — waiting for fault dump (%d DIDs left)", m_dump_remaining.load());
            } else {
                FinalizeChargeSession();
            }
        } else {
            m_charge_session = ChargeSessionState{};   // not in session — nothing to finalize
        }
        m_charge_wait_slept = false;
```
(Note: the old code reset `m_charge_session` unconditionally after the report; now the reset happens inside `FinalizeChargeSession`, or is deferred. Make sure the non-`in_session` path still clears as before.)
- [ ] **Step 4:** Apply the SAME defer logic to the AWAKE PISW-reconcile close path in `HandleAwakeState` (the `in_session && m_pisw_zero_count >= 2` block): if `m_dump_remaining > 0`, set `m_report_pending`/deadline instead of calling `GenerateChargeReport()` + reset.
- [ ] **Step 5:** Add a deferral-completion check at the TOP of `HandleAwakeState` (Events task, runs each tick), BEFORE the PISW-reconcile, guarded so it owns the close while pending:
```cpp
    if (m_report_pending) {
        if (m_dump_remaining.load() <= 0 || StandardMetrics.ms_m_monotonic->AsInt() >= m_report_pending_deadline) {
            ESP_LOGI(TAG, "Fault dump done (or timed out) — finalizing deferred report");
            FinalizeChargeSession();
            m_report_pending = false;
        }
        return;   // hold normal AWAKE handling (incl. the PISW-reconcile) until the deferred report is written
    }
```
- [ ] **Step 6:** Inspection/reasoning: trace a faulted unplug — session close sees `m_dump_remaining > 0` → defers; ~3-6 s later the dump completes → the per-tick check finalizes (report now contains the full dump); on a hung DID, the 10 s deadline finalizes anyway. Confirm the `return` prevents the PISW-reconcile from double-finalizing while pending, and that `m_report_pending` is cleared on finalize. Confirm `FinalizeChargeSession` resets `m_charge_session` (so `report_pending`'s view of the session stays valid until then).
- [ ] **Step 7:** Commit `etnga: defer charge report until fault dump completes (INC-3, #139)`.

---

## Task 4: Human-readable decode of the confident DID subset

**Files:** `src/etnga_charge_dump.cpp` (a decode function), `src/vehicle_toyota_etnga.h` (decl).

- [ ] **Step 1:** Declare `std::string DumpDidDecode(uint16_t did, const std::string& raw);` (member).
- [ ] **Step 2:** Implement in `etnga_charge_dump.cpp` — return a human-readable string for the **confident subset**, else `""` (caller shows raw only). Use `uint8_t` access to `raw` bytes (guard length). Confident decodes:
```cpp
// Human-readable decode for the confident DID subset. "" = no confident decode (raw-only).
std::string OvmsVehicleToyotaETNGA::DumpDidDecode(uint16_t did, const std::string& raw)
{
    auto u8  = [&](size_t i)->int { return (i < raw.size()) ? (uint8_t)raw[i] : -1; };
    auto u16 = [&](size_t i)->int { return (i+1 < raw.size()) ? (((uint8_t)raw[i]<<8)|(uint8_t)raw[i+1]) : -1; };
    char b[64];
    switch (did) {
        case 0x1688: { const char* l = ChargeOutcomeLabel(u8(0)); return l[0] ? l : ""; }
        case 0x1666: { const char* l = HlcStateLabel(u8(0));     return l[0] ? l : ""; }
        case 0x1684: { const char* l = AcOpStatusLabel(u8(0));   return l[0] ? l : ""; }
        case 0x10D4: { int v=u16(0); if(v<0)return ""; snprintf(b,sizeof(b),"%.2f kW", (v-0x8000)*0.01f); return b; }
        case 0x1829: case 0x182A: { int v=u16(0); if(v<0)return ""; snprintf(b,sizeof(b),"%.1f °C", v/256.0f-50.0f); return b; }  // batt max/min temp, Q8.8 -50
        case 0x1657: case 0x1658: { int v=u8(0);  if(v<0)return ""; snprintf(b,sizeof(b),"%d °C", v-50); return b; }              // PFC / DC-DC temp, u8 -50
        case 0x1669: { int v=u8(0); if(v<0)return ""; snprintf(b,sizeof(b),"PISW 0x%02X", v); return b; }
        default: return "";   // uncertain — raw only
    }
}
```
(Temp formulas confirmed against the live dump: `0x1829`=`45 00` -> 19.0 °C, `0x182A`=`44 00` -> 18.0 °C, matching the 66 °F battery; `0x1657`=`54` -> 34 °C. If `AcOpStatusLabel`/`HlcStateLabel` are `static`, the unqualified call from this member is fine. Verify each label fn exists with that exact name and a `const char*` return.)
- [ ] **Step 3:** Inspection: each `case` matches an existing decoder/known formula; default returns `""`; length-guarded.
- [ ] **Step 4:** Commit `etnga: decode confident DIDs to human-readable for the fault dump (INC-3)`.

---

## Task 5: Render decoded value beside raw hex (report dump table)

**Files:** `src/etnga_charge_report.cpp` (the dump-render block).

- [ ] **Step 1:** In the dump-render block, add a third column "decoded" and the latched trigger label. Change the header line + the per-row emit:
  - Section intro: use the latched trigger — `Phase <m_dump_phase_idx+1> fault (outcome 0x<m_dump_outcome> "<ChargeOutcomeLabel>")`.
  - Table header: `<tr><th>DID</th><th>raw (hex)</th><th>decoded</th></tr>`.
  - Per row: after the raw-hex `<td>`, add `<td>` + `DumpDidDecode(it->first, it->second)` (HTML-escaped; it's controlled text but escape defensively) or empty.
```cpp
            f << "<td>" << DumpDidDecode(it->first, it->second) << "</td>";
```
- [ ] **Step 2:** Inspection: 3-column table; decoded column populated for the confident subset (outcome label, temps, kW, HLC/AC-op), blank otherwise; the section title cites the triggering `0x29`-style code, not the benign current one.
- [ ] **Step 3:** Commit `etnga: render decoded DID values + correct trigger label in report (INC-3)`.

---

## Task 6: changes.txt + CI

- [ ] **Step 1:** Update the existing INC-3 `changes.txt` bullet (under the `????` pending block) to mention the dump now shows decoded values and is captured reliably on a fault (reword the existing e-TNGA fault-dump bullet rather than adding a new one).
- [ ] **Step 2:** Commit. Then `gh workflow run build.yml --ref feature/etnga-charge-error-dump`, watch, fix any compile error.
- [ ] **Step 3:** On-vehicle validation checklist (record): induce/observe a fault (an interrupted charge that reports `0x23/24/25/29` or `0x32/33/39/3A`) → the report (a) is generated even if ~no energy, (b) contains the full dump table with decoded values, (c) labels the *triggering* fault code/phase. The benign-stop non-fire still holds.

## Self-Review
- #139.1 (lost dump): Task 2 (no-skip on fault) + Task 3 (defer until complete) → the report reliably contains the dump. ✓
- #139.2 (mislabel): Task 1 (latch trigger outcome+phase). ✓
- Human-readable: Task 4 (confident decode) + Task 5 (render decoded col). ✓ Raw kept alongside. ✓
- Threading: all report/deferral logic on the Events task; poller callbacks unchanged (map+counter only); `phases[]` never touched off-Events. The defer holds `in_session` until finalize — Task 3 Step 5's `return` prevents the PISW-reconcile from racing. ✓
- Decode confidence: only existing-decoder / known-formula DIDs; uncertain ones raw-only. ✓
