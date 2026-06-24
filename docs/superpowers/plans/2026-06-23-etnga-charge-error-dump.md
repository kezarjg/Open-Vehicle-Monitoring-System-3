# e-TNGA charge report — error DID-dump (INC-3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** On a charge *fault*, capture a one-shot raw diagnostic snapshot of ~30 OBC DIDs and include it (raw hex) in the multi-phase charge report — async, TWDT-safe, decode-agnostic.

**Architecture:** Stacks on INC-2 (`feature/etnga-charge-limiting-side`). When the OBC reports an abnormal `0x1688` charge-stop outcome, set a fault flag; on the next `CHARGE_WAIT` entry, fire a burst of `OnceOffPoll` reads (the existing `RequestVIN` one-shot pattern) for the fixed DID set — all on ECU `0x745`/`0x74D`. Shared success/fail callbacks store the **raw response bytes** keyed by DID in a mutex-protected map; the report renders them as hex. Decode happens offline against the `solterra-can` RE repo, so no per-DID decode is implemented here.

**Tech Stack:** C++ (ESP-IDF 3.3 / older GCC), the OVMS poller (`OnceOffPoll`/`PollRequest`), the e-TNGA vehicle component. No new persistent polls.

## Why this is safe (the design that earns the increment)

- **Single ECU, confirmed:** all DIDs sit on `0x745` (tx) / `0x74D` (rx) — verified by a full gallia scan of the OBC. No ECU guessing.
- **Raw capture, not decode:** the dump stores the raw UDS response per DID as hex. Uncertain decodes are irrelevant; the artifact is decoded offline.
- **`OnceOffPoll`, not blocking:** uses the same async one-shot mechanism as `RequestVIN()` — each read auto-removes after one reply (`SeriesStatus::RemoveNext`); nothing blocks the poller or Events task, so no TWDT exposure. `PollSingleRequest` (which blocks and may not run on the poller task) is **not** used.
- **`"!v."`-prefixed names:** the `OnceOffPoll`s bind `this`; per the documented poller naming contract (the #123/#124 teardown-UAF lesson) they MUST be registered as `"!v.xte.dmp.*"` so `RemovePollRequestStarting("!v.")` reclaims them on vehicle teardown — otherwise a pending poll dereferences a freed signal.
- **Threading discipline:** the fault flag is set on the **poller task** (`IncomingPlugInControlSystem`, where `0x1688` is processed); the dump is kicked off and rendered on the **Events task** (`TransitionToChargeWaitState` / `GenerateChargeReport`); the shared results map is mutex-protected and the remaining-counter is `std::atomic`. **No `phases[]` is written from the poller task** — the dump is session-level state, rendered on the Events task.

## Global Constraints

- **C++ for ESP-IDF 3.3 / older GCC.** Match the surrounding style (`snprintf`, `ESP_LOGx(TAG, …)`, `std::bind` with `placeholders::_1..6`, `OvmsMutex`/`OvmsMutexLock`). No exceptions.
- **No host unit-test suite / no local build.** Per-task gate = inspection + reasoning; compile verified by CI (`gh workflow run build.yml --ref feature/etnga-charge-error-dump`). Do NOT propose a local `make`.
- **Fault trigger = abnormal `0x1688` codes ONLY:** AC `0x23`,`0x24`,`0x25`,`0x29`; DC `0x32`,`0x33`,`0x39`,`0x3A`. NOT benign stops (unplug `0x27`, IG-off `0x44`, over-60-min `0x38`, connector-unlock `0x2C`, reduced-supply `0x28`, operation `0x26`). This is the user-confirmed set.
- **Mechanism = transient `OnceOffPoll` burst** (user-confirmed), NOT `PollSingleRequest`, NOT a runtime-rebuilt static poll array.
- **`"!v.xte.dmp.<hex>"` registration names** — mandatory for teardown reclamation.
- **Raw hex only** — no per-DID decode in this PR.
- **Threading:** fault flag `std::atomic<bool>`; remaining-count `std::atomic<int>`; results `std::map<uint16_t,std::string>` guarded by a new `OvmsMutex m_dump_mutex`; never write `m_charge_session.phases` from a dump callback.
- **Degradable:** a DID that fails/times out is recorded as an explicit "(no reply)" entry, never blocks completion. If no fault occurs, nothing changes.
- **Single-purpose PR / `changes.txt`** bullet under the existing `????-??-?? ???  ???????  OTA release` pending block (no invented dated header).
- **Work in** `/home/devuser/wt-etnga-error-dump` on branch `feature/etnga-charge-error-dump` (stacked on `feature/etnga-charge-limiting-side`). Paths below are relative to `vehicle/OVMS.V3/components/vehicle_toyota_etnga/`.

## Interfaces from INC-1/INC-2 (already present)

- `ChargeSessionState` with `phases` (`std::vector<ChargePhase>`), `cur`; `ChargePhase` with `outcome`, `is_dc`, `start_utc`, etc.
- `OpenChargePhase`/`CloseChargePhase`/`ClassifyLimitingSide` in `etnga_charge_report.cpp`; `GenerateChargeReport()` with the per-phase render loop.
- `IncomingPollReply` → `IncomingPlugInControlSystem(uint16_t pid)` (`etnga_poll_processor.cpp`) — the `0x745` reply handler; `0x1688` is processed here (sets `m_v_charge_outcome`).
- `RequestVIN()` (`etnga_poll_processor.cpp`) — the exact `OnceOffPoll` precedent: `OnceOffPoll(successCb, failCb, TX, RX, VEHICLE_POLL_TYPE_READDATA, pid, ISOTP_STD, 0, retry_fail)` registered via `PollRequest(m_can2, "!v.xte.vin", entry)`; success cb `(uint16_t type, uint32_t sent, uint32_t rec, uint16_t pid, CAN_frame_format_t fmt, const std::string& data)`, fail cb `(…, int errorcode)`.
- ECU IDs `PLUG_IN_CONTROL_SYSTEM_TX = 0x745`, `PLUG_IN_CONTROL_SYSTEM_RX = 0x74D`.

## File Structure

| File | Responsibility | Change |
|---|---|---|
| `src/vehicle_toyota_etnga.h` | members + decls | Fault flag/atomics/map/mutex, dump-phase index + outcome, the DID table extern, decls for `MaybeStartChargeFaultDump`, `IncomingDumpSuccess/Fail`, `IsChargeFaultCode`. |
| `src/etnga_charge_dump.cpp` (NEW) | dump mechanism | DID table, `IsChargeFaultCode`, `MaybeStartChargeFaultDump` (kick off OnceOffPolls), success/fail callbacks (capture raw → map). |
| `src/etnga_poll_processor.cpp` | fault detection | In the `0x1688` path, set `m_charge_fault_pending` when `IsChargeFaultCode`. |
| `src/etnga_poll_states.cpp` | kickoff hook | Call `MaybeStartChargeFaultDump()` in `TransitionToChargeWaitState` (after `CloseChargePhase`). |
| `src/etnga_charge_report.cpp` | render | Add a "Diagnostic DID dump" section to the report when results exist; clear dump state on session open. |
| `component.mk` / `CMakeLists.txt` | build | (Only if the component doesn't auto-glob `src/*.cpp` — verify; e-TNGA globs, so likely no change.) |
| `changes.txt` | changelog | New bullet. |

---

## Task 1: State + DID table + declarations

**Files:** Modify `src/vehicle_toyota_etnga.h`.

**Interfaces produced:** members `std::atomic<bool> m_charge_fault_pending{false}`, `std::atomic<int> m_dump_remaining{0}`, `std::map<uint16_t,std::string> m_dump_results`, `OvmsMutex m_dump_mutex`, `int m_dump_phase_idx = -1`, `int m_dump_outcome = -1`; decls `static bool IsChargeFaultCode(int code);`, `void MaybeStartChargeFaultDump();`, `void IncomingDumpSuccess(uint16_t,uint32_t,uint32_t,uint16_t,CAN_frame_format_t,const std::string&);`, `void IncomingDumpFail(uint16_t,uint32_t,uint32_t,uint16_t,int);`.

- [ ] **Step 1: Add includes + members**

Ensure `<atomic>`, `<map>`, `<string>` are included (check the top of the header; add any missing). In the private member area near `m_charge_session`, add:

```cpp
    // INC-3: charge-fault diagnostic DID dump (raw one-shot snapshot of OBC DIDs on a fault).
    std::atomic<bool> m_charge_fault_pending{false};   // set on poller task (0x1688 fault), consumed on Events task
    std::atomic<int>  m_dump_remaining{0};             // OnceOffPolls still outstanding
    std::map<uint16_t,std::string> m_dump_results;     // pid -> raw response bytes ("" = no reply)
    OvmsMutex         m_dump_mutex;                     // guards m_dump_results (poller task writes, Events task reads)
    int               m_dump_phase_idx = -1;           // which phase faulted (index into m_charge_session.phases)
    int               m_dump_outcome = -1;             // the 0x1688 code that triggered the dump
```

- [ ] **Step 2: Add method declarations**

Near the `RequestVIN` / charge-report declarations, add:

```cpp
    static bool IsChargeFaultCode(int code);   // INC-3: true for abnormal 0x1688 stop codes
    void MaybeStartChargeFaultDump();          // INC-3: kick off the OnceOffPoll burst (Events task)
    void IncomingDumpSuccess(uint16_t type, uint32_t module_sent, uint32_t module_rec,
                             uint16_t pid, CAN_frame_format_t format, const std::string& data);
    void IncomingDumpFail(uint16_t type, uint32_t module_sent, uint32_t module_rec,
                          uint16_t pid, int errorcode);
```

- [ ] **Step 3: Inspection check**

Run: `grep -n 'm_charge_fault_pending\|m_dump_remaining\|m_dump_results\|m_dump_mutex\|IsChargeFaultCode\|MaybeStartChargeFaultDump\|IncomingDumpSuccess\|IncomingDumpFail' src/vehicle_toyota_etnga.h` and `grep -n '#include <atomic>\|#include <map>' src/vehicle_toyota_etnga.h`
Expected: all members + decls present; `<atomic>` and `<map>` included.

- [ ] **Step 4: Commit**

```bash
git add src/vehicle_toyota_etnga.h
git commit -m "etnga: add charge-fault DID-dump state + decls (INC-3, #81)"
```

---

## Task 2: The dump mechanism (`etnga_charge_dump.cpp`)

**Files:** Create `src/etnga_charge_dump.cpp`.

**Interfaces consumed:** the Task-1 members; `PLUG_IN_CONTROL_SYSTEM_TX/RX`; `PollRequest`, `OvmsPoller::OnceOffPoll`, `VEHICLE_POLL_TYPE_READDATA`, `ISOTP_STD`; `m_charge_session.phases`.
**Interfaces produced:** definitions of `IsChargeFaultCode`, `MaybeStartChargeFaultDump`, `IncomingDumpSuccess`, `IncomingDumpFail`, and the file-scope DID table.

- [ ] **Step 1: Create the file with the DID table + fault-code test**

Create `src/etnga_charge_dump.cpp`:

```cpp
/*
   Project:   Open Vehicle Monitor System
   Module:    Vehicle Toyota e-TNGA platform — charge-fault diagnostic DID dump
   Date:      23rd June 2026

   (C) 2026   Jerry Kezar <solterra@kezarnet.com>
   Licensed under the MIT License.

   On an abnormal 0x1688 charge-stop, fire a one-shot OnceOffPoll burst over the OBC's
   diagnostic DIDs (all on 0x745/0x74D) and capture each raw response for the charge report.
   Raw hex only — decode happens offline against the solterra-can RE repo.
*/

#include "vehicle_toyota_etnga.h"

using namespace std;

// All diagnostic DIDs live on the Plug-In Control System / OBC (tx 0x745 / rx 0x74D),
// confirmed by a full gallia scan. Raw responses are captured; no decode here.
static const uint16_t etnga_dump_dids[] = {
    // state-machine
    0x1666, 0x1684, 0x1688, 0x1664, 0x1667, 0x1668, 0x1736,
    // trip-flag
    0x16AA, 0x16A9, 0x161B, 0x1806, 0x1702,
    // connector + safety
    0x1669, 0x1602, 0x1601, 0x1625, 0x1670, 0x164A,
    // electrical at fault
    0x10D4, 0x1654, 0x166C, 0x1621, 0x166B, 0x165E,
    // thermal
    0x1632, 0x1705, 0x1829, 0x182A, 0x1657, 0x1658,
};
static const int ETNGA_DUMP_DID_COUNT = sizeof(etnga_dump_dids) / sizeof(etnga_dump_dids[0]);

// True only for genuine abnormal stops (per the user-confirmed set) — NOT benign stops.
bool OvmsVehicleToyotaETNGA::IsChargeFaultCode(int code)
{
    switch (code & 0xFF) {
        case 0x23: case 0x24: case 0x25: case 0x29:   // AC: Abnormal / Battery / High-Power / System
        case 0x32: case 0x33: case 0x39: case 0x3A:   // DC: Abnormal / Battery / System / Vehicle-System
            return true;
        default:
            return false;
    }
}
```

- [ ] **Step 2: Add the success/fail callbacks**

Append to `etnga_charge_dump.cpp`:

```cpp
// Poller-task callbacks: store the raw response (or an empty marker on failure) keyed by DID,
// and decrement the outstanding counter. No phases[] access here (Events task owns that).
void OvmsVehicleToyotaETNGA::IncomingDumpSuccess(uint16_t, uint32_t, uint32_t,
        uint16_t pid, CAN_frame_format_t, const std::string& data)
{
    {
        OvmsMutexLock lock(&m_dump_mutex);
        m_dump_results[pid] = data;
    }
    if (--m_dump_remaining <= 0)
        ESP_LOGI(TAG, "Charge-fault DID dump complete (%d DIDs captured)", (int) m_dump_results.size());
}

void OvmsVehicleToyotaETNGA::IncomingDumpFail(uint16_t, uint32_t, uint32_t,
        uint16_t pid, int errorcode)
{
    {
        OvmsMutexLock lock(&m_dump_mutex);
        m_dump_results[pid] = "";   // "" => rendered as "(no reply)"
    }
    ESP_LOGD(TAG, "Charge-fault dump: DID %04X no reply (err %d)", pid, errorcode);
    if (--m_dump_remaining <= 0)
        ESP_LOGI(TAG, "Charge-fault DID dump complete (%d DIDs)", (int) m_dump_results.size());
}
```

- [ ] **Step 3: Add the kickoff**

Append to `etnga_charge_dump.cpp`:

```cpp
// Events task (called from TransitionToChargeWaitState after CloseChargePhase). If a fault was
// flagged and no dump is already running, fire a OnceOffPoll for every diagnostic DID. Each
// auto-removes after one reply; "!v." prefix => reclaimed on teardown (poller naming contract).
void OvmsVehicleToyotaETNGA::MaybeStartChargeFaultDump()
{
    if (!m_charge_fault_pending.exchange(false))
        return;
    if (m_dump_remaining.load() > 0)
        return;   // a dump is already in flight; don't overlap

    using std::placeholders::_1; using std::placeholders::_2; using std::placeholders::_3;
    using std::placeholders::_4; using std::placeholders::_5; using std::placeholders::_6;

    {
        OvmsMutexLock lock(&m_dump_mutex);
        m_dump_results.clear();
    }
    m_dump_phase_idx = (int) m_charge_session.phases.size() - 1;   // the phase that just closed/faulted
    m_dump_outcome   = m_v_charge_outcome->AsInt();
    m_dump_remaining = ETNGA_DUMP_DID_COUNT;

    char name[24];
    for (int i = 0; i < ETNGA_DUMP_DID_COUNT; i++) {
        uint16_t did = etnga_dump_dids[i];
        auto entry = std::shared_ptr<OvmsPoller::OnceOffPoll>(
            new OvmsPoller::OnceOffPoll(
                std::bind(&OvmsVehicleToyotaETNGA::IncomingDumpSuccess, this, _1, _2, _3, _4, _5, _6),
                std::bind(&OvmsVehicleToyotaETNGA::IncomingDumpFail,    this, _1, _2, _3, _4, _5),
                PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX,
                VEHICLE_POLL_TYPE_READDATA, did,
                ISOTP_STD, 0, /*retry_fail=*/1));
        snprintf(name, sizeof(name), "!v.xte.dmp.%04X", did);   // "!v." => teardown-reclaimed
        PollRequest(m_can2, name, entry);
    }
    ESP_LOGW(TAG, "Charge fault (0x%02X) — diagnostic dump of %d OBC DIDs started",
             m_dump_outcome & 0xFF, ETNGA_DUMP_DID_COUNT);
}
```

- [ ] **Step 4: Inspection check**

Run: `grep -n 'etnga_dump_dids\|IsChargeFaultCode\|MaybeStartChargeFaultDump\|OnceOffPoll\|!v.xte.dmp' src/etnga_charge_dump.cpp`
Expected: 30-DID table; fault-code test with the exact code set; kickoff builds `ETNGA_DUMP_DID_COUNT` OnceOffPolls named `"!v.xte.dmp.%04X"`, shared callbacks bound with `_1.._6`/`_1.._5`. Confirm `m_can2`, `PollRequest`, `OvmsPoller::OnceOffPoll`, `VEHICLE_POLL_TYPE_READDATA`, `ISOTP_STD` match the `RequestVIN` usage in `etnga_poll_processor.cpp` (open it and compare the constructor arg order).

- [ ] **Step 5: Commit**

```bash
git add src/etnga_charge_dump.cpp
git commit -m "etnga: charge-fault DID-dump mechanism via OnceOffPoll burst (INC-3, #81)"
```

---

## Task 3: Detect the fault (set the flag)

**Files:** Modify `src/etnga_poll_processor.cpp` — the `0x1688` (`PID_CHARGE_HISTORY`) handling in `IncomingPlugInControlSystem`.

- [ ] **Step 1: Set the flag on an abnormal outcome**

Locate where `0x1688` / `PID_CHARGE_HISTORY` is decoded (it sets `m_v_charge_outcome`). Immediately after the outcome value is set, add:

```cpp
            // INC-3: flag a diagnostic dump on a genuine charge fault (consumed on CHARGE_WAIT entry).
            if (IsChargeFaultCode(outcome_code))
                m_charge_fault_pending = true;
```

(Use whatever local holds the decoded code — if the handler decodes into e.g. `int val = m_rxbuf[0];` pass that to `IsChargeFaultCode`. If no local exists, read `GetRxBUint8(m_rxbuf, 0)` consistent with the surrounding decode. Match the existing decode of this PID exactly; do not change what `m_v_charge_outcome` is set to.)

- [ ] **Step 2: Inspection check**

Run: `grep -n 'IsChargeFaultCode\|m_charge_fault_pending\|PID_CHARGE_HISTORY\|0x1688' src/etnga_poll_processor.cpp`
Expected: the flag is set only inside the `0x1688` handler, guarded by `IsChargeFaultCode`, without altering the existing `m_v_charge_outcome` set. Confirm the code value passed matches how this handler reads the outcome byte.

- [ ] **Step 3: Commit**

```bash
git add src/etnga_poll_processor.cpp
git commit -m "etnga: flag diagnostic dump on abnormal 0x1688 outcome (INC-3, #81)"
```

---

## Task 4: Kick off the dump on CHARGE_WAIT entry

**Files:** Modify `src/etnga_poll_states.cpp` — `TransitionToChargeWaitState`.

- [ ] **Step 1: Call the kickoff after the phase closes**

In `TransitionToChargeWaitState()`, after the existing `CloseChargePhase();` call (INC-1), add:

```cpp
    MaybeStartChargeFaultDump();   // INC-3: if a fault was flagged, snapshot the OBC diagnostic DIDs
```

- [ ] **Step 2: Inspection check**

Run: `grep -n 'CloseChargePhase\|MaybeStartChargeFaultDump' src/etnga_poll_states.cpp`
Expected: `MaybeStartChargeFaultDump()` immediately follows `CloseChargePhase()` in `TransitionToChargeWaitState` (so `phases.back()` is the just-closed faulted phase and `m_dump_phase_idx` is correct). Reason: fault `0x1688` arrives during AC/DC; the AC/DC→WAIT transition closes the phase then kicks the dump — correct ordering.

- [ ] **Step 3: Commit**

```bash
git add src/etnga_poll_states.cpp
git commit -m "etnga: start charge-fault dump on CHARGE_WAIT entry (INC-3, #81)"
```

---

## Task 5: Render the dump + reset on session open

**Files:** Modify `src/etnga_charge_report.cpp`.

- [ ] **Step 1: Clear dump state at session open**

INC-1 opens the session in `TransitionToChargeHandshakeState` (in `etnga_poll_states.cpp`), but the dump state is cleared most safely where the report/session resets are centralized. Add a small helper call: in `GenerateChargeReport()`, AFTER the report is rendered+enqueued (just before the function returns), clear the dump state so it cannot leak into a later session:

```cpp
    // INC-3: dump state is session-scoped; clear after the report consumes it.
    {
        OvmsMutexLock lock(&m_dump_mutex);
        m_dump_results.clear();
    }
    m_dump_phase_idx = -1;
    m_dump_outcome = -1;
    m_dump_remaining = 0;
```

(Also defensively clear in the existing session-open path if the implementer finds the report is skipped on a no-energy session — but the report-end clear covers the normal flow.)

- [ ] **Step 2: Render the diagnostic dump section**

In `GenerateChargeReport()`, after the per-phase loop and before the "Session events" table, add a dump section guarded by results existing:

```cpp
    {
        OvmsMutexLock lock(&m_dump_mutex);
        if (!m_dump_results.empty()) {
            f << "<h2>Diagnostic DID dump</h2>\n";
            char db[64];
            snprintf(db, sizeof(db), "Phase %d fault (outcome 0x%02X) — OBC 0x745, raw UDS 0x22 responses.",
                     m_dump_phase_idx + 1, m_dump_outcome & 0xFF);
            f << "<p>" << db << "</p>\n";
            f << "<table><tr><th>DID</th><th>raw response (hex)</th></tr>\n";
            for (std::map<uint16_t,std::string>::const_iterator it = m_dump_results.begin();
                 it != m_dump_results.end(); ++it) {
                f << "<tr><td>0x" << std::hex << std::uppercase << it->first << std::dec << "</td><td>";
                if (it->second.empty()) f << "(no reply)";
                else {
                    char hx[4];
                    for (size_t k = 0; k < it->second.size(); k++) {
                        snprintf(hx, sizeof(hx), "%02X ", (uint8_t) it->second[k]);
                        f << hx;
                    }
                }
                f << "</td></tr>\n";
            }
            f << "</table>\n";
        }
    }
```

- [ ] **Step 3: Inspection check**

Run: `grep -n 'Diagnostic DID dump\|m_dump_results\|OvmsMutexLock' src/etnga_charge_report.cpp`
Expected: one render block (mutex-locked) before the events table; one clear block at report end (mutex-locked). The render must hold `m_dump_mutex` for the whole iteration (poller-task callbacks may still be writing). Confirm `<iomanip>`/`std::hex` usage compiles in this file's style — prefer the `snprintf("%02X")` form shown (avoids `<iomanip>` dependency); for the DID header use `snprintf(db2, sizeof(db2), "0x%04X", it->first)` rather than stream hex manipulators if the file doesn't already use `<iomanip>`. **Implementer: use `snprintf` for both the DID and the bytes to match the file's existing style; do not introduce `<iomanip>` if it isn't already included.**

- [ ] **Step 4: Commit**

```bash
git add src/etnga_charge_report.cpp
git commit -m "etnga: render charge-fault diagnostic DID dump in report (INC-3, #81)"
```

---

## Task 6: changes.txt + CI build + validation handoff

- [ ] **Step 1: changes.txt bullet** (under the existing `????-??-?? ???  ???????  OTA release` pending block, matching neighbors, NO invented header):

```
- Toyota e-TNGA (Subaru Solterra / Toyota bZ4X): when a charge ends abnormally (an OBC
    fault stop), the charge report now includes a raw diagnostic snapshot of ~30 OBC
    DIDs captured at the fault, to aid offline diagnosis. Normal charges are unaffected.
```

- [ ] **Step 2: Commit**

```bash
git add vehicle/OVMS.V3/changes.txt
git commit -m "etnga: changes.txt for charge-fault DID dump (INC-3, #81)"
```

- [ ] **Step 3: CI build**

Run: `gh workflow run build.yml --ref feature/etnga-charge-error-dump`, watch with `gh run watch <id> --exit-status`. Expected green. **Pay attention** to: the new `etnga_charge_dump.cpp` being picked up by the component build (e-TNGA globs `src/*.cpp` — verify a `component.mk`/`CMakeLists.txt` glob exists; if the component lists sources explicitly, add `src/etnga_charge_dump.cpp`). Fix any error and amend the relevant commit.

- [ ] **Step 4: On-vehicle validation checklist (record in PR / memory)**

The real gate — car-gated, rare (needs a charge fault):
1. **No-fault regression:** a normal AC/DC charge produces NO "Diagnostic DID dump" section; no extra bus traffic in CHARGE_WAIT; report otherwise identical to INC-2.
2. **Fault path:** induce/observe an abnormal stop (e.g. an interrupted DC session that reports `0x32`) → the report shows the dump table with raw hex for the DID set; "(no reply)" rows are tolerated; no TWDT, no crash, poller resumes normally.
3. **Teardown:** `vehicle module NONE` mid-dump does not crash (the `"!v."` names reclaim the pending OnceOffPolls — this is the #123/#124 contract; explicitly exercise it).

---

## Self-Review

**Spec coverage (design doc §6.4):**
- ~30 DIDs read individually via `0x22`, one-shot, in CHARGE_WAIT → Tasks 2+4. ✓ (all on 0x745, confirmed.)
- Triggered by `error_in_phase` → Task 3 (abnormal `0x1688` set). ✓
- Appended to the report → Task 5. ✓ (rendered as a session-level dump section keyed to the faulted phase, raw hex; per-phase `errors[]` vector deliberately not used to avoid cross-task `phases[]` writes — a documented, safer deviation.)
- Logged to OVMS log → Tasks 2 (`ESP_LOGW`/`ESP_LOGI`). ✓

**Placeholder scan:** no TBD/TODO; every step has complete code. The `0x1688` decode-local in Task 3 Step 1 is the one "match the existing handler" instruction (unavoidable without pinning the live line; bounded and inspection-gated).

**Threading review:** flag is `atomic<bool>` (poller-set, Events-consumed via `exchange`); counter is `atomic<int>`; `m_dump_results` is always accessed under `m_dump_mutex` (callbacks write, kickoff clears, report reads — all locked); `phases[]` is never touched off the Events task; `OnceOffPoll`s are `"!v."`-named for teardown reclamation. No `PollSingleRequest`. No blocking on either task.

**Type consistency:** callback signatures match the `RequestVIN` precedent (`_1.._6` success, `_1.._5` fail). `IsChargeFaultCode(int)`/`MaybeStartChargeFaultDump()`/`IncomingDumpSuccess/Fail` signatures match between header (Task 1) and definitions (Task 2). `m_dump_*` members written/read consistently across Tasks 2-5. `ETNGA_DUMP_DID_COUNT` drives both the counter and the loop.
