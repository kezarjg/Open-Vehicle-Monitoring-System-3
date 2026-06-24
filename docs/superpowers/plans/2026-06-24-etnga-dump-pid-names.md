# e-TNGA: human-readable PID names in the charge-fault dump Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a human-readable name for each DID in the charge-fault diagnostic dump table (`DID | description | raw (hex) | decoded`).

**Architecture:** A static `DumpDidName(uint16_t)` maps each of the 30 dump DIDs to its solterra-can RE label; the dump render in `GenerateChargeReport` gains a "description" column between the DID and raw cells. Static-string names only — no decode change.

**Tech Stack:** C++ (ESP-IDF 3.3 / older GCC), the e-TNGA vehicle component. No new polls.

## Global Constraints

- **C++ for ESP-IDF 3.3 / older GCC.** Match surrounding style. No host tests; CI compiles (`gh workflow run build.yml --ref feature/etnga-dump-pid-names`). Do NOT propose a local `make`.
- **Names are verbatim** from the spec's name table — do NOT paraphrase or invent. `default: return "";`.
- **Single-purpose branch off master** (`feature/etnga-dump-pid-names`) — unrelated to PR #140. Work in `/home/devuser/wt-etnga-dump-names`. Paths relative to `vehicle/OVMS.V3/components/vehicle_toyota_etnga/`.

## Interfaces already in place
- The dump render in `etnga_charge_report.cpp` (under `OvmsMutexLock lock(&m_dump_mutex)`): header `f << "<table><tr><th>DID</th><th>raw (hex)</th><th>decoded</th></tr>\n";` then per-row `f << "<tr><td>" << did << "</td><td>";` … raw bytes … `f << "</td><td>" << DumpDidDecode(it->first, it->second) << "</td></tr>\n";`.
- `DumpDidDecode` (member) + `LimSideLabel` (static) are nearby label helpers; `static const char* TAG`.

---

## Task 1: `DumpDidName` helper

**Files:** `src/vehicle_toyota_etnga.h` (decl), `src/etnga_charge_dump.cpp` (impl).

- [ ] **Step 1: Declare** in `src/vehicle_toyota_etnga.h`, near the `DumpDidDecode` declaration:
```cpp
    static const char* DumpDidName(uint16_t did);   // solterra-can RE label for a dump DID; "" if unknown
```
- [ ] **Step 2: Implement** in `src/etnga_charge_dump.cpp` (near `DumpDidDecode`):
```cpp
// Human-readable name for a dump DID (solterra-can RE labels). "" for an unexpected DID.
const char* OvmsVehicleToyotaETNGA::DumpDidName(uint16_t did)
{
    switch (did) {
        case 0x1666: return "HLC Comm Sequence Status";
        case 0x1684: return "AC Charging Operation Status";
        case 0x1688: return "Charging History Information";
        case 0x1664: return "Charging Control Signal Status";
        case 0x1667: return "CCM Charge Stop Request";
        case 0x1668: return "DC Charging Control Status";
        case 0x1736: return "DC Operation Mode";
        case 0x16AA: return "DC Fault / Trip-Flag Register";
        case 0x16A9: return "DC Power-Limit History Flags";
        case 0x161B: return "Charge-Limit Status Flags";
        case 0x1806: return "AC Charging Relay Flags";
        case 0x1702: return "HV Charging Relay Flags";
        case 0x1669: return "PISW Status";
        case 0x1602: return "Connector Connect Status";
        case 0x1601: return "Connector Status Voltage";
        case 0x1625: return "Charging Lid Switch Status";
        case 0x1670: return "Hood Switch Signal";
        case 0x164A: return "HV Circuit Shutdown Signal";
        case 0x10D4: return "Battery Charging Power";
        case 0x1654: return "AC Input Current";
        case 0x166C: return "Station Present Output Current";
        case 0x1621: return "HV Battery Total Voltage";
        case 0x166B: return "Station Present Output Voltage";
        case 0x165E: return "PFC Boost Output Voltage";
        case 0x1632: return "AC Inlet Temperature Cluster";
        case 0x1705: return "DC Inlet Temperature Cluster";
        case 0x1829: return "Battery Maximum Temperature";
        case 0x182A: return "Battery Minimum Temperature";
        case 0x1657: return "PFC Temperature";
        case 0x1658: return "DC/DC Converter Temperature";
        default:     return "";
    }
}
```
- [ ] **Step 3: Inspection check** — Run: `grep -c 'case 0x' src/etnga_charge_dump.cpp` includes 30 new cases for `DumpDidName`; `grep -n 'DumpDidName' src/vehicle_toyota_etnga.h src/etnga_charge_dump.cpp` shows the decl + def. Spot-check 3 names against the spec table (`0x1657`→"PFC Temperature", `0x1829`→"Battery Maximum Temperature", `0x16AA`→"DC Fault / Trip-Flag Register").
- [ ] **Step 4: Commit** — `git add src/vehicle_toyota_etnga.h src/etnga_charge_dump.cpp && git commit -m "etnga: add DumpDidName — human-readable names for fault-dump DIDs"`.

---

## Task 2: Render the description column

**Files:** `src/etnga_charge_report.cpp` (dump render block).

- [ ] **Step 1: Header** — change the dump table header to add a description column after DID:
```cpp
            f << "<table><tr><th>DID</th><th>description</th><th>raw (hex)</th><th>decoded</th></tr>\n";
```
- [ ] **Step 2: Per-row cell** — change the per-row open from `f << "<tr><td>" << did << "</td><td>";` to insert the description cell before the raw cell:
```cpp
                f << "<tr><td>" << did << "</td><td>" << DumpDidName(it->first) << "</td><td>";
```
(The raw-bytes loop + `</td><td>decoded</td></tr>` close are unchanged, so the row is now `DID | description | raw | decoded` — 4 cells matching the 4 headers.)
- [ ] **Step 3: Inspection check** — Run: `grep -n 'th>description\|DumpDidName(it->first)' src/etnga_charge_report.cpp`. Confirm the header has 4 `<th>` (DID, description, raw, decoded) and the row emits 4 `<td>` (DID, DumpDidName, raw, decoded); both inside the existing `m_dump_mutex` lock.
- [ ] **Step 4: Commit** — `git add src/etnga_charge_report.cpp && git commit -m "etnga: show DID name column in the fault-dump report table"`.

---

## Task 3: changes.txt + CI

- [ ] **Step 1: changes.txt** — under the existing `????-??-?? ???  ???????  OTA release` pending block (no invented header), add:
```
- Toyota e-TNGA (Subaru Solterra / Toyota bZ4X): the charge-fault diagnostic DID dump now labels
    each DID with a human-readable name (e.g. "PFC Temperature", "HLC Comm Sequence Status")
    alongside the raw bytes and decoded value.
```
- [ ] **Step 2: Commit** — `git add vehicle/OVMS.V3/changes.txt && git commit -m "etnga: changes.txt for fault-dump DID names"`.
- [ ] **Step 3: CI build** — `gh workflow run build.yml --ref feature/etnga-dump-pid-names`, watch, fix any compile error.
- [ ] **Step 4: On-vehicle (opportunistic)** — on the next charge fault, the dump table shows the description column populated (`0x1657 | PFC Temperature | 51 | 31 °C`). Cosmetic; decoded values + raw bytes unchanged.

## Self-Review
- Name helper (30 DIDs, verbatim, default "") → Task 1. ✓
- Description column (header + per-row, between DID and raw) → Task 2. ✓
- changes.txt → Task 3. ✓
- Placeholder scan: no TBD; all 30 names inline. Type consistency: `DumpDidName(uint16_t)` decl (Task 1) matches the call `DumpDidName(it->first)` (Task 2; `it->first` is `uint16_t`). ✓
