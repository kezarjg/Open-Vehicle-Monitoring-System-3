# e-TNGA: human-readable PID names in the charge-fault dump — design

**Date:** 2026-06-24
**Issue:** follow-up to #137 (charge-fault diagnostic DID dump)
**Status:** design — pending user review

## Problem

The charge-fault diagnostic dump (merged via #137) renders a 3-column table — `DID | raw (hex) | decoded` — in the charge report. The `decoded` column shows a value for the confident subset (temps, kW, state labels), but every row's identity is just the bare hex DID (`0x1657`), so a reader has to look up what each DID *is*. Add a human-readable **name** per DID.

## Design

Add a new **"description" column** to the dump table, so it reads `DID | description | raw (hex) | decoded`:

```
DID     description                  raw     decoded
0x1657  PFC Temperature              51      31 °C
0x1666  HLC Comm Sequence Status     FF      HLC: Unconnected
0x1688  Charging History Information 26      AC Charging Stop (Operation)
```

A new static helper `DumpDidName(uint16_t did)` returns the name (a `const char*`), or `""` for an unexpected DID. The names are the solterra-can RE labels (sourced from `plug-in-charge-control.md` / `charging_state_machine_architecture.md` §6.4) — all 30 dump DIDs have a documented name, so the column is populated for every row.

### Name table (verbatim values)

| DID | name |
|---|---|
| `0x1666` | HLC Comm Sequence Status |
| `0x1684` | AC Charging Operation Status |
| `0x1688` | Charging History Information |
| `0x1664` | Charging Control Signal Status |
| `0x1667` | CCM Charge Stop Request |
| `0x1668` | DC Charging Control Status |
| `0x1736` | DC Operation Mode |
| `0x16AA` | DC Fault / Trip-Flag Register |
| `0x16A9` | DC Power-Limit History Flags |
| `0x161B` | Charge-Limit Status Flags |
| `0x1806` | AC Charging Relay Flags |
| `0x1702` | HV Charging Relay Flags |
| `0x1669` | PISW Status |
| `0x1602` | Connector Connect Status |
| `0x1601` | Connector Status Voltage |
| `0x1625` | Charging Lid Switch Status |
| `0x1670` | Hood Switch Signal |
| `0x164A` | HV Circuit Shutdown Signal |
| `0x10D4` | Battery Charging Power |
| `0x1654` | AC Input Current |
| `0x166C` | Station Present Output Current |
| `0x1621` | HV Battery Total Voltage |
| `0x166B` | Station Present Output Voltage |
| `0x165E` | PFC Boost Output Voltage |
| `0x1632` | AC Inlet Temperature Cluster |
| `0x1705` | DC Inlet Temperature Cluster |
| `0x1829` | Battery Maximum Temperature |
| `0x182A` | Battery Minimum Temperature |
| `0x1657` | PFC Temperature |
| `0x1658` | DC/DC Converter Temperature |

(High confidence for 23; the 7 bit-field/relay registers — `0x16AA`, `0x16A9`, `0x161B`, `0x1806`, `0x1702`, plus the documented category labels — are named at register/category level, which is what the RE supports.)

## Files

| File | Change |
|---|---|
| `src/vehicle_toyota_etnga.h` | declare `static const char* DumpDidName(uint16_t did);` |
| `src/etnga_charge_dump.cpp` | implement `DumpDidName` (switch over the 30 DIDs → names above; default `""`) |
| `src/etnga_charge_report.cpp` | dump render: add `<th>description</th>` to the header and a `<td>DumpDidName(it->first)</td>` cell per row, between the DID and raw columns (inside the existing `m_dump_mutex` lock) |
| `vehicle/OVMS.V3/changes.txt` | entry: the charge-fault dump now labels each DID with a human-readable name |

## Scope / non-goals

- **In:** the name helper + the description column. Names are static strings (no decode of the bit-field registers — that's a separate, RE-gated effort; the *value* decode stays as-is in `DumpDidDecode`).
- **Out:** any change to which DIDs are dumped, the decode logic, or the trigger/delivery (all merged in #137). This is a **separate single-purpose branch off master** — unrelated to PR #140 (the CAN-stale fix).
- No new polls.

## Validation

- **Compile:** CI.
- **On-vehicle:** on the next charge fault, the dump table shows the description column populated (e.g. `0x1657 | PFC Temperature | 51 | 31 °C`). Cosmetic; the decoded values and raw bytes are unchanged.
