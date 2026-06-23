# e-TNGA pack-variant–correct BMS arrangement — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make e-TNGA per-cell BMS correct across all three documented HV packs (78 / 96 / 104 cells) by moving pack knowledge into the base and deriving the cell/sensor arrangement from each battery reply.

**Architecture:** The `OvmsVehicleToyotaETNGA` base owns all BMS pack knowledge: it sets shared-chemistry limits/thresholds plus a bootstrap 96-cell arrangement so per-cell routing is on for every e-TNGA vehicle, then on each `0x182E`/`0x1814` reply it derives the actual cell/sensor counts from the reply length and re-arranges via a small `cellCount → moduleCount` table. Leaf wrappers (Solterra, bZ4X, Lexus RZ) carry no BMS code.

**Tech Stack:** C++ (ESP-IDF 3.3 / older GCC), OVMS `OvmsVehicle` BMS API (`BmsSetCellArrangement*`, `BmsCheckChangeCellArrangement*`, `BmsSetCellVoltage/Temperature`).

## Global Constraints

- **No host test suite.** Per-task verification = compiles in CI (GitHub Actions, e-TNGA vehicles enabled); the only runtime test path is on-device. Do not invent pytest/host tests.
- **Match each file's existing style exactly** (indentation, brace style, comment density). e-TNGA source uses 4-space indentation.
- **Always name a PID with its ECU.** `0x182E` (cell voltages) and `0x1814` (cell temperatures) are on the Hybrid Battery System ECU (`HYBRID_BATTERY_SYSTEM_RX`).
- **78/104 packs are unvalidatable here** (no such hardware) — mark any 78/104-specific reasoning as unvalidated in code comments and `changes.txt`.
- **The 96-cell path must remain a byte-for-byte no-op** vs current master behaviour (the daily-driver Solterra).
- **Two PRs off current master**, superseding (then closing) #130, #131, #132: Tasks 1–6 = PR 1 (`feature/etnga-bms-pack-variants`, this branch); Task 7 = PR 2 (Lexus RZ, fork-only). Task 8 is on-device verification (manual, post-merge-candidate).

---

### Task 1: Base BMS defaults, bootstrap arrangement, and the pack table

Open per-cell BMS for every e-TNGA vehicle and add the `cellCount → moduleCount` lookup.

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h` (class member + method declaration)
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.cpp` (constructor + helper definition)

**Interfaces:**
- Produces: `int OvmsVehicleToyotaETNGA::PackModuleCount(int cellCount)` — returns module count for a known pack (96→4, 78→3, 104→4), or `0` for an unrecognised count.
- Produces: member `int m_bms_modules` — the resolved module count, default `4`; written by Task 3's voltage path, read by Task 3's temperature path.

- [ ] **Step 1: Declare the member and helper in the header**

In `vehicle_toyota_etnga.h`, inside the `class OvmsVehicleToyotaETNGA` body: add the helper declaration alongside the other battery/BMS helper method declarations (the `protected:` section that declares `SetBatteryCellVoltages` etc.), and the member alongside the other `m_*` data members:

```cpp
    // Resolve the e-TNGA HV pack module count from the per-reply cell count.
    // Known packs: 96 (2022-24, 4x24), 78 (2025/26 FWD, 3x26), 104 (2025/26 AWD, 4x26).
    // Returns 0 for an unrecognised count so callers keep the last good arrangement.
    int PackModuleCount(int cellCount);
```

```cpp
    int m_bms_modules = 4;   // resolved HV pack module count (bootstrap = 96-cell / 4 modules)
```

- [ ] **Step 2: Add the BMS defaults + bootstrap arrangement to the base constructor**

In `vehicle_toyota_etnga.cpp`, in the `OvmsVehicleToyotaETNGA::OvmsVehicleToyotaETNGA()` constructor, after the existing metric/CAN/web initialisation and before the closing brace, add:

```cpp
  // BMS pack: owned by the e-TNGA platform (not per-badge). All e-TNGA EVs share the
  // Toyota EM "Type B" cell chemistry; only the cell/sensor counts and module grouping
  // vary by model year, and those are resolved per-reply from the bus (see PackModuleCount
  // + SetBatteryCellVoltages/SetBatteryTemperatures). Declare a bootstrap 96-cell
  // arrangement here so per-cell BMS routing is active from boot, before the first reply.
  BmsSetCellArrangementVoltage(96, 24);
  BmsSetCellArrangementTemperature(24, 6);

  BmsSetCellLimitsVoltage(2.5f, 4.3f);
  BmsSetCellLimitsTemperature(-30.0f, 60.0f);

  BmsSetCellDefaultThresholdsVoltage(0.020f, 0.030f);     // 20 mV warn / 30 mV alert
  BmsSetCellDefaultThresholdsTemperature(4.0f, 8.0f);     // 4 °C warn / 8 °C alert
```

- [ ] **Step 3: Define the pack table helper**

In `vehicle_toyota_etnga.cpp` (anywhere among the battery helper definitions, e.g. near `CalculateBatteryCellVoltages`), add:

```cpp
int OvmsVehicleToyotaETNGA::PackModuleCount(int cellCount)
{
    // e-TNGA HV pack variants (documented 2026-06-23). 96-cell is on-vehicle validated;
    // 78/104 are reasoned from spec and UNVALIDATED (no such hardware available).
    switch (cellCount) {
        case 96:  return 4;   // 2022-24            : 4 modules x 24 cells
        case 78:  return 3;   // 2025/26 FWD        : 3 modules x 26 cells (UNVALIDATED)
        case 104: return 4;   // 2025/26 AWD/high   : 4 modules x 26 cells (UNVALIDATED)
        default:  return 0;   // unrecognised -> caller keeps last good arrangement
    }
}
```

- [ ] **Step 4: Verify it compiles**

Push the branch (or `gh workflow run build.yml --ref feature/etnga-bms-pack-variants`) and confirm the Firmware build job is green. Expected: builds clean; no behaviour change yet (the bootstrap arrangement matches the Solterra's previous declaration, and `PackModuleCount` is not yet called).

- [ ] **Step 5: Commit**

```bash
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.cpp
git commit -m "etnga: own BMS pack defaults + bootstrap arrangement + cellCount->modules table in base"
```

---

### Task 2: Length-driven decode of the cell-voltage and temperature replies

Decode `N` readings from the reply length instead of a fixed 96 cells / 24 sensors.

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_metrics.cpp` (`CalculateBatteryCellVoltages`, `CalculateBatteryTemperatures`)

**Interfaces:**
- Consumes: nothing from prior tasks.
- Produces: `CalculateBatteryCellVoltages` / `CalculateBatteryTemperatures` now return a vector sized to the reply (`data.size()/2`), not a fixed length.

- [ ] **Step 1: Make `CalculateBatteryCellVoltages` length-driven**

Replace the body of `CalculateBatteryCellVoltages`:

```cpp
std::vector<float> OvmsVehicleToyotaETNGA::CalculateBatteryCellVoltages(const std::string& data)
{
    // 0x182E payload (Hybrid Battery ECU): N cells x uint16 BE; each LSB = 5/65535 V (~76 uV).
    // Cell count is taken from the reply length so differently-sized e-TNGA packs are
    // index-safe -- there is no cell-count PID.
    std::vector<float> voltages;
    voltages.reserve(data.size() / 2);

    for (size_t i = 0; i + 1 < data.size(); i += 2) {
        uint16_t raw = GetRxBUint16(data, i);
        voltages.push_back(static_cast<float>(raw) * 5.0f / 65535.0f);
    }

    return voltages;
}
```

- [ ] **Step 2: Make `CalculateBatteryTemperatures` length-driven**

Replace the body of `CalculateBatteryTemperatures`:

```cpp
std::vector<float> OvmsVehicleToyotaETNGA::CalculateBatteryTemperatures(const std::string& data)
{
    // 0x1814 payload (Hybrid Battery ECU): N sensors x int16 BE Q8.8, -50 C. Sensor count
    // from reply length (no sensor-count PID) so all e-TNGA pack variants are index-safe.
    std::vector<float> temperatures;
    temperatures.reserve(data.size() / 2);

    for (size_t i = 0; i + 1 < data.size(); i += 2) {
        int16_t temperatureRaw = GetRxBInt16(data, i);
        float temperature = static_cast<float>(temperatureRaw) / 256.0f - 50.0f;
        temperatures.push_back(temperature);
    }

    return temperatures;
}
```

- [ ] **Step 3: Verify it compiles**

Build via CI. Expected: green. For a 96-cell pack the decode still yields 96 voltages / 24 temps (192/48-byte replies), so behaviour is unchanged; the new capability is only exercised once Tasks 3–4 land.

- [ ] **Step 4: Commit**

```bash
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_metrics.cpp
git commit -m "etnga: decode BMS cell/temperature counts from reply length"
```

---

### Task 3: Resolve and apply the arrangement from the detected counts

Re-arrange the BMS to the detected count with the correct per-module grouping.

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_metrics.cpp` (`SetBatteryCellVoltages`, `SetBatteryTemperatures`)

**Interfaces:**
- Consumes: `PackModuleCount(int)` and member `m_bms_modules` from Task 1; length-sized vectors from Task 2.
- Produces: `SetBatteryCellVoltages` sets `m_bms_modules`; both setters apply the resolved arrangement before routing readings.

- [ ] **Step 1: Resolve + apply in `SetBatteryCellVoltages`**

Replace the body of `SetBatteryCellVoltages`:

```cpp
void OvmsVehicleToyotaETNGA::SetBatteryCellVoltages(const std::vector<float>& voltages)
{
    // The e-TNGA base always declares an arrangement (bootstrap in the ctor), so per-cell
    // data routes through the BMS API. Re-arrange to the actual pack on each reply.
    if (BmsGetCellArangementVoltage() > 0) {
        int cells = static_cast<int>(voltages.size());
        int modules = PackModuleCount(cells);
        if (modules == 0) {
            ESP_LOGW(TAG, "0x182E: unrecognised cell count %d, keeping current BMS arrangement", cells);
            return;
        }
        m_bms_modules = modules;
        // Align the arrangement total + per-module grouping with the detected pack.
        // No-op (returns false) when both already match, e.g. the 96-cell pack (4x24).
        BmsCheckChangeCellArrangementVoltage(cells, cells / modules);
        BmsRestartCellVoltages();
        for (size_t i = 0; i < voltages.size(); ++i) {
            BmsSetCellVoltage(static_cast<int>(i), voltages[i]);
        }
    } else {
        StandardMetrics.ms_v_bat_cell_voltage->SetValue(voltages);
    }
}
```

- [ ] **Step 2: Resolve + apply in `SetBatteryTemperatures`**

Replace the body of `SetBatteryTemperatures`:

```cpp
void OvmsVehicleToyotaETNGA::SetBatteryTemperatures(const std::vector<float>& temperatures)
{
    // Group temperature sensors using the module count resolved from the cell-voltage
    // reply (m_bms_modules). If the sensor count does not divide evenly, fall back to a
    // single group so the cell data stays correct and only the display grouping degrades.
    if (BmsGetCellArangementTemperature() > 0) {
        int sensors = static_cast<int>(temperatures.size());
        if (sensors == 0) {
            return;
        }
        int perModule = (m_bms_modules > 0 && (sensors % m_bms_modules) == 0)
                        ? sensors / m_bms_modules
                        : sensors;
        if (perModule == sensors && m_bms_modules > 1) {
            ESP_LOGW(TAG, "0x1814: %d sensors not divisible by %d modules; using one group",
                     sensors, m_bms_modules);
        }
        BmsCheckChangeCellArrangementTemperature(sensors, perModule);
        BmsRestartCellTemperatures();
        for (size_t i = 0; i < temperatures.size(); ++i) {
            BmsSetCellTemperature(static_cast<int>(i), temperatures[i]);
        }
    } else {
        StandardMetrics.ms_v_bat_cell_temp->SetValue(temperatures);
    }
}
```

- [ ] **Step 3: Verify it compiles**

Build via CI. Expected: green. For the 96-cell pack: `cells=96, modules=4` → `BmsCheckChangeCellArrangementVoltage(96, 24)` (no-op, already 96/24); `sensors=24, m_bms_modules=4` → `(24,6)` (no-op). Behaviour identical to master.

- [ ] **Step 4: Commit**

```bash
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_metrics.cpp
git commit -m "etnga: re-arrange BMS to detected pack (count + per-module grouping) each reply"
```

---

### Task 4: Replace the fixed-size reply guards with a count whitelist

Stop rejecting the smaller (78-cell) pack; reject only genuinely unusable replies.

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_processor.cpp` (`PID_BATTERY_TEMPERATURES` and `PID_BATTERY_CELL_VOLTAGES` cases in `IncomingHybridBatterySystem`)

**Interfaces:**
- Consumes: `PackModuleCount(int)` from Task 1; length-driven `Calculate*` from Task 2.
- Produces: nothing for later tasks.

- [ ] **Step 1: Replace the cell-voltage case guard**

Find the `case PID_BATTERY_CELL_VOLTAGES:` block (currently guarded by `if (m_rxbuf.size() < 192)`) and replace the whole case with:

```cpp
        case PID_BATTERY_CELL_VOLTAGES: {
            // Dispatch only happens on a COMPLETE ISOTP reply (mlremain==0), so the length
            // reflects the pack, not a truncation. Accept any recognised pack; reject the
            // rest (no fixed 192-byte floor -- that rejected the smaller 78-cell pack).
            std::vector<float> voltages = CalculateBatteryCellVoltages(m_rxbuf);
            if (voltages.empty() || PackModuleCount(static_cast<int>(voltages.size())) == 0) {
                ESP_LOGW(TAG, "0x182E: unrecognised reply (%d bytes), skipping BMS voltage update",
                         (int)m_rxbuf.size());
                break;
            }
            SetBatteryCellVoltages(voltages);
            SetBatteryCellVoltageStatistics(voltages);
            break;
        }
```

- [ ] **Step 2: Replace the temperature case guard**

Find the `case PID_BATTERY_TEMPERATURES:` block (currently guarded by `if (m_rxbuf.size() < 48)`) and replace the whole case with:

```cpp
        case PID_BATTERY_TEMPERATURES: {
            // Temperature sensor count has no whitelist (it varies with the pack and is
            // grouped via m_bms_modules); reject only an empty/odd reply.
            std::vector<float> temperatures = CalculateBatteryTemperatures(m_rxbuf);
            if (temperatures.empty() || (m_rxbuf.size() % 2) != 0) {
                ESP_LOGW(TAG, "0x1814: invalid reply (%d bytes), skipping BMS temperature update",
                         (int)m_rxbuf.size());
                break;
            }
            SetBatteryTemperatures(temperatures);
            SetBatteryTemperatureStatistics(temperatures);
            break;
        }
```

- [ ] **Step 3: Verify it compiles**

Build via CI. Expected: green. 96-cell: `voltages.size()=96`, `PackModuleCount(96)=4` (≠0) → processes as before. A hypothetical 78-cell (156-byte) reply now passes instead of being rejected.

- [ ] **Step 4: Commit**

```bash
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_processor.cpp
git commit -m "etnga: whitelist-gate BMS replies by pack (drop fixed 192/48-byte floor)"
```

---

### Task 5: Remove the Solterra BMS block (now base-owned)

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_subaru_solterra/src/vehicle_subaru_solterra.cpp`

**Interfaces:**
- Consumes: the base bootstrap arrangement from Task 1.
- Produces: nothing.

- [ ] **Step 1: Delete the BMS declarations from the constructor**

In the `OvmsVehicleSubaruSolterra::OvmsVehicleSubaruSolterra()` constructor, remove the entire BMS block (the `BmsSetCellArrangementVoltage(96, 24)` / `...Temperature(24, 6)` / `BmsSetCellLimits*` / `BmsSetCellDefaultThresholds*` lines and their comments), leaving only:

```cpp
OvmsVehicleSubaruSolterra::OvmsVehicleSubaruSolterra()
{
  ESP_LOGI(TAG, "Subaru Solterra vehicle module");  // Log an informational message
}
```

- [ ] **Step 2: Verify it compiles**

Build via CI. Expected: green. Net BMS behaviour for the Solterra is unchanged — the base now supplies the identical 96/24 + limits + thresholds.

- [ ] **Step 3: Commit**

```bash
git add vehicle/OVMS.V3/components/vehicle_subaru_solterra/src/vehicle_subaru_solterra.cpp
git commit -m "solterra: drop BMS arrangement (now owned by the e-TNGA base)"
```

---

### Task 6: bZ4X destructor/indent fix + docs + changelog

bZ4X already has no BMS block (the base now provides it); fix its long-standing destructor log copy-paste and update docs.

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_bz4x/src/vehicle_toyota_bz4x.cpp`
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_bz4x/docs/index.rst`
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/docs/index.rst`
- Modify: `vehicle/OVMS.V3/changes.txt`

**Interfaces:** none.

- [ ] **Step 1: Fix the bZ4X destructor log + body indentation**

In `vehicle_toyota_bz4x.cpp`, change the constructor/destructor bodies to be 2-space indented (matching the Solterra) and fix the destructor message:

```cpp
// Constructor for the OvmsVehicleToyotaBz4x class
OvmsVehicleToyotaBz4x::OvmsVehicleToyotaBz4x()
{
  ESP_LOGI(TAG, "Toyota bZ4X vehicle module");  // Log an informational message
}

// Destructor for the OvmsVehicleToyotaBz4x class
OvmsVehicleToyotaBz4x::~OvmsVehicleToyotaBz4x()
{
  ESP_LOGI(TAG, "Shutdown Toyota bZ4X vehicle module");  // Log an informational message
}
```

- [ ] **Step 2: Update the bZ4X doc**

In `vehicle_toyota_bz4x/docs/index.rst`, replace the "Vehicle-specific support" body and the "Web UI" `/bms/cellmon` sentence so they state that per-cell BMS is provided by the e-TNGA base (not declared in the wrapper):

Replace the paragraph under `Vehicle-specific support` with:

```rst
The bZ4X adds no behavioural overrides on top of the e-TNGA platform; all of its support — including
per-cell BMS voltage/temperature monitoring — comes from the shared e-TNGA base, which owns the HV
pack arrangement for every e-TNGA vehicle and derives the actual cell/sensor counts from the battery
replies at runtime.

.. note::

   Per-cell BMS has been validated on the 96-cell Solterra. The e-TNGA base also supports the
   2025/26 refresh packs (78-cell FWD, 104-cell AWD) by deriving the count from the bus, but those
   are reasoned from spec and **not yet validated on bZ4X hardware**.
```

In the `Web UI` section, replace the `/bms/cellmon` sentence with:

```rst
full list.  The per-cell ``/bms/cellmon`` page is populated because the e-TNGA base declares a BMS
pack arrangement for all e-TNGA vehicles (see above).  The charging pages
```

- [ ] **Step 3: Update the e-TNGA doc**

In `vehicle_toyota_etnga/docs/index.rst`, update the BMS note (the `.. note::` after the support-overview table) so it reflects base-owned, auto-arranged BMS. Replace its body with:

```rst
   **BMS v+t Display** is ``Yes`` for all e-TNGA vehicles: the base class declares the HV pack
   arrangement (Toyota EM "Type B" chemistry, shared across the platform) and derives the actual
   cell and temperature-sensor counts from the ``0x182E`` / ``0x1814`` reply length at runtime, so
   per-cell history, deviation flags, and pack statistics work across pack variants (96-cell
   2022-24; 78-cell and 104-cell 2025/26 refresh). Only the 96-cell pack is on-vehicle validated.
```

Then update the `/bms/cellmon` row in the web-pages table so it no longer says the bZ4X is empty:

```rst
     - BMS cell monitor — per-cell voltage and temperature display.  Populated for all e-TNGA
       vehicles (the base declares the pack arrangement).
```

- [ ] **Step 4: Add the changelog entry**

In `changes.txt`, under the `????-??-??` OTA release block, add:

```
- Toyota e-TNGA: per-cell BMS now works across HV pack variants. The cell and temperature-sensor
    counts are derived from the battery reply length and the module grouping is selected from the
    pack (96-cell 2022-24 = 4x24; 78-cell 2025/26 FWD = 3x26; 104-cell 2025/26 AWD = 4x26), instead
    of assuming a fixed 96-cell pack. /bms/cellmon is now populated on all e-TNGA vehicles
    (including bZ4X). No change for the 96-cell pack (Solterra/bZ4X today). The 78/104 packs are
    reasoned from spec and NOT yet validated on hardware. No config changes.
```

- [ ] **Step 5: Verify it compiles**

Build via CI. Expected: green (docs/changelog do not affect the build; the .cpp change is trivial).

- [ ] **Step 6: Commit**

```bash
git add vehicle/OVMS.V3/components/vehicle_toyota_bz4x/src/vehicle_toyota_bz4x.cpp \
        vehicle/OVMS.V3/components/vehicle_toyota_bz4x/docs/index.rst \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/docs/index.rst \
        vehicle/OVMS.V3/changes.txt
git commit -m "bz4x/etnga: fix bZ4X destructor log + docs for base-owned pack-variant BMS"
```

**End of PR 1.** Push `feature/etnga-bms-pack-variants`, open the PR against fork master, confirm CI green.

---

### Task 7: Experimental Lexus RZ wrapper (PR 2, fork-only)

A thin `LEXRZ` wrapper with no BMS code (the base provides it). On its own branch off PR 1.

**Files (all new unless noted):**
- Create: `vehicle/OVMS.V3/components/vehicle_lexus_rz/CMakeLists.txt`
- Create: `vehicle/OVMS.V3/components/vehicle_lexus_rz/component.mk`
- Create: `vehicle/OVMS.V3/components/vehicle_lexus_rz/LICENSE`
- Create: `vehicle/OVMS.V3/components/vehicle_lexus_rz/src/vehicle_lexus_rz.h`
- Create: `vehicle/OVMS.V3/components/vehicle_lexus_rz/src/vehicle_lexus_rz.cpp`
- Modify: `vehicle/OVMS.V3/main/Kconfig`
- Modify: `.github/workflows/build.yml`
- Modify: `vehicle/OVMS.V3/changes.txt`

**Interfaces:**
- Consumes: `OvmsVehicleToyotaETNGA` (base) and its now base-owned BMS.
- Produces: vehicle type `LEXRZ`.

- [ ] **Step 1: Branch off PR 1**

```bash
git checkout -b feature/lexus-rz feature/etnga-bms-pack-variants
```

- [ ] **Step 2: Create the wrapper header**

`vehicle/OVMS.V3/components/vehicle_lexus_rz/src/vehicle_lexus_rz.h`:

```cpp
/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Lexus RZ
   Date:          7th June 2026

   (C) 2026       Jerry Kezar <solterra@kezarnet.com>

   EXPERIMENTAL / UNVALIDATED: this wrapper inherits the Toyota e-TNGA logic
   wholesale and adds nothing. It has NOT been validated against a real Lexus RZ.
   Fork-only -- do not upstream until validated on hardware.

   Licensed under the MIT License. See the LICENSE file for details.
*/

#ifndef __VEHICLE_LEXUS_RZ_H__
#define __VEHICLE_LEXUS_RZ_H__

#include "../../vehicle_toyota_etnga/src/vehicle_toyota_etnga.h"  // Toyota e-TNGA base

class OvmsVehicleLexusRZ : public OvmsVehicleToyotaETNGA
{
public:
  OvmsVehicleLexusRZ();
  ~OvmsVehicleLexusRZ();
  static constexpr const char* TAG = "v-lexus-rz";

};

#endif // __VEHICLE_LEXUS_RZ_H__
```

- [ ] **Step 3: Create the wrapper source**

`vehicle/OVMS.V3/components/vehicle_lexus_rz/src/vehicle_lexus_rz.cpp`:

```cpp
/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Lexus RZ
   Date:          7th June 2026

   (C) 2026       Jerry Kezar <solterra@kezarnet.com>

   EXPERIMENTAL / UNVALIDATED -- see vehicle_lexus_rz.h. Fork-only.

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include "ovms_log.h"
#include "vehicle_lexus_rz.h"

OvmsVehicleLexusRZ::OvmsVehicleLexusRZ()
{
  ESP_LOGI(TAG, "Lexus RZ vehicle module (EXPERIMENTAL / UNVALIDATED)");
}

OvmsVehicleLexusRZ::~OvmsVehicleLexusRZ()
{
  ESP_LOGI(TAG, "Shutdown Lexus RZ vehicle module");
}

class OvmsVehicleLexusRZInit
{
  public:
    OvmsVehicleLexusRZInit();
} MyOvmsVehicleLexusRZInit __attribute__ ((init_priority (9000)));

OvmsVehicleLexusRZInit::OvmsVehicleLexusRZInit()
  {
  ESP_LOGI(OvmsVehicleLexusRZ::TAG, "Registering Vehicle: Lexus RZ (9000) [EXPERIMENTAL/UNVALIDATED]");

  MyVehicleFactory.RegisterVehicle<OvmsVehicleLexusRZ>("LEXRZ","Lexus RZ");
  }
```

- [ ] **Step 4: Create the build files**

`vehicle/OVMS.V3/components/vehicle_lexus_rz/CMakeLists.txt`:

```cmake
set(srcs)
set(include_dirs)

if (CONFIG_OVMS_VEHICLE_LEXUS_RZ)
  list(APPEND srcs "src/vehicle_lexus_rz.cpp")
  list(APPEND include_dirs "src")
endif ()

# requirements can't depend on config
idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${include_dirs}
                       PRIV_REQUIRES "main"
                       WHOLE_ARCHIVE)
```

`vehicle/OVMS.V3/components/vehicle_lexus_rz/component.mk`:

```make
#
# Main component makefile.
#
# This Makefile can be left empty. By default, it will take the sources in the
# src/ directory, compile them and link them into lib(subdirectory_name).a
# in the build directory. This behaviour is entirely configurable,
# please read the ESP-IDF documents if you need to do this.
#

ifdef CONFIG_OVMS_VEHICLE_LEXUS_RZ
COMPONENT_ADD_INCLUDEDIRS:=src
COMPONENT_SRCDIRS:=src
COMPONENT_ADD_LDFLAGS = -Wl,--whole-archive -l$(COMPONENT_NAME) -Wl,--no-whole-archive
endif
```

`vehicle/OVMS.V3/components/vehicle_lexus_rz/LICENSE`: copy the MIT text from `vehicle/OVMS.V3/components/vehicle_toyota_bz4x/LICENSE` verbatim.

- [ ] **Step 5: Add the Kconfig toggle (default off)**

In `vehicle/OVMS.V3/main/Kconfig`, after the `config OVMS_VEHICLE_SUBARU_SOLTERRA` block and before `endmenu # Vehicle Support`, add:

```
config OVMS_VEHICLE_LEXUS_RZ
    bool "Include support for Lexus RZ vehicles (EXPERIMENTAL/UNVALIDATED)"
    default n
    depends on OVMS
    help
        Enable to include support for Lexus RZ vehicles.
        EXPERIMENTAL: inherits the Toyota e-TNGA logic and has not been validated
        on a real Lexus RZ. Off by default.
```

- [ ] **Step 6: Add to the CI compile set**

In `.github/workflows/build.yml`, in the `ENABLE_VEHICLES:` block, add the RZ symbol after `CONFIG_OVMS_VEHICLE_TOYOTA_BZ4X`:

```yaml
            CONFIG_OVMS_VEHICLE_LEXUS_RZ
```

- [ ] **Step 7: Add the changelog entry**

In `changes.txt`, under the `????-??-??` OTA release block, add:

```
- New experimental vehicle type "LEXRZ" (Lexus RZ), built on the Toyota e-TNGA base. Inherits the
    e-TNGA logic and BMS wholesale (the base owns the pack arrangement). EXPERIMENTAL and NOT
    validated on a real Lexus RZ; compiled into the fork build but disabled by default in menuconfig
    (CONFIG_OVMS_VEHICLE_LEXUS_RZ). No config changes.
```

- [ ] **Step 8: Verify it compiles**

Push `feature/lexus-rz` and confirm CI is green (the RZ symbol is now in `ENABLE_VEHICLES`, so the new component is compiled).

- [ ] **Step 9: Commit**

```bash
git add vehicle/OVMS.V3/components/vehicle_lexus_rz/ vehicle/OVMS.V3/main/Kconfig \
        .github/workflows/build.yml vehicle/OVMS.V3/changes.txt
git commit -m "lexus_rz: add EXPERIMENTAL fork-only Lexus RZ (LEXRZ) over e-TNGA"
```

**End of PR 2.** Open against fork master with PR 1 as its base (or rebase onto master after PR 1 merges).

---

### Task 8: On-device verification + close superseded PRs

The only real runtime test. Manual; do once PR 1 builds green.

**Files:** none.

- [ ] **Step 1: Flash PR 1 to the Solterra**

Build the image (CI artifact / MinIO per CLAUDE.local) and flash via `ota flash http` per the slot-stick procedure. Reach the module via the `solterra-ovms` SSH alias.

- [ ] **Step 2: Confirm the 96-cell no-op**

With the car in READY (so the Hybrid Battery ECU answers), check `/bms/cellmon` and metrics:
Expected — **identical to today**: 96 cell voltages, 24 temperatures, displayed as 4 modules of 24 cells + 6 sensors; pack voltage ≈ 96 × cell average (~355 V); no poll errors; no re-arrange warnings in the log (`0x182E: unrecognised…` must NOT appear).

- [ ] **Step 3: Record the result**

Note the outcome in the PR and in memory `[[project_etnga_multivariant]]`. 78/104 remain unverified (no hardware) — state this explicitly.

- [ ] **Step 4: Close the superseded PRs**

After PR 1 (and PR 2) are open and green, close #130, #131, #132 with a comment pointing to the new PRs as their replacement.

```bash
gh pr close 130 --repo kezarjg/Open-Vehicle-Monitoring-System-3 --comment "Superseded by the pack-variant BMS work (base-owned arrangement). See the new e-TNGA pack-variants PR."
gh pr close 132 --repo kezarjg/Open-Vehicle-Monitoring-System-3 --comment "Superseded by the pack-variant BMS work (table-driven count + grouping, bidirectional). See the new e-TNGA pack-variants PR."
gh pr close 131 --repo kezarjg/Open-Vehicle-Monitoring-System-3 --comment "Superseded by the new fork-only Lexus RZ PR (wrapper with no BMS block; base owns the pack)."
```

---

## Self-Review

**Spec coverage:**
- §1 ownership/structure → Task 1 (base defaults/bootstrap) + Task 5 (empty Solterra) + Task 6 (bZ4X has none) + Task 7 (RZ has none). ✅
- §2 count→layout resolution → Task 1 (table) + Task 2 (decode) + Task 3 (apply + grouping + temp cache + even-divide fallback). ✅
- §3 safety guard (drop byte-floor, whitelist, no hysteresis) → Task 4. ✅
- §4 migration / two PRs / close #130-132 → Tasks 1–6 (PR 1), Task 7 (PR 2), Task 8 step 4. ✅
- §5 testing (on-device no-op; 78/104 unvalidatable; CI; optional synthetic test) → Task 8. The optional synthetic-reply `test` command was **dropped** (YAGNI: `PackModuleCount` is a pure switch, trivially correct by inspection; the integration risk is only on real hardware, which a synthetic command cannot cover). Noted here intentionally.

**Placeholder scan:** No TBD/TODO/"handle edge cases" — every code step shows complete code. ✅

**Type consistency:** `PackModuleCount(int)→int` and `m_bms_modules` (int, default 4) are declared in Task 1 and used identically in Tasks 3 and 4. `BmsCheckChangeCellArrangementVoltage(int,int)` / `...Temperature(int,int)` match the base API (`vehicle.h:672-673`). `CalculateBattery*`/`SetBattery*`/`SetBattery*Statistics` names match the existing functions. ✅
