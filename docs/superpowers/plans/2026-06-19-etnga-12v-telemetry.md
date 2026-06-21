# e-TNGA 12V Battery History — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete the OVMS "12V Battery History" chart for e-TNGA and add headline 12V aux-battery health metrics, by rebasing the stale `feature/etnga-12v-aux-health` branch onto master and extending it with the two missing pieces (the `charging12v`/`aux12v` flags and the `0x15E8` lifetime integrators).

**Architecture:** All work is in the **base `vehicle_toyota_etnga`** component (shared by Solterra/bZ4X/Lexus RZ). 12V telemetry is read from the **EV ECU `0x7D2`/`0x7DA`**. The branch already polls `0x15EE/15FD/15F8/15E5` and has correct decode helpers; this plan refreshes it onto current master (the poll list changed format) and adds the flags + `0x15E8`.

**Tech Stack:** C++ (ESP-IDF 3.3 / older GCC), OVMS poller framework, OVMS metrics.

## Global Constraints

- **No host test suite.** Verification is **GitHub CI build** (push to branch) + **on-device** checks. No pytest/unit tests. In-session verification per task = targeted `grep`/inspection + compile-readiness review.
- **Build is never local** — do not propose `make` in the devcontainer as the gate; CI is the gate.
- **Match each file's existing style exactly.** Single-purpose PR (one vehicle feature).
- **Always pair a PID with its ECU** in comments/docs (all 12V DIDs are on the EV ECU `0x7D2`).
- **Do not change how `v.b.12v.voltage` is sourced** — it stays the ESP32 ADC (housekeeping). `0x15EE` goes to the custom `xte.v.b.12v.voltage` only.
- Worktree: `/home/devuser/wt-etnga-12v`, branch `feature/etnga-12v-aux-health`.
- Poll-list offset blocks: `obdii_polls_base[]` columns `{SLEEP, AWAKE, DRIVING, -}`; `obdii_polls_charge[]` columns `{HANDSHAKE, WAIT, AC, DC}`. EV-ECU PIDs poll in DRIVING + AC/DC only (never AWAKE).

---

### Task 1: Rebase the branch onto master and reformat the 12V poll rows to offset blocks

**Files:**
- Rebase target: branch `feature/etnga-12v-aux-health` onto `master` (`7d4bc1b`).
- Modify (conflict resolution): `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.cpp`, `etnga_metrics.cpp`, `etnga_poll_processor.cpp`, `etnga_poll_states.cpp`, `vehicle_toyota_etnga.h`, `vehicle/OVMS.V3/changes.txt`.

**Interfaces:**
- Consumes: nothing (first task).
- Produces: a rebased branch where the existing 4 PIDs (`0x15EE/15FD/15F8/15E5`) are polled via `obdii_polls_base[]` + `obdii_polls_charge[]` in the new offset-block format, and the decode helpers (`CalculateAux12vVoltage/Current/Temperature/FullCharge`, `SetAux12v*`) compile against current master.

- [ ] **Step 1: Start the rebase**

```bash
cd /home/devuser/wt-etnga-12v
git fetch origin && git rebase master
```
Expected: conflicts in the 6 files above (primarily `vehicle_toyota_etnga.cpp` poll list).

- [ ] **Step 2: In `vehicle_toyota_etnga.cpp`, delete the old 7-column 12V rows and add offset-block rows**

The stale rows look like `{ ..., PID_AUX_BATTERY_VOLTAGE, { 0, 30, 30, 0, 0, 60, 60}, 0, ISOTP_STD }`. Remove all four. Then add to **`obdii_polls_base[]`** (place near the other `HYBRID_CONTROL_SYSTEM` rows):

```cpp
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AUX_BATTERY_CURRENT, { 0, 0,  10, 0}, 0, ISOTP_STD }, // 0x15FD 12V aux current (EV ECU): DRIVING @10s
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AUX_BATTERY_VOLTAGE, { 0, 0,  30, 0}, 0, ISOTP_STD }, // 0x15EE 12V aux voltage (EV ECU, hi-res): DRIVING @30s
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AUX_BATTERY_TEMP,    { 0, 0, 120, 0}, 0, ISOTP_STD }, // 0x15F8 12V aux temp (EV ECU): DRIVING @120s
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AUX_BATTERY_FULL_CHARGE, { 0, 0, 120, 0}, 0, ISOTP_STD }, // 0x15E5 12V aux CAC (EV ECU): DRIVING @120s
```

And to **`obdii_polls_charge[]`** (place near the other `HYBRID_CONTROL_SYSTEM` charge rows):

```cpp
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AUX_BATTERY_CURRENT, { 0, 0,  10,  10}, 0, ISOTP_STD }, // 0x15FD 12V aux current: AC+DC @10s
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AUX_BATTERY_VOLTAGE, { 0, 0,  30,  30}, 0, ISOTP_STD }, // 0x15EE 12V aux voltage: AC+DC @30s
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AUX_BATTERY_TEMP,    { 0, 0, 120, 120}, 0, ISOTP_STD }, // 0x15F8 12V aux temp: AC+DC @120s
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AUX_BATTERY_FULL_CHARGE, { 0, 0, 120, 120}, 0, ISOTP_STD }, // 0x15E5 12V aux CAC: AC+DC @120s
```

- [ ] **Step 3: Resolve the remaining file conflicts by re-applying the branch's intent**

For `etnga_metrics.cpp`, `etnga_poll_processor.cpp`, `etnga_poll_states.cpp`, `vehicle_toyota_etnga.h`, `changes.txt`: keep the branch's 12V additions (metric inits at the `xte.v.b.12v.*` names, `PID_AUX_BATTERY_*` defines, `CalculateAux12v*`/`SetAux12v*`, the dispatch cases, the "clear 12V metrics when off" blocks, the changes.txt entry) and accept master's surrounding code. Do not re-introduce the old 7-column poll arrays anywhere.

- [ ] **Step 4: Finish the rebase**

```bash
cd /home/devuser/wt-etnga-12v
git add -A && git rebase --continue
```
Expected: rebase completes; `git log --oneline master..HEAD` shows the 3 feature commits + the spec commit on top of `7d4bc1b`.

- [ ] **Step 5: Verify the poll rows and that no 7-column array survives**

```bash
cd /home/devuser/wt-etnga-12v/vehicle/OVMS.V3/components/vehicle_toyota_etnga/src
grep -nE "PID_AUX_BATTERY" vehicle_toyota_etnga.cpp
grep -cE "\{ *[0-9]+, *[0-9]+, *[0-9]+, *[0-9]+, *[0-9]+, *[0-9]+, *[0-9]+\}" vehicle_toyota_etnga.cpp
```
Expected: 8 `PID_AUX_BATTERY` rows (4 in base, 4 in charge), each with a **4-element** cadence array; the 7-element-array count is `0`.

- [ ] **Step 6: Commit the rebase result**

The rebase already rewrote history; no extra commit needed unless conflict resolution required manual edits beyond what the commits carried. If Step 3 required manual edits, they were folded in by `rebase --continue`. Confirm clean:
```bash
git status --porcelain   # expect empty
```

---

### Task 2: Derive the `v.e.charging12v` and `v.e.aux12v` flags

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_metrics.cpp` (the `SetAux12vCurrent` definition, ~line 516, and remove the TODO at ~line 513).
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp` (the two "clear 12V metrics" blocks).

**Interfaces:**
- Consumes: `StandardMetrics.ms_v_bat_12v_current` (set by `SetAux12vCurrent` from Task 1).
- Produces: `StandardMetrics.ms_v_env_charging12v` (bool) and `ms_v_env_aux12v` (bool) populated during DRIVING/charge and cleared when off.

- [ ] **Step 1: Replace `SetAux12vCurrent` to also set the flags**

In `etnga_metrics.cpp`, delete the `// TODO: optionally derive ... charging12v` comment and replace the one-line `SetAux12vCurrent` with:

```cpp
void OvmsVehicleToyotaETNGA::SetAux12vCurrent(float v)
{
    StandardMetrics.ms_v_bat_12v_current->SetValue(v);
    // Positive current = HV->12V DC-DC pushing charge into the aux battery.
    StandardMetrics.ms_v_env_charging12v->SetValue(v > 0.5f);
    // Any valid EV-ECU 12V reply means the aux system is energized.
    StandardMetrics.ms_v_env_aux12v->SetValue(true);
}
```

- [ ] **Step 2: Clear the flags in both "car off" blocks of `etnga_poll_states.cpp`**

Each block currently clears `ms_v_bat_12v_current` and the `xte` 12V metrics (the two blocks near the `m_v_bat_12v_*->Clear()` lines). In **both** blocks, immediately after the `StandardMetrics.ms_v_bat_12v_current->Clear();` line, add:

```cpp
    StandardMetrics.ms_v_env_charging12v->SetValue(false);
    StandardMetrics.ms_v_env_aux12v->SetValue(false);
```

- [ ] **Step 3: Verify**

```bash
cd /home/devuser/wt-etnga-12v/vehicle/OVMS.V3/components/vehicle_toyota_etnga/src
grep -n "ms_v_env_charging12v\|ms_v_env_aux12v" etnga_metrics.cpp etnga_poll_states.cpp
grep -c "TODO.*charging12v" etnga_metrics.cpp   # expect 0
```
Expected: `charging12v`/`aux12v` set once in `etnga_metrics.cpp` and twice (false) in `etnga_poll_states.cpp`; the TODO is gone.

- [ ] **Step 4: Commit**

```bash
git add -A && git commit -m "etnga: derive v.e.charging12v / v.e.aux12v from 12V aux current"
```

---

### Task 3: Add the `0x15E8` lifetime integrators (charge/discharge Ah, ready-on hours)

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h` (PID define, 3 metric members, 1 method decl).
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_metrics.cpp` (3 `InitFloat`, `DecodeAux12vIntegrators`).
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_processor.cpp` (dispatch case).
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.cpp` (poll rows).
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp` (clear new metrics when off).

**Interfaces:**
- Consumes: `GetRxBUint32(data, i)`, `GetRxBUint16(data, i)` (inline helpers in `vehicle_toyota_etnga.h`).
- Produces: `xte.v.b.12v.charge.ah`, `xte.v.b.12v.discharge.ah`, `xte.v.b.12v.readyon.h` metrics; `void DecodeAux12vIntegrators(const std::string& data)`.

- [ ] **Step 1: Header — add PID define, members, method decl (`vehicle_toyota_etnga.h`)**

After `PID_AUX_BATTERY_FULL_CHARGE = 0x15E5,` add:
```cpp
    PID_AUX_BATTERY_INTEGRATORS = 0x15E8,  // 12V aux cluster (EV ECU 0x7D2): lifetime charge/discharge Ah + ready-on hours
```
After the `m_v_bat_12v_cac;` member add:
```cpp
    OvmsMetricFloat* m_v_bat_12v_charge_ah;    // xte.v.b.12v.charge.ah    0x15E8 bytes 1-4  lifetime charge integral (Ah)
    OvmsMetricFloat* m_v_bat_12v_discharge_ah; // xte.v.b.12v.discharge.ah 0x15E8 bytes 5-8  lifetime discharge integral (Ah)
    OvmsMetricFloat* m_v_bat_12v_readyon_h;    // xte.v.b.12v.readyon.h    0x15E8 bytes 11-12 integrated Ready-ON time (h)
```
Near the other `SetAux12v*`/decode declarations add:
```cpp
    void DecodeAux12vIntegrators(const std::string& data);
```

- [ ] **Step 2: `etnga_metrics.cpp` — init the 3 metrics**

After the `m_v_bat_12v_cac = MyMetrics.InitFloat("xte.v.b.12v.cac", ...)` line add:
```cpp
    m_v_bat_12v_charge_ah    = MyMetrics.InitFloat("xte.v.b.12v.charge.ah",    SM_STALE_MAX, 0.0f, AmpHours); // 0x15E8 lifetime charge integral
    m_v_bat_12v_discharge_ah = MyMetrics.InitFloat("xte.v.b.12v.discharge.ah", SM_STALE_MAX, 0.0f, AmpHours); // 0x15E8 lifetime discharge integral
    m_v_bat_12v_readyon_h    = MyMetrics.InitFloat("xte.v.b.12v.readyon.h",    SM_STALE_MAX, 0.0f, Hours);    // 0x15E8 integrated Ready-ON time
```

- [ ] **Step 3: `etnga_metrics.cpp` — add the decoder**

Near the other `SetAux12v*` definitions add:
```cpp
void OvmsVehicleToyotaETNGA::DecodeAux12vIntegrators(const std::string& data)
{
    // 0x15E8 (EV ECU 0x7D2), 17-byte aux-battery cluster (1-indexed per solterra-can doc):
    //   bytes 1-4   charging integrated current    u32 BE x0.1 Ah
    //   bytes 5-8   discharging integrated current  u32 BE x0.1 Ah
    //   bytes 11-12 Integrated Ready ON Time        u16 BE hours
    if (data.size() < 12)
        return;
    m_v_bat_12v_charge_ah->SetValue(static_cast<float>(GetRxBUint32(data, 0)) / 10.0f);
    m_v_bat_12v_discharge_ah->SetValue(static_cast<float>(GetRxBUint32(data, 4)) / 10.0f);
    m_v_bat_12v_readyon_h->SetValue(static_cast<float>(GetRxBUint16(data, 10)));
}
```

- [ ] **Step 4: `etnga_poll_processor.cpp` — dispatch the new PID**

After the `case PID_AUX_BATTERY_FULL_CHARGE:` block add:
```cpp
        case PID_AUX_BATTERY_INTEGRATORS: {
            DecodeAux12vIntegrators(m_rxbuf);
            break;
        }
```

- [ ] **Step 5: `vehicle_toyota_etnga.cpp` — add the poll rows**

In `obdii_polls_base[]` (with the other EV-ECU 12V rows):
```cpp
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AUX_BATTERY_INTEGRATORS, { 0, 0, 120, 0}, 0, ISOTP_STD }, // 0x15E8 12V lifetime integrators: DRIVING @120s
```
In `obdii_polls_charge[]`:
```cpp
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AUX_BATTERY_INTEGRATORS, { 0, 0, 120, 120}, 0, ISOTP_STD }, // 0x15E8 12V lifetime integrators: AC+DC @120s
```

- [ ] **Step 6: `etnga_poll_states.cpp` — clear the 3 new metrics in both "car off" blocks**

In **both** blocks, after the `m_v_bat_12v_cac->Clear();` line add:
```cpp
    m_v_bat_12v_charge_ah->Clear();
    m_v_bat_12v_discharge_ah->Clear();
    m_v_bat_12v_readyon_h->Clear();
```

- [ ] **Step 7: Verify decode offsets and wiring**

```bash
cd /home/devuser/wt-etnga-12v/vehicle/OVMS.V3/components/vehicle_toyota_etnga/src
grep -n "PID_AUX_BATTERY_INTEGRATORS" vehicle_toyota_etnga.h vehicle_toyota_etnga.cpp etnga_poll_processor.cpp
grep -n "12v.charge.ah\|12v.discharge.ah\|12v.readyon.h" etnga_metrics.cpp etnga_poll_states.cpp
```
Expected: PID defined once, polled twice (base+charge), dispatched once; the 3 metrics inited once, cleared twice. Re-read `DecodeAux12vIntegrators` and confirm offsets `0`, `4`, `10` match bytes 1-4 / 5-8 / 11-12.

- [ ] **Step 8: Commit**

```bash
git add -A && git commit -m "etnga: add 12V lifetime integrators (0x15E8: charge/discharge Ah, ready-on hours)"
```

---

### Task 4: Documentation — changes.txt and PID table

**Files:**
- Modify: `vehicle/OVMS.V3/changes.txt`
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/docs/index.rst`

**Interfaces:** none (docs only).

- [ ] **Step 1: Update the changes.txt 12V entry**

Ensure the existing e-TNGA 12V entry (carried by the branch) reflects the final metric set. The bullet body should read (match the file's existing date/author/bullet format):
```
- Toyota e-TNGA: 12V auxiliary battery telemetry from the EV ECU (0x7D2) now populates the
  12V Battery History chart — v.b.12v.current plus the v.e.charging12v / v.e.aux12v indicators —
  and adds aux-battery health metrics: xte.v.b.12v.voltage (hi-res), .temp, .cac (capacity),
  and lifetime .charge.ah / .discharge.ah / .readyon.h.
```
No `New configs:` block (no config keys added).

- [ ] **Step 2: Add the 5 PIDs to `docs/index.rst`**

In the PID table, add rows pairing each DID with the EV ECU (`0x7D2`), matching the table's existing column format:
```
   * - ``PID_AUX_BATTERY_CURRENT`` (``0x15FD``)
     - EV ECU ``0x7D2`` — 12V aux current, ``(raw-12500)×0.0038147`` A (bidirectional)
   * - ``PID_AUX_BATTERY_VOLTAGE`` (``0x15EE``)
     - EV ECU ``0x7D2`` — 12V aux voltage (hi-res), ``u16×5/4096`` V → ``xte.v.b.12v.voltage``
   * - ``PID_AUX_BATTERY_TEMP`` (``0x15F8``)
     - EV ECU ``0x7D2`` — 12V aux temp, ``(raw-400)×0.1`` °C
   * - ``PID_AUX_BATTERY_FULL_CHARGE`` (``0x15E5``)
     - EV ECU ``0x7D2`` — 12V aux capacity (CAC), ``u8×0.5`` Ah
   * - ``PID_AUX_BATTERY_INTEGRATORS`` (``0x15E8``)
     - EV ECU ``0x7D2`` — 12V lifetime charge/discharge Ah + ready-on hours
```

- [ ] **Step 3: Commit**

```bash
git add -A && git commit -m "etnga: document 12V telemetry PIDs and changes.txt entry"
```

---

### Task 5: CI build + on-vehicle validation

**Files:** none (verification only).

**Interfaces:** none.

- [ ] **Step 1: Push and confirm CI build is green**

```bash
cd /home/devuser/wt-etnga-12v
git push -u origin feature/etnga-12v-aux-health
```
Watch the GitHub Actions `ovms3-firmware` build for the pushed commit. Expected: build succeeds (this is the compile gate — there is no local build).

- [ ] **Step 2: On-vehicle validation (record results in the spec's Validation section)**

With the car Ready / charging, confirm via the OVMS shell / app:
- `metrics list v.b.12v.current` tracks a manual `obdii can2 request device 7D2 7DA 2215FD` decode (reference: `0x5D00` → +43.1 A, 2026-06-19).
- `v.e.charging12v` = yes when the DC-DC is charging (current > 0.5 A), clears when off.
- `v.e.aux12v` = yes when Ready/charging, no when asleep.
- The app/server **12V Battery History** chart shows the current trace and charging indicator.
- `xte.v.b.12v.voltage` ≈ ADC `v.b.12v.voltage` (already confirmed agreeing to 0.01 V at 14.07 V).
- `xte.v.b.12v.temp/.cac/.charge.ah/.discharge.ah/.readyon.h` are plausible (CAC ≈ 29 Ah).
- No new `IncomingPollError` for the EV-ECU 12V PIDs in DRIVING/charge; no CAN2 wedge.

- [ ] **Step 3: Open the PR** once CI is green and on-vehicle checks pass (single-purpose: e-TNGA 12V telemetry).

---

## Self-Review

**Spec coverage:** voltage (existing, Task 1) ✓; current → `v.b.12v.current` (Task 1) ✓; `charging12v`/`aux12v` flags (Task 2) ✓; temp/cac (existing, Task 1) ✓; `0x15E8` charge/discharge Ah + ready-on hours (Task 3) ✓; rebase + offset-block reformat (Task 1) ✓; changes.txt + docs (Task 4) ✓; CI + on-vehicle validation incl. AWAKE open-question deferral (Task 5) ✓. No gaps.

**Placeholder scan:** no TBD/TODO/"handle edge cases"/"similar to" — all steps carry real code or exact commands.

**Type consistency:** `DecodeAux12vIntegrators(const std::string&)` decl (Task 3 Step 1) matches definition (Step 3) and call site (Step 4). Metric members `m_v_bat_12v_charge_ah/discharge_ah/readyon_h` consistent across header, init, decode, and clear. `SetAux12vCurrent` signature unchanged (Task 2). `GetRxBUint32`/`GetRxBUint16` match the inline helper signatures in the header.
