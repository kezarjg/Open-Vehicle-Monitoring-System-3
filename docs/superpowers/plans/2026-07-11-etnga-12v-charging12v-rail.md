# e-TNGA 12V Reference Fix (#147) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Derive e-TNGA's `v.e.charging12v` from the module's 12V rail voltage instead of the CAN-sourced DC-DC current, so the base class can never latch a DC-DC float voltage (~14.1 V) as the aux battery's *resting* reference.

**Architecture:** A single new `UpdateCharging12v()` runs every second from `OvmsVehicleToyotaETNGA::Ticker1()`, reading `v.b.12v.voltage` (the module's own ADC, `ovms_housekeeping.cpp:90` — the one signal that does not travel over CAN) and applying hysteresis. It becomes the flag's **only** writer; the three existing writers are removed. `OvmsVehicle::VehicleTicker1()` calls the vehicle's `Ticker1()` at `vehicle.cpp:875` and then runs the base 12V monitor at `vehicle.cpp:947`, so the base sees the fresh value in the same tick — no new hook required.

**Tech Stack:** C++ (ESP-IDF 3.3, legacy `make`). Component `vehicle/OVMS.V3/components/vehicle_toyota_etnga`.

**Spec:** `docs/superpowers/specs/2026-07-11-etnga-12v-ref-design.md`
**Branch:** `fix/etnga-12v-charging12v-rail` (worktree `/home/devuser/wt-etnga-12v-ref`, off `origin/master`)

## Global Constraints

- **No framework changes.** `components/vehicle/vehicle.cpp`, `components/can/`, and `components/powermgmt/` are **not** touched. Every other vehicle keeps today's behaviour. Fork-local, e-TNGA only.
- **No host test suite exists.** Per CLAUDE.md, firmware compiles only in CI (GitHub Actions) or the devcontainer, and "tests" run on-device. There is no red/green TDD cycle available for this change. The verification gates are (1) the CI `Firmware build` workflow and (2) an on-vehicle drive-and-park cycle. **Do not invent or add host unit tests** — there is no harness to run them.
- **No `changes.txt` entry.** Per CLAUDE.md, `changes.txt` is for changes users must act on or notice — new features, config changes. This is a plain bug fix requiring no user action and adds no config keys, so per maintainer guidance it is deliberately omitted.
- **Match the surrounding file style exactly** (4-space indent, brace placement, comment density as found in each file).
- Thresholds are hardcoded `static const float` file-locals — **not** config keys (YAGNI, and matches the existing `AUX_12V_WAKE_SET_V` pattern).

---

### Task 1: Make the 12V rail voltage the single source of `v.e.charging12v`

This is one atomic change. Splitting "add the new writer" from "remove the old writers" would leave an intermediate commit where both rules fight each other, which is worse than either.

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h:406`
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_metrics.cpp:552-559`
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.cpp:235-237`
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp:384` and `:414`

**Interfaces:**
- Produces: `void OvmsVehicleToyotaETNGA::UpdateCharging12v()` — no args, no return. Reads `StandardMetrics.ms_v_bat_12v_voltage`, writes `StandardMetrics.ms_v_env_charging12v`. Called once per second from `Ticker1()`.
- Consumes: nothing from other tasks.

- [ ] **Step 1: Declare the new method in the header**

In `src/vehicle_toyota_etnga.h`, the aux-12V setters are declared together at lines 403-406. Add the new method directly after `SetAux12vFullCharge`:

```cpp
    void SetAux12vCurrent(float v);
    void SetAux12vVoltage(float v);
    void SetAux12vTemperature(float v);
    void SetAux12vFullCharge(float v);
    void UpdateCharging12v();
```

- [ ] **Step 2: Add the constants and the new method, and strip `charging12v` out of `SetAux12vCurrent`**

In `src/etnga_metrics.cpp`, replace this exact block (currently at lines 552-559):

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

with:

```cpp
// Aux-12V charge detection comes from the rail voltage (v.b.12v.voltage — the module's own ADC),
// NOT from the CAN-sourced DC-DC current: the current tapers below any sane threshold while the car
// is still running and the rail is still held at float, and it is unavailable entirely if the bus
// drops. A resting lead-acid aux never exceeds ~13V, so a rail above that means the DC-DC is holding
// it up. Hysteresis stops the flag chattering as the rail decays after shutdown.
static const float AUX_12V_CHARGING_ON_V  = 13.2f;  // rail above resting -> DC-DC is charging
static const float AUX_12V_CHARGING_OFF_V = 12.9f;  // rail back at rest  -> not charging

void OvmsVehicleToyotaETNGA::UpdateCharging12v()
{
    float v = StandardMetrics.ms_v_bat_12v_voltage->AsFloat();
    if (v <= 0.0f)  // ADC not ready yet
        return;

    bool charging = StandardMetrics.ms_v_env_charging12v->AsBool();
    if (!charging && v > AUX_12V_CHARGING_ON_V)
        charging = true;
    else if (charging && v < AUX_12V_CHARGING_OFF_V)
        charging = false;

    StandardMetrics.ms_v_env_charging12v->SetValue(charging);
}

void OvmsVehicleToyotaETNGA::SetAux12vCurrent(float v)
{
    StandardMetrics.ms_v_bat_12v_current->SetValue(v);
    // Any valid EV-ECU 12V reply means the aux system is energized.
    StandardMetrics.ms_v_env_aux12v->SetValue(true);
}
```

Note the `v > 0.0f` guard: `HousekeepingUpdate12V()` (`main/ovms_housekeeping.cpp:96`) writes `0` when the ADC reads below 1.0 V, so a zero means "no reading", not "flat battery". Returning early leaves the flag at its last value rather than forcing it false.

- [ ] **Step 3: Call it from `Ticker1()`**

In `src/vehicle_toyota_etnga.cpp`, `Ticker1()` begins at line 235. Insert the call as the first statement:

```cpp
void OvmsVehicleToyotaETNGA::Ticker1(uint32_t ticker)
{
    // Aux-12V charge state is derived from the rail voltage every tick, independent of CAN and poll
    // state. The base 12V monitor in VehicleTicker1() samples v.b.12v.voltage.ref from this flag
    // later in the same tick, so it must be fresh before that runs.
    UpdateCharging12v();

    if (StandardMetrics.ms_v_charge_inprogress->AsBool()) {
```

(The existing `if (StandardMetrics.ms_v_charge_inprogress->AsBool())` block and everything after it is unchanged.)

- [ ] **Step 4: Remove the two poll-state writers**

In `src/etnga_poll_states.cpp`, this exact three-line sequence appears **twice** — once in the transition to SLEEP (line ~384) and once in the transition to AWAKE (line ~414):

```cpp
    StandardMetrics.ms_v_bat_12v_current->Clear();
    StandardMetrics.ms_v_env_charging12v->SetValue(false);
    StandardMetrics.ms_v_env_aux12v->SetValue(false);
```

Replace **both** occurrences with:

```cpp
    StandardMetrics.ms_v_bat_12v_current->Clear();
    StandardMetrics.ms_v_env_aux12v->SetValue(false);
```

Use `replace_all` — the blocks are byte-identical. Leave the preceding comment ("12V aux metrics come only from the EV ECU…") and every other line in both blocks untouched; it still correctly describes the remaining PID-sourced metrics.

These two writes are not merely redundant under the new rule, they are **actively wrong**: entering AWAKE while the car is running at 14 V would force the flag false and re-open the bug.

- [ ] **Step 5: Verify no `charging12v` writers remain outside `UpdateCharging12v()`**

Run:

```bash
cd /home/devuser/wt-etnga-12v-ref
grep -rn "ms_v_env_charging12v" vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/
```

Expected: exactly **two** hits, both inside `UpdateCharging12v()` in `etnga_metrics.cpp` (one `AsBool()` read, one `SetValue()` write). Any hit in `etnga_poll_states.cpp`, or inside `SetAux12vCurrent`, means a step was missed.

- [ ] **Step 6: Commit**

```bash
cd /home/devuser/wt-etnga-12v-ref
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.cpp \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_metrics.cpp \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp
git commit -m "etnga: derive v.e.charging12v from the 12V rail voltage, not DC-DC current

The base class samples v.b.12v.voltage.ref when its calmdown ticker
expires, assuming charging12v==false means the rail is at rest. e-TNGA
set charging12v from the CAN-sourced DC-DC current (>0.5A), which goes
false while the car is still running and the rail is still held at ~14.1V
float — and cannot be true at all if the CAN bus drops. Either way the
'resting' reference latched at float voltage, so a healthy 12.27V battery
read as critical and powermgmt armed a 30-minute auto-DeepSleep.

Derive the flag from v.b.12v.voltage (the module's own ADC) with
hysteresis instead. It stays truthful through both a current taper and a
CAN outage, and makes UpdateCharging12v() the flag's single writer.

Fixes #147"
```

---

### Task 2: Prove it compiles (CI is the only build gate)

Firmware does not build on this host. CI is the build.

**Files:** none (CI only).

- [ ] **Step 1: Push the branch**

```bash
cd /home/devuser/wt-etnga-12v-ref
git push -u origin fix/etnga-12v-charging12v-rail
```

- [ ] **Step 2: Open the PR**

```bash
gh pr create --repo kezarjg/Open-Vehicle-Monitoring-System-3 \
  --base master --head fix/etnga-12v-charging12v-rail \
  --title "etnga: derive v.e.charging12v from the 12V rail voltage (fixes #147)" \
  --body "Fixes #147. Design: \`docs/superpowers/specs/2026-07-11-etnga-12v-ref-design.md\`

The base class samples \`v.b.12v.voltage.ref\` when its calmdown ticker expires, assuming \`charging12v == false\` implies the rail is at rest (\`vehicle.cpp:947-961\`). e-TNGA derived that flag from the CAN-sourced DC-DC current (\`> 0.5A\`, \`etnga_metrics.cpp:556\`), which:

1. goes false mid-drive as the current tapers, while the rail is still held at ~14.1V float; and
2. cannot be true at all if the CAN bus drops (which is what happened on 2026-07-11 — see #148).

Either route latches the reference at float voltage (14.09V observed), so a healthy 12.27V resting battery trips \`12V Battery critical\` and powermgmt arms a 30-minute auto-DeepSleep.

This derives the flag from \`v.b.12v.voltage\` — the module's own ADC, the one signal that doesn't travel over CAN — with 13.2V/12.9V hysteresis, and makes \`UpdateCharging12v()\` its single writer. No framework changes; e-TNGA only.

The corrupted persisted reference self-heals on the first drive-and-park cycle after this ships.

On-vehicle validation pending."
```

- [ ] **Step 3: Wait for CI and confirm the firmware build passed**

```bash
gh run list --repo kezarjg/Open-Vehicle-Monitoring-System-3 \
  --branch fix/etnga-12v-charging12v-rail --limit 5 \
  --json workflowName,status,conclusion,headSha
```

Expected: `Firmware build` with `"conclusion": "success"`.

Note per CLAUDE.md: `Docs build` runs here too (this branch touches `docs/**`), and its Pages `deploy` job is **skipped on PRs** by design — that is not a failure. Only `Firmware build` gates this change. If `Firmware build` fails, read the log (`gh run view <run-id> --log-failed`), fix, and re-push — do not proceed to Task 3 on a red build.

---

### Task 3: On-vehicle validation (the real gate)

This is the only test that can actually prove the fix. It requires a drive-and-park cycle on the physical car.

**Files:** none.

- [ ] **Step 1: Flash the CI build to the module**

Follow `CLAUDE.local.md`. The CI binary lands in MinIO at `ci-artifacts/<run_id>/ovms3/ovms3.bin` (**not** GitHub artifacts). Serve it from `os-k3s` and flash with `ota flash http`, minding the slot-stick rule (`ota flash http` writes the **inactive** slot; the launcher autostarts `default_slot` = `ota_1`).

- [ ] **Step 2: Record the pre-drive baseline**

```bash
ssh solterra-ovms 'metrics list v.b.12v.voltage'
```

Record `v.b.12v.voltage`, `.ref`, and `.alert`. (The reference was manually set to 12.6 V on 2026-07-11; the point of this test is that it stays sane on its own.)

- [ ] **Step 3: Drive, then park**

During the drive, `v.e.charging12v` must be **`yes` continuously**:

```bash
ssh solterra-ovms 'metrics list v.e.charging12v'
```

- [ ] **Step 4: Check the log for the flapping signature**

Pull the log (use the `ovms-log-pull` skill) and grep:

```bash
grep -cE "Charging 12V battery|No longer charging 12V battery" ~/ovms-logs/log.txt
```

Expected: **no rapid alternating pairs during the drive.** Before the fix these appeared every 11–22 seconds. One transition on power-up and one on shutdown is correct and expected; a burst of them is the bug.

- [ ] **Step 5: Confirm the reference lands at resting voltage**

After parking, wait for the calmdown to expire (up to 15 minutes), then:

```bash
ssh solterra-ovms 'metrics list v.b.12v.voltage'
```

Expected:
- `v.b.12v.voltage.ref` ≈ **12.3–12.7 V** (resting) — **not** ~14 V.
- `v.b.12v.voltage.alert` = **no**.
- No `12V Battery critical` notification, and no `powermgmt: 12V battery alert detected` in the log.

- [ ] **Step 6: Regression check — the 12V wake trigger still works**

The wake trigger fires on `v12 > ref + 0.2` (`etnga_poll_states.cpp:76`), so a sane reference should restore it. On the next power-on, the log should show:

```
v-toyota-etnga: Aux 12V has exceeded the threshold
```

- [ ] **Step 7: Record the result on #147 and merge**

Post the observed `ref`, `alert`, and the flapping-grep count as a comment on #147. **Only merge once the on-vehicle numbers are in** — per CLAUDE.md, do not claim behaviour that hasn't been verified on real hardware.

---

## Self-Review

**Spec coverage:** Every section of the spec maps to a task. The rule and hysteresis constants → Task 1 Step 2. Ordering (Ticker1 before the base monitor) → Task 1 Step 3. Single-owner table (all three writers) → Task 1 Steps 2, 4, verified in Step 5. Verification (CI + on-vehicle drive-and-park, plus the wake-trigger regression check) → Tasks 2 and 3. Out-of-scope items (base class, wake trigger, #148) → Global Constraints. Self-healing reference → asserted in the PR body and measured in Task 3 Step 5.

**Placeholder scan:** No TBDs. Every code step contains the complete literal code. No "add error handling" hand-waving — the only guard (`v <= 0.0f`) is spelled out with its reason.

**Type consistency:** `UpdateCharging12v()` is declared `void`, no args, in Task 1 Step 1 and defined identically in Step 2; called with no args in Step 3. Constants `AUX_12V_CHARGING_ON_V` / `AUX_12V_CHARGING_OFF_V` are named identically in the definition and both uses.
