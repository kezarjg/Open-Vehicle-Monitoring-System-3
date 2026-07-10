# e-TNGA awake/bus-liveness decouple — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore the e-TNGA's `v.e.awake` metric to the standard "switched on by the user" meaning by moving CAN2 bus-liveness into an internal `m_last_can2_time` member, so framework consumers stop reporting the car as on/idle during a charge.

**Architecture:** Split the two concepts currently both living in `v.e.awake`. (1) An internal `m_last_can2_time` timestamp, stamped per accepted CAN2 frame and read via `IsBusAlive()`, drives `SLEEP ↔ AWAKE`. (2) `v.e.awake` is set centrally in `SetPollState()` as a pure function of poll state (`AWAKE || DRIVING`), so it is false in SLEEP and all four CHARGE_* states.

**Tech Stack:** C++ (ESP-IDF 3.3, older GCC), OVMS metrics framework. Component: `vehicle/OVMS.V3/components/vehicle_toyota_etnga`.

## Global Constraints

- **No host test suite.** Firmware is not built or unit-tested on the host. Compile verification = GitHub Actions `build.yml` (see CLAUDE.md); behavior verification = on-vehicle. Do NOT claim a local build verified anything.
- **Feature-branch CI must be triggered explicitly:** `gh workflow run build.yml --ref feature/etnga-awake-decouple` (push trigger fires for master only). Artifacts land in MinIO `ci-artifacts/<run_id>/ovms3/ovms3.bin`.
- **Match existing file style** (the project asks contributors to match each module's style exactly).
- **Behavior-preserving for the state machine:** `SLEEP↔AWAKE` timing must stay identical — same 120 s window (`SM_STALE_MID`), same monotonic-seconds clock (`ms_m_monotonic`), same rising-edge-on-forced-sleep behavior. The only intended behavior change is the value of the `v.e.awake` metric during charge.
- **Single-purpose fork PR:** e-TNGA only; no base-class / framework edits.
- **Spec:** `docs/superpowers/specs/2026-07-10-etnga-awake-decouple-design.md`.

---

### Task 1: Decouple bus-liveness from `v.e.awake` (all code edits)

This is one atomic code change: the edits are interdependent (repointing a reader before the helper exists would not compile; setting the metric per-state while still setting it per-frame would be incoherent). Land them together in one commit.

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h` (member + method decl)
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_metrics.cpp` (`IsBusAlive()` + constant, `SetPollState` awake set, delete stale-reset block)
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_can_processor.cpp` (stamp member)
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp` (3 reads → `IsBusAlive()`, forced-sleep reset)
- Test: none on host (see Global Constraints). Static self-check + CI build.

**Interfaces:**
- Produces: `bool OvmsVehicleToyotaETNGA::IsBusAlive() const;` — true iff a CAN2 frame was accepted within `BUS_STALE_SECS` (120) seconds. `uint32_t m_last_can2_time` — monotonic seconds of the last accepted CAN2 frame, `0` when never/after forced sleep.
- Consumes: `StandardMetrics.ms_m_monotonic->AsInt()` (monotonic seconds), `enum PollState` (SLEEP/AWAKE/DRIVING/CHARGE_*), existing `SetAwake(bool)` and `m_allow_wake`.

- [ ] **Step 1: Declare the member and helper in the header**

In `src/vehicle_toyota_etnga.h`, add the member alongside the sleep/cooldown state (after line 78, `int m_sleep_backoff_idx = 0; ...`):

```cpp
    uint32_t m_last_can2_time = 0;   // monotonic secs of last accepted CAN2 frame; drives SLEEP<->AWAKE bus-liveness
```

And add the method declaration immediately after `void SetAwake(bool awake);` (line 351):

```cpp
    bool IsBusAlive() const;   // true if a CAN2 frame arrived within BUS_STALE_SECS
```

- [ ] **Step 2: Define `IsBusAlive()` and the stale constant**

In `src/etnga_metrics.cpp`, immediately after the `SetAwake` definition:

```cpp
void OvmsVehicleToyotaETNGA::SetAwake(bool awake)
{
    StandardMetrics.ms_v_env_awake->SetValue(awake);
}
```

append:

```cpp
// CAN2 bus-liveness. Mirrors the old v.e.awake ~120s auto-stale window (SM_STALE_MID)
// but is kept internal, so v.e.awake can carry the standard "switched on" meaning.
// Uses the ms_m_monotonic seconds clock (same clock as the CHARGE_WAIT drain timer).
static const uint32_t BUS_STALE_SECS = 120;

bool OvmsVehicleToyotaETNGA::IsBusAlive() const
{
    if (m_last_can2_time == 0)
        return false;
    uint32_t now = (uint32_t) StandardMetrics.ms_m_monotonic->AsInt();
    return (now - m_last_can2_time) <= BUS_STALE_SECS;
}
```

- [ ] **Step 3: Set `v.e.awake` centrally from poll state in `SetPollState`**

In `src/etnga_metrics.cpp`, in `SetPollState`, insert the awake set between the transition log and `PollSetState`:

```cpp
    ESP_LOGI(TAG, "Transitioning from the %s to the %s state", CurrentPollStateText, NextPollStateText);

    // v.e.awake reflects the standard "switched on" meaning: true only while actively
    // awake for a non-charge reason. Charge states poll with the car off -> awake stays false.
    SetAwake(state == PollState::AWAKE || state == PollState::DRIVING);

    PollSetState(state);
}
```

- [ ] **Step 4: Stamp `m_last_can2_time` instead of `SetAwake(true)` per frame**

In `src/etnga_can_processor.cpp`, `IncomingFrameCan2`, replace the `else` body (keep the `m_allow_wake` gate exactly — stamping only when wake is allowed is what preserves the cooldown semantics):

```cpp
    if (!m_allow_wake) {
        // Verbose: this fires for EVERY frame received during a cooldown window — at
        // parked-bus broadcast rates an INFO message here flooded the log.
        ESP_LOGV(TAG, "OBD message ignored during cooldown");
    } else {
        // Bus-liveness only; v.e.awake is now driven by poll state in SetPollState().
        m_last_can2_time = (uint32_t) StandardMetrics.ms_m_monotonic->AsInt();
    }
```

- [ ] **Step 5: Repoint the three SLEEP/AWAKE/CHARGE_WAIT reads to `IsBusAlive()`**

In `src/etnga_poll_states.cpp`:

Line ~65 (SLEEP handler, wake if there is life):
```cpp
    if (IsBusAlive()) {
```
(was `if (StandardMetrics.ms_v_env_awake->AsBool()) {`)

Line ~168 (AWAKE handler, sleep if bus dead):
```cpp
    if (!IsBusAlive()) {
```
(was `if (!StandardMetrics.ms_v_env_awake->AsBool()) {`)

Line ~283 (CHARGE_WAIT handler, sleep if bus dead):
```cpp
    if (!IsBusAlive()) {
```
(was `if (!StandardMetrics.ms_v_env_awake->AsBool()) {`)

Leave the surrounding comments unchanged.

- [ ] **Step 6: Reset liveness on forced sleep**

In `src/etnga_poll_states.cpp`, `TransitionToSleepState` (line ~379-380), replace the `SetAwake(false)` after `SetPollState(PollState::SLEEP)`:

```cpp
    SetPollState(PollState::SLEEP);   // also sets v.e.awake = false
    m_last_can2_time = 0;             // force rising-edge: next wake needs a fresh CAN2 frame
```

Rationale: `SetPollState(SLEEP)` now sets `v.e.awake` false (Step 3), and the cooldowns `{10,30,60,120,300}` include values ≤120 s, so without zeroing the timestamp a forced sleep taken while the bus is alive would satisfy `IsBusAlive()` again the instant the cooldown lifts and re-wake without a fresh frame.

- [ ] **Step 7: Delete the awake stale-reset block**

In `src/etnga_metrics.cpp`, `ResetStaleMetrics`, delete this block entirely (lines ~87-91):

```cpp
    // Check to make sure the 'awake' signal has been updated recently
    if (StandardMetrics.ms_v_env_awake->IsStale() && StandardMetrics.ms_v_env_awake->AsBool()) {
        ESP_LOGD(TAG, "Awake is stale. Manually setting to off");
        SetAwake(false);
    }
```

`v.e.awake` is now poll-state-driven (never needs a stale reset); bus staleness lives in `IsBusAlive()`. Leave the `controlstate` stale check above it and the chargeport-door stale logic below it untouched.

- [ ] **Step 8: Static self-check — no stray `v.e.awake` bus-liveness references remain**

Run:
```bash
cd vehicle/OVMS.V3/components/vehicle_toyota_etnga
grep -rn "ms_v_env_awake" src/
grep -rn "SetAwake" src/
```
Expected `ms_v_env_awake`: exactly **one** hit — the setter body in `etnga_metrics.cpp` (`SetValue(awake)`).
Expected `SetAwake`: the header decl, the definition, the new `SetPollState` call, and the two init calls (`etnga_metrics.cpp` init, `vehicle_toyota_etnga.cpp:165`). No `SetAwake(true)`, no `SetAwake` in `can_processor` or in `TransitionToSleepState`.

If anything else appears, reconcile before committing.

- [ ] **Step 9: Commit**

```bash
cd /home/devuser/wt-etnga-awake-decouple
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_metrics.cpp \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_can_processor.cpp \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp
git commit -m "etnga: decouple CAN2 bus-liveness from v.e.awake

Move bus-liveness to an internal m_last_can2_time member (IsBusAlive(),
120s window) that drives SLEEP<->AWAKE. Set v.e.awake centrally in
SetPollState() as (AWAKE || DRIVING), so it is false during all CHARGE_*
states. Fixes the hourly 'Vehicle is idling' notifications that fired all
through a charge because v.e.awake was held true to keep the poller running."
```

---

### Task 2: Docs — `changes.txt` and state-machine notes

**Files:**
- Modify: `vehicle/OVMS.V3/changes.txt`
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/docs/state_machine.rst`

**Interfaces:** none (documentation only).

- [ ] **Step 1: Add the `changes.txt` entry**

Add a new dated entry at the top of `vehicle/OVMS.V3/changes.txt` following the existing format (date/author header, `-` bullets). It is a user-noticeable behavior change, so it belongs here:

```
2026-07-10 (Jerry Kezar)
	Toyota e-TNGA / Subaru Solterra:
	- v.e.awake now reflects "vehicle switched on" (AWAKE/DRIVING) rather than raw
	  CAN2 bus activity. It no longer reads true during a charge, so the module no
	  longer emits the periodic "Vehicle is idling / stopped turned on" notification
	  while parked and charging. Internal CAN2 bus-liveness (sleep/wake timing) is
	  unchanged.
```

(Match the surrounding entries' exact leading-tab/indent style.)

- [ ] **Step 2: Update the state-machine doc**

In `components/vehicle_toyota_etnga/docs/state_machine.rst`, update the "Two views of 'vehicle on'" note and the "`env_awake` is never explicitly cleared" note so they describe the new split: internal `m_last_can2_time` / `IsBusAlive()` (120 s window) drives `SLEEP↔AWAKE`; the `v.e.awake` metric is set in `SetPollState()` as `AWAKE || DRIVING` and is false in SLEEP and all CHARGE_* states. Keep the surrounding prose/format; adjust only the two notes that referenced `env_awake` as the bus-liveness signal.

- [ ] **Step 3: Commit**

```bash
cd /home/devuser/wt-etnga-awake-decouple
git add vehicle/OVMS.V3/changes.txt \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/docs/state_machine.rst
git commit -m "docs: etnga — record v.e.awake semantic change and bus-liveness split"
```

---

### Task 3: CI build verification

**Files:** none (verification only).

- [ ] **Step 1: Push the branch**

```bash
cd /home/devuser/wt-etnga-awake-decouple
git push -u origin feature/etnga-awake-decouple
```

- [ ] **Step 2: Trigger the firmware build (feature branches don't auto-build on push)**

```bash
gh workflow run build.yml --ref feature/etnga-awake-decouple
```

- [ ] **Step 3: Watch it to green**

```bash
gh run list --workflow=build.yml --branch feature/etnga-awake-decouple --limit 3 \
  --json databaseId,status,conclusion,headSha
# then: gh run watch <databaseId>   (or re-list until status=completed)
```
Expected: `conclusion = success`. Note the `run_id` — the bench build is at MinIO `ci-artifacts/<run_id>/ovms3/ovms3.bin`. If the build fails to compile, fix in Task 1 and re-run.

---

### Task 4: On-vehicle validation (run on the module; cannot be done in this environment)

**Files:** none. This is the behavior gate. Flash the Task 3 build to the bench/vehicle module (see CLAUDE.local.md slot-stick flashing) and confirm each item. Capture via `metrics list v.e.` / `stat` over SSH and by pulling `log.txt` after each window.

- [ ] **Step 1: Sleep/wake timing unchanged (parked, no charge)**
  - Park with bus quiet → module enters SLEEP ~120 s after the last CAN2 frame (watch for `CAN bus idle (env_awake cleared) — sleeping` is now `... (bus idle) ...` per any log wording you kept; confirm the ~120 s latency).
  - Passive CAN wake (open a door) → `Transitioning from the SLEEP to the AWAKE state`; confirm `v.e.awake` becomes `yes` only on that transition.
  - Confirm 12 V rising-edge wake still works (unchanged path).

- [ ] **Step 2: AC charge — the fix**
  - Start an AC charge. Confirm `v.e.awake == no` for the whole `CHARGE_AC` window.
  - Confirm **no** `notify/alert/vehicle/idle` ("Vehicle is idling / stopped turned on") is emitted during the charge (grep the pulled log).
  - Confirm charge metrics / charge report still populate normally and there is no spurious SLEEP↔AWAKE oscillation.

- [ ] **Step 3: CHARGE_WAIT 12 V-drain path (validates the forced-sleep reset)**
  - Plug in with a delayed/scheduled charge (car in CHARGE_WAIT, not yet charging).
  - Confirm the tiered slow-poll → forced sleep → passive resume cycle still works and does **not** immediately bounce back to AWAKE after the cooldown lifts (this exercises `m_last_can2_time = 0`).

- [ ] **Step 4: Driving**
  - Drive briefly; confirm `v.e.awake == yes` and `v.e.on == yes` in DRIVING, and both return to `no` after power-off + sleep.

- [ ] **Step 5: Open the PR** once Steps 1–4 pass, targeting the fork master, single-purpose, referencing the spec and the observed 2026-07-09→10 idle-notification logs.

## Self-Review

**Spec coverage:**
- Internal bus-liveness member + `IsBusAlive()` + 120 s constant → Task 1 Steps 1-2. ✔
- `v.e.awake` set centrally in `SetPollState` as `AWAKE||DRIVING` → Task 1 Step 3. ✔
- Per-frame stamp replacing `SetAwake(true)`, preserving `m_allow_wake` gate → Task 1 Step 4. ✔
- Three reads → `IsBusAlive()` → Task 1 Step 5. ✔
- Forced-sleep `m_last_can2_time = 0` reset → Task 1 Step 6. ✔
- Delete stale-reset block → Task 1 Step 7. ✔
- Init `SetAwake(false)` calls kept (no step needed; explicitly not touched) → covered by Step 8 self-check expectation. ✔
- Behavior-preserving validation (sleep timing, 12 V wake, charge, drain path) → Task 4. ✔
- `changes.txt` + `state_machine.rst` → Task 2. ✔
- CI build gate → Task 3. ✔

**Placeholder scan:** No TBD/TODO; every code step shows exact before/after; commands have expected output. ✔

**Type consistency:** `IsBusAlive()` (const, returns bool) and `m_last_can2_time` (`uint32_t`, seconds) named identically in the header decl (Step 1), definition (Step 2), and all readers (Steps 5-6). `BUS_STALE_SECS` defined once (Step 2). `SetAwake(bool)` reused unchanged. ✔
