# e-TNGA Adaptive Sleep Cooldown Backoff — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce the parked 12V duty cycle by escalating the sleep cooldown on consecutive no-activity sleeps, while keeping the 12V-rise wake as a responsive escape.

**Architecture:** A single escalation index (`m_sleep_backoff_idx`) selects a cooldown from a hardcoded schedule `{10,30,60,120,300}`s. Cooldown arming is centralized in `TransitionToSleepState()`; real activity (drive / charge / charge-door / 12V wake) resets the index to 0. All logic lives in `etnga_poll_states.cpp` + 2 header members.

**Tech Stack:** C++ (ESP-IDF 3.3, legacy `make`), OVMS3 vehicle framework. Component: `vehicle/OVMS.V3/components/vehicle_toyota_etnga`.

**Spec:** `docs/superpowers/specs/2026-06-04-etnga-sleep-cooldown-backoff-design.md`

---

## Testing note (read first)

This is a hardware project with **no host unit-test suite** (per `CLAUDE.md`: "tests run
on-device"). The per-task verification here is therefore a **clean incremental build**
(`make`), not a unit test. Final acceptance is **on-vehicle log observation** (Task 5),
which is the spec's acceptance gate. Do not look for or invent a host test harness.

All work happens in the existing worktree on branch `feature/etnga-sleep-cooldown-backoff`
(`/home/devuser/wt-etnga-sleep-cooldown`). Improvement #1 (info-level sleep logs) is already
committed on this branch.

## Build prerequisite (one-time, before Task 1's build step)

The worktree needs an `sdkconfig` with the e-TNGA vehicle enabled before `make` will work:

```bash
cd /home/devuser/wt-etnga-sleep-cooldown/vehicle/OVMS.V3
# Reuse the main checkout's config if present, else seed + select:
cp /home/devuser/Open-Vehicle-Monitoring-System-3/vehicle/OVMS.V3/sdkconfig sdkconfig 2>/dev/null \
  || cp support/sdkconfig.default.hw31 sdkconfig
# Ensure CONFIG_OVMS_VEHICLE_TOYOTA_ETNGA=y (via `make menuconfig` → OVMS - Vehicle Support,
# or confirm it is already set):
grep -q '^CONFIG_OVMS_VEHICLE_TOYOTA_ETNGA=y' sdkconfig && echo "etnga enabled" || echo "ENABLE etnga in menuconfig"
```

**Build command used as verification throughout:**
`cd /home/devuser/wt-etnga-sleep-cooldown/vehicle/OVMS.V3 && make -j$(nproc)`
Expected: compiles `etnga_poll_states.o`, links `build/ovms3.bin`, exits 0. (First build is
slow; later builds are incremental and only recompile the changed file.)

---

## Task 1: Add header state + helper declaration

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h`

- [ ] **Step 1: Add the two backoff members**

Find:
```cpp
    bool m_allow_wake = true;  // Used to implement a cooldown timer if the vehicle is put into sleep
    int m_sleep_entry_time = 0;  // Used to track the time that cooldown timer started
```
Replace with:
```cpp
    bool m_allow_wake = true;  // Used to implement a cooldown timer if the vehicle is put into sleep
    int m_sleep_entry_time = 0;  // Used to track the time that cooldown timer started
    int m_sleep_cooldown_secs = 10;  // Cooldown window (s) that applied to the current sleep
    int m_sleep_backoff_idx = 0;     // Index into SLEEP_COOLDOWN_SECS; escalates on consecutive no-activity sleeps
```

- [ ] **Step 2: Declare the reset helper**

Find:
```cpp
    void TransitionToChargeDcState();

    void RequestVIN();
```
Replace with:
```cpp
    void TransitionToChargeDcState();

    void ResetSleepBackoff();   // reset cooldown escalation to the base step (real activity seen)

    void RequestVIN();
```

- [ ] **Step 3: Build to verify it still compiles** (unused members/undeclared-but-uncalled helper are fine)

Run: `cd /home/devuser/wt-etnga-sleep-cooldown/vehicle/OVMS.V3 && make -j$(nproc)`
Expected: exit 0. (A header-only change with no new references compiles cleanly.)

- [ ] **Step 4: Commit**

```bash
git -C /home/devuser/wt-etnga-sleep-cooldown add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h
git -C /home/devuser/wt-etnga-sleep-cooldown commit -m "etnga: add sleep cooldown backoff state + reset helper decl"
```

---

## Task 2: Schedule, reset helper, and centralized cooldown arming

This adds the schedule + `ResetSleepBackoff()`, moves cooldown arming into
`TransitionToSleepState()`, and removes the now-duplicated arming from the three forced-sleep
sites. After this task the index escalates but `HandleSleepState` still uses the old fixed
`> 10` check (changed in Task 3) — intermediate state compiles and is safe.

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp`

- [ ] **Step 1: Add schedule + ResetSleepBackoff() above HandleSleepState**

Find:
```cpp
//    CHARGE_DC (6)         : DC fast charging in progress

void OvmsVehicleToyotaETNGA::HandleSleepState()
```
Replace with:
```cpp
//    CHARGE_DC (6)         : DC fast charging in progress

// Escalating sleep cooldown schedule (seconds). Each consecutive no-activity sleep uses the
// next entry; real activity (drive/charge/charge-door/12V wake) resets to [0]. Caps at the
// last entry. See ResetSleepBackoff() and TransitionToSleepState().
static const int SLEEP_COOLDOWN_SECS[] = {10, 30, 60, 120, 300};
static const int SLEEP_COOLDOWN_STEPS  = sizeof(SLEEP_COOLDOWN_SECS) / sizeof(SLEEP_COOLDOWN_SECS[0]);

void OvmsVehicleToyotaETNGA::ResetSleepBackoff()
{
    m_sleep_backoff_idx = 0;
}

void OvmsVehicleToyotaETNGA::HandleSleepState()
```

- [ ] **Step 2: Centralize cooldown arming in TransitionToSleepState**

Find:
```cpp
void OvmsVehicleToyotaETNGA::TransitionToSleepState()
{
    m_armed_for_charge = false;
    SetPollState(PollState::SLEEP);
    SetAwake(false);
}
```
Replace with:
```cpp
void OvmsVehicleToyotaETNGA::TransitionToSleepState()
{
    int monotonic = StandardMetrics.ms_m_monotonic->AsInt();
    m_armed_for_charge = false;
    // Arm the escalating cooldown: ignore CAN-frame wakes for the current window, then step
    // the index up (clamped) so the next consecutive no-activity sleep waits longer.
    // ResetSleepBackoff() returns the index to 0 on real activity.
    m_sleep_entry_time = monotonic;
    m_sleep_cooldown_secs = SLEEP_COOLDOWN_SECS[m_sleep_backoff_idx];
    m_allow_wake = false;
    if (m_sleep_backoff_idx < SLEEP_COOLDOWN_STEPS - 1)
        m_sleep_backoff_idx++;
    SetPollState(PollState::SLEEP);
    SetAwake(false);
}
```

- [ ] **Step 3: Remove duplicated arming — door-watch forced sleep**

Find:
```cpp
            ESP_LOGI(TAG, "Vehicle awake for over 300s with no activity — forcing sleep state");
            m_sleep_entry_time = monotonic;
            m_allow_wake = false;
            TransitionToSleepState();
```
Replace with:
```cpp
            ESP_LOGI(TAG, "Vehicle awake for over 300s with no activity — forcing sleep state");
            TransitionToSleepState();
```

- [ ] **Step 4: Remove duplicated arming — cable-watch forced sleep**

Find:
```cpp
        ESP_LOGI(TAG, "Armed 15min, no cable plug-in — giving up");
        m_sleep_entry_time = monotonic;
        m_allow_wake = false;
        TransitionToSleepState();
```
Replace with:
```cpp
        ESP_LOGI(TAG, "Armed 15min, no cable plug-in — giving up");
        TransitionToSleepState();
```

- [ ] **Step 5: Remove duplicated arming — charge-wait bus-dead sleep**

Find:
```cpp
        // Bus went dead during scheduled wait (OBC slept or gateway isolated OBD)
        m_sleep_entry_time = StandardMetrics.ms_m_monotonic->AsInt();
        m_allow_wake = false;
        TransitionToSleepState();
```
Replace with:
```cpp
        // Bus went dead during scheduled wait (OBC slept or gateway isolated OBD)
        TransitionToSleepState();
```

- [ ] **Step 6: Build**

Run: `cd /home/devuser/wt-etnga-sleep-cooldown/vehicle/OVMS.V3 && make -j$(nproc)`
Expected: exit 0. (Note: `monotonic` is still used elsewhere in `HandleAwakeState` — lines for
`m_cable_watch_start` and `m_v_env_awaketime` — so no unused-variable warning.)

- [ ] **Step 7: Commit**

```bash
git -C /home/devuser/wt-etnga-sleep-cooldown add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp
git -C /home/devuser/wt-etnga-sleep-cooldown commit -m "etnga: centralize escalating sleep cooldown in TransitionToSleepState"
```

---

## Task 3: Apply the cooldown window + reset on 12V wake

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp`

- [ ] **Step 1: Use the per-sleep cooldown in HandleSleepState + log the duration**

Find:
```cpp
    if (!m_allow_wake)
    {
        if ((monotonic - m_sleep_entry_time) > 10)
        {
            ESP_LOGI(TAG, "Cooling off period ended, allowing wake");
            m_allow_wake = true;
        }
    }
```
Replace with:
```cpp
    if (!m_allow_wake)
    {
        if ((monotonic - m_sleep_entry_time) > m_sleep_cooldown_secs)
        {
            ESP_LOGI(TAG, "Cooling off period ended (%ds), allowing wake", m_sleep_cooldown_secs);
            m_allow_wake = true;
        }
    }
```

- [ ] **Step 2: Reset backoff on the 12V-rise wake (the responsive escape)**

Find:
```cpp
        ESP_LOGI(TAG, "Aux 12V has exceeded the threshold");
        // Send a CAN reset.
        esp_err_t result = m_can2->Reset();
```
Replace with:
```cpp
        ESP_LOGI(TAG, "Aux 12V has exceeded the threshold");
        // Real power-up — resume responsive cooldowns.
        ResetSleepBackoff();
        // Send a CAN reset.
        esp_err_t result = m_can2->Reset();
```

- [ ] **Step 3: Build**

Run: `cd /home/devuser/wt-etnga-sleep-cooldown/vehicle/OVMS.V3 && make -j$(nproc)`
Expected: exit 0.

- [ ] **Step 4: Commit**

```bash
git -C /home/devuser/wt-etnga-sleep-cooldown add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp
git -C /home/devuser/wt-etnga-sleep-cooldown commit -m "etnga: apply escalating cooldown window + reset on 12V wake"
```

---

## Task 4: Reset backoff on the remaining real-activity events

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp`

- [ ] **Step 1: Reset on entering DRIVING**

Find:
```cpp
void OvmsVehicleToyotaETNGA::TransitionToDrivingState()
{
    m_armed_for_charge = false;
    SetPollState(PollState::DRIVING);
```
Replace with:
```cpp
void OvmsVehicleToyotaETNGA::TransitionToDrivingState()
{
    m_armed_for_charge = false;
    ResetSleepBackoff();   // real drive — resume responsive cooldowns
    SetPollState(PollState::DRIVING);
```

- [ ] **Step 2: Reset on entering CHARGE_HANDSHAKE**

Find:
```cpp
void OvmsVehicleToyotaETNGA::TransitionToChargeHandshakeState()
{
    m_armed_for_charge = false;  // arm state consumed on entering handshake
    m_charge_state_entry = StandardMetrics.ms_m_monotonic->AsInt();
```
Replace with:
```cpp
void OvmsVehicleToyotaETNGA::TransitionToChargeHandshakeState()
{
    m_armed_for_charge = false;  // arm state consumed on entering handshake
    ResetSleepBackoff();   // charge session beginning — resume responsive cooldowns
    m_charge_state_entry = StandardMetrics.ms_m_monotonic->AsInt();
```

- [ ] **Step 3: Reset when the charge door opens (arm)**

Find:
```cpp
        if (lid_open) {
            // Charge door just opened: arm and start the 15-min cable watch
            m_armed_for_charge = true;
            m_cable_watch_start = monotonic;
        }
```
Replace with:
```cpp
        if (lid_open) {
            // Charge door just opened: arm and start the 15-min cable watch
            m_armed_for_charge = true;
            m_cable_watch_start = monotonic;
            ResetSleepBackoff();   // deliberate user action — resume responsive cooldowns
        }
```

- [ ] **Step 4: Clean full build**

Run: `cd /home/devuser/wt-etnga-sleep-cooldown/vehicle/OVMS.V3 && make -j$(nproc)`
Expected: exit 0, links `build/ovms3.bin`.

- [ ] **Step 5: Commit**

```bash
git -C /home/devuser/wt-etnga-sleep-cooldown add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp
git -C /home/devuser/wt-etnga-sleep-cooldown commit -m "etnga: reset sleep backoff on drive/charge/charge-door activity"
```

---

## Task 5: On-vehicle acceptance (manual, post-merge-candidate)

No host tests exist; this is the real acceptance gate from the spec. Not run in-session —
record results when flashed to the module.

- [ ] **Step 1: Flash the build to the module**

Per memory `howto_ovms_fast_deploy`: install via `ota flash http` from os-k3s (`sd mount`
first), reachable through the `solterra-ovms` SSH alias. Do **not** use destructive `module`
commands.

- [ ] **Step 2: Confirm the ramp**

Park and let it cycle; in `/sd/logs/log.txt` confirm successive lines:
`Cooling off period ended (10s), allowing wake` → `(30s)` → `(60s)` → `(120s)` → `(300s)`.

- [ ] **Step 3: Confirm 12V reset**

Trigger a 12V-rise wake; confirm `Aux 12V has exceeded the threshold` followed by the **next**
`Cooling off period ended (10s…)` (index reset to base).

- [ ] **Step 4: Confirm activity resets**

Confirm a real drive (DRIVING) and a charge plug-in each return the next cooldown to `(10s)`.

- [ ] **Step 5: (Optional) measure drain**

Compare `ms_v_bat_12v_voltage` across parked windows before/after for actual sag improvement.

---

## changes.txt (deferred — confirm with maintainer)

e-TNGA has no `changes.txt` presence yet (in-development vehicle). Per the earlier decision,
hold the release-note entry until the vehicle is announced as a whole, or add a bullet now if
preferred. Not included as a code task.

---

## Self-review (completed by plan author)

- **Spec coverage:** schedule → T2; members → T1; centralized arming + duplicate removal (3
  sites) → T2; cooldown window + duration log → T3; 12V reset → T3; DRIVING/charge/charge-door
  resets → T4; on-vehicle validation → T5; #1 logging already committed. All spec sections map
  to a task.
- **Placeholder scan:** none — every code step shows full find/replace text.
- **Type consistency:** `m_sleep_backoff_idx`, `m_sleep_cooldown_secs`, `SLEEP_COOLDOWN_SECS`,
  `SLEEP_COOLDOWN_STEPS`, `ResetSleepBackoff()` are spelled identically in the header (T1) and
  every use (T2–T4).
