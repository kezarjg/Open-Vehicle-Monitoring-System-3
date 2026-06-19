# e-TNGA CHARGE_WAIT 12V-drain Fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the Toyota e-TNGA `CHARGE_WAIT` poll state from draining the vehicle 12V during long (scheduled-charge) waits, while still reliably catching the charge when it starts.

**Architecture:** Two tiers. Tier 1 slows the `CHARGE_WAIT` engage-watch PIDs from 1s to 10s (poll-table edit). Tier 2 forces the *existing* `TransitionToSleepState()` after a sustained idle wait, so polling stops, the bus idles, and 12V recovers; the charge session is preserved across the sleep and resumes via the existing passive (CAN-frame / 12V rising-edge) wake. Supporting changes: a re-sleep flag to keep oscillation duty-cycle low, resume routing that re-enters `CHARGE_WAIT` (not a fresh `HANDSHAKE`), and a PISW debounce that prevents the OBC post-wake `0x00` transient from falsely closing a session.

**Tech Stack:** C++ (ESP-IDF 3.3, older GCC). No host test suite — firmware compiles on GitHub Actions CI (push/PR), validation is on-device on a real vehicle. Per-task verification is *CI compile + correctness-by-inspection*; behavioral validation is the final on-vehicle task.

## Global Constraints

- **No host unit tests.** Do not add a host test harness. "Test the change" = it compiles on GitHub CI and is correct by inspection; real behavior is validated on-vehicle (Task 6).
- **Match surrounding code style exactly** (project rule); these files use `ESP_LOGx(TAG, ...)`, `int` monotonic seconds from `StandardMetrics.ms_m_monotonic->AsInt()`, and `m_`-prefixed members.
- **Hardcoded tuning constants** in the `static const ... SECS` style already in `etnga_poll_states.cpp` (like `SLEEP_COOLDOWN_SECS`). No new `[instance]` config params in v1.
- **PID-with-ECU rule:** all touched PIDs are on the **Plug-In Control System ECU** (`PLUG_IN_CONTROL_SYSTEM_TX/RX`): `0x1684` AC-op, `0x1666` HLC, `0x10D1` ctrl-mode, `0x1669` PISW.
- **Constants:** `CHARGE_WAIT_SLEEP_SECS = 600`, `CHARGE_WAIT_RESLEEP_SECS = 15`, WAIT engage-watch cadence = `10`, PISW cable-removed debounce = `2` consecutive AWAKE reads.
- Branch `feature/etnga-charge-wait-12v-sleep` in worktree `~/wt-etnga-charge-wait`. Commit per task.

---

### Task 1: Tier 1 — slow the CHARGE_WAIT engage-watch cadence

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.cpp` (the `obdii_polls_charge[]` table; WAIT is the 2nd value of each `{HS,WAIT,AC,DC}` tuple) — lines 101, 110, 111.

**Interfaces:**
- Consumes: nothing.
- Produces: nothing new (data-only change). Establishes that WAIT polls `0x1684`/`0x1666`/`0x10D1` at 10s.

- [ ] **Step 1: Change the three WAIT-column cadences from 1 to 10**

In `obdii_polls_charge[]`, edit only the WAIT (2nd) column of these three rows:

`0x10D1` ctrl-mode (line ~101): `{ 1,  1,  1,  1}` → `{ 1, 10,  1,  1}`
```cpp
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CONTROL_SYSTEM_MODE,        { 1, 10,  1,  1}, 0, ISOTP_STD }, // 0x10D1 ctrl-mode: WAIT slowed to 10s (12V-drain fix)
```
`0x1684` AC-op (line ~110): `{ 1,  1,  1,  1}` → `{ 1, 10,  1,  1}`
```cpp
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AC_CHARGING_OP_STATUS,      { 1, 10,  1,  1}, 0, ISOTP_STD }, // 0x1684 AC op: WAIT slowed to 10s (was 1s); AC engage caught within 10s
```
`0x1666` HLC (line ~111): `{ 1,  1,  0,  1}` → `{ 1, 10,  0,  1}`
```cpp
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_HLC_STATE,                  { 1, 10,  0,  1}, 0, ISOTP_STD }, // 0x1666 HLC: WAIT slowed to 10s (was 1s); DC re-engage caught within 10s
```

- [ ] **Step 2: Verify by inspection**

Confirm: only the WAIT (2nd) column changed on exactly these 3 rows; HS/AC/DC columns untouched; no other rows changed. Run:
```bash
cd ~/wt-etnga-charge-wait && git diff -- vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.cpp
```
Expected: exactly 3 changed lines, each `1` → `10` in the 2nd tuple slot.

- [ ] **Step 3: Commit**

```bash
cd ~/wt-etnga-charge-wait
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.cpp
git commit -m "etnga: slow CHARGE_WAIT engage-watch polls 1s->10s (12V-drain fix, Tier 1)"
```

---

### Task 2: Tier 2 — sleep after a sustained CHARGE_WAIT, with re-sleep flag

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h` (add members after line 82).
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp` (constants near line 28; `HandleChargeWaitState` 202-228; `TransitionToChargeAcState` 409-417; `TransitionToChargeDcState` 419-427; `TransitionToAwakeState` session-close 317-327; `TransitionToChargeHandshakeState` `!in_session` block ~347).

**Interfaces:**
- Consumes: `TransitionToSleepState()` (existing; stops polling, arms cooldown, preserves `m_charge_session`).
- Produces:
  - member `bool m_charge_wait_slept` — true once Tier 2 has slept this wait; gates the short re-sleep threshold.
  - constants `CHARGE_WAIT_SLEEP_SECS` (600), `CHARGE_WAIT_RESLEEP_SECS` (15).

- [ ] **Step 1: Add the member to the header**

In `vehicle_toyota_etnga.h`, after line 82 (`m_charge_state_entry`), add:
```cpp
    bool m_charge_wait_slept = false;  // true after Tier-2 CHARGE_WAIT sleep; selects the short re-sleep threshold
```

- [ ] **Step 2: Add the Tier-2 constants**

In `etnga_poll_states.cpp`, just below the existing `SLEEP_COOLDOWN_STEPS` line (~line 29), add:
```cpp
// CHARGE_WAIT 12V-drain protection: stop polling after a sustained idle wait so the bus
// idles and the 12V recovers (session is preserved; resumes on passive wake). First wait
// gets a responsive window; a wait re-entered after a prior sleep re-sleeps quickly to keep
// the oscillation duty-cycle low. See HandleChargeWaitState().
static const int CHARGE_WAIT_SLEEP_SECS   = 600;  // first wait, no charge -> sleep
static const int CHARGE_WAIT_RESLEEP_SECS = 15;   // re-entered after a wait-sleep -> sleep fast
```

- [ ] **Step 3: Add the Tier-2 sleep check to `HandleChargeWaitState`**

In `etnga_poll_states.cpp`, in `HandleChargeWaitState()`, the current tail (lines ~223-227) is:
```cpp
    if (!StandardMetrics.ms_v_env_awake->AsBool()) {
        // Bus went dead during scheduled wait (OBC slept or gateway isolated OBD)
        TransitionToSleepState();
        return;
    }
}
```
Replace it with (keep the env-dead check, add the idle-timeout sleep after it):
```cpp
    if (!StandardMetrics.ms_v_env_awake->AsBool()) {
        // Bus went dead during scheduled wait (OBC slept or gateway isolated OBD)
        TransitionToSleepState();
        return;
    }
    // 12V-drain protection: if we have sat in CHARGE_WAIT without charge engaging for the
    // threshold, stop polling and sleep so the bus idles and 12V recovers. The session is
    // preserved across the sleep and resumes on passive wake (see HandleAwakeState). A wait
    // re-entered after a prior sleep uses the short threshold to keep oscillation cheap.
    int monotonic = StandardMetrics.ms_m_monotonic->AsInt();
    int sleep_after = m_charge_wait_slept ? CHARGE_WAIT_RESLEEP_SECS : CHARGE_WAIT_SLEEP_SECS;
    if (monotonic - m_charge_state_entry >= sleep_after) {
        ESP_LOGI(TAG, "CHARGE_WAIT idle %ds — sleeping to protect 12V", sleep_after);
        m_charge_wait_slept = true;
        TransitionToSleepState();
        return;
    }
}
```

- [ ] **Step 4: Clear the flag when the wait genuinely ends**

The flag must clear when charge actually engages or a new session begins/closes (NOT on the SLEEP→AWAKE→WAIT resume). Make these four edits in `etnga_poll_states.cpp`:

In `TransitionToChargeAcState()` (after `m_charge_state_entry = ...;`, ~line 411):
```cpp
    m_charge_wait_slept = false;   // real charge engaged — end the wait
```
In `TransitionToChargeDcState()` (after `m_charge_state_entry = ...;`, ~line 421):
```cpp
    m_charge_wait_slept = false;   // real charge engaged — end the wait
```
In `TransitionToChargeHandshakeState()`, inside the `if (!m_charge_session.in_session) {` block (a genuinely new session, ~line 348, right after `m_charge_session.in_session = true;`):
```cpp
        m_charge_wait_slept = false;   // brand-new session — fresh responsive wait window
```
In `TransitionToAwakeState()`, inside the `if (oldState >= PollState::CHARGE_HANDSHAKE)` block where the session is reset (right before/after `m_charge_session = ChargeSessionState{};`, ~line 326):
```cpp
        m_charge_wait_slept = false;   // leaving the charge sub-machine — clear wait flag
```

- [ ] **Step 5: Verify by inspection**

Run:
```bash
cd ~/wt-etnga-charge-wait && git diff
```
Confirm: header has `m_charge_wait_slept`; two constants added; `HandleChargeWaitState` sleeps on the idle timeout using the flag-selected threshold; the flag is set in the Tier-2 sleep and cleared in exactly the four transitions above (AC, DC, new-session handshake, awake/session-close). Confirm no other state handler was changed in this task.

- [ ] **Step 6: Commit**

```bash
cd ~/wt-etnga-charge-wait
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp
git commit -m "etnga: sleep after sustained CHARGE_WAIT to protect 12V (Tier 2)"
```

---

### Task 3: Resume into CHARGE_WAIT (not fresh HANDSHAKE) + don't reset backoff on resume

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp` (`HandleAwakeState` PISW arm branch 128-139; `TransitionToChargeHandshakeState` `ResetSleepBackoff()` at line 342).

**Interfaces:**
- Consumes: `m_charge_session.in_session` (existing), `TransitionToChargeWaitState()` (existing), `TransitionToChargeHandshakeState()` (existing).
- Produces: behavior — when waking with cable seated and a session already open, re-enter `CHARGE_WAIT` directly; the escalating sleep cooldown is only reset on a genuinely new plug-in.

- [ ] **Step 1: Branch the AWAKE PISW-seated path on `in_session`**

In `HandleAwakeState()`, the current block (lines ~135-139) is:
```cpp
    if (pisw >= 0x02) {
        // Cable is seated — enter charge handshake immediately
        TransitionToChargeHandshakeState();
        return;
    }
```
Replace with:
```cpp
    if (pisw >= 0x02) {
        // Cable is seated. If a session is already open we are resuming after a CHARGE_WAIT
        // sleep — go straight back to CHARGE_WAIT (the engage-watch + direct AC/DC checks there
        // catch the charge when it starts) and let Tier 2 re-sleep. Only a genuinely new
        // plug-in (no open session) runs the full handshake negotiation.
        if (m_charge_session.in_session) {
            TransitionToChargeWaitState();
        } else {
            TransitionToChargeHandshakeState();
        }
        return;
    }
```

- [ ] **Step 2: Only reset the sleep backoff on a new session**

In `TransitionToChargeHandshakeState()`, remove the unconditional `ResetSleepBackoff();` (line ~342) and move it inside the `if (!m_charge_session.in_session) {` block so it runs only for a new session. The current top of the function:
```cpp
    m_armed_for_charge = false;  // arm state consumed on entering handshake
    ResetSleepBackoff();   // charge session beginning — resume responsive cooldowns
    m_charge_state_entry = StandardMetrics.ms_m_monotonic->AsInt();
```
becomes:
```cpp
    m_armed_for_charge = false;  // arm state consumed on entering handshake
    m_charge_state_entry = StandardMetrics.ms_m_monotonic->AsInt();
```
and inside `if (!m_charge_session.in_session) {`, add as the first line of the block:
```cpp
        ResetSleepBackoff();   // brand-new charge session — resume responsive cooldowns
```

- [ ] **Step 3: Verify by inspection**

Run:
```bash
cd ~/wt-etnga-charge-wait && git diff -- vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp
```
Confirm: AWAKE seated-cable path now branches on `in_session`; `ResetSleepBackoff()` appears exactly once, inside the `!in_session` block; no other behavior changed. (Note: with Task 2 + Task 3, resuming a slept session enters `CHARGE_WAIT`, which on a still-idle car re-sleeps after `CHARGE_WAIT_RESLEEP_SECS` without resetting the cooldown — the intended low-duty oscillation.)

- [ ] **Step 4: Commit**

```bash
cd ~/wt-etnga-charge-wait
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp
git commit -m "etnga: resume slept charge session into CHARGE_WAIT; keep sleep backoff on resume"
```

---

### Task 4: PISW debounce — don't false-close a session on the OBC post-wake transient

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h` (add member after the Task-2 member).
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp` (`HandleAwakeState` wake-reconcile 101-114).

**Interfaces:**
- Consumes: `m_v_charge_pisw_raw` (existing metric), `m_charge_session.in_session` (existing).
- Produces: member `int m_pisw_zero_count` — count of consecutive fresh AWAKE PISW reads == 0x00; the wake-reconcile finalizes only at ≥2.

- [ ] **Step 1: Add the debounce member to the header**

In `vehicle_toyota_etnga.h`, immediately after the `m_charge_wait_slept` line added in Task 2:
```cpp
    int  m_pisw_zero_count = 0;        // consecutive fresh AWAKE PISW==0x00 reads; debounces the OBC post-wake transient
```

- [ ] **Step 2: Require two consecutive PISW==0 reads before closing**

In `HandleAwakeState()`, the current wake-reconcile guard (lines ~101-103) is:
```cpp
    if (m_charge_session.in_session &&
        !m_v_charge_pisw_raw->IsStale() &&
        m_v_charge_pisw_raw->AsInt() == 0x00) {
```
Replace the whole reconcile guard + body opening so a fresh non-stale read updates the counter and only acts at 2. Change the three guard lines above to:
```cpp
    // Debounce the cable-removed read: the OBC can briefly report PISW=0x00 for ~30s after
    // wake before it is fully up. Require two consecutive fresh 0x00 reads before finalizing,
    // so a wake-from-CHARGE_WAIT-sleep transient does not falsely close a still-plugged session.
    if (!m_v_charge_pisw_raw->IsStale()) {
        if (m_v_charge_pisw_raw->AsInt() == 0x00)
            m_pisw_zero_count++;
        else
            m_pisw_zero_count = 0;
    }
    if (m_charge_session.in_session && m_pisw_zero_count >= 2) {
```
(The existing body — set `ms_v_charge_state` "done", `LogChargeEvent`, `GenerateChargeReport`, reset `m_charge_session` — stays unchanged below this line.)

- [ ] **Step 3: Reset the counter when a session opens**

So a stale count from a previous wait can't carry into a new session, reset it where a new session opens. In `TransitionToChargeHandshakeState()`, inside the `if (!m_charge_session.in_session) {` block (alongside the `m_charge_wait_slept = false;` added in Task 2):
```cpp
        m_pisw_zero_count = 0;   // fresh session — clear cable-removed debounce
```

- [ ] **Step 4: Verify by inspection**

Run:
```bash
cd ~/wt-etnga-charge-wait && git diff
```
Confirm: header has `m_pisw_zero_count`; the wake-reconcile increments on fresh `0x00`, resets on any non-zero fresh read, and only finalizes at `>= 2`; the counter is reset in the new-session block. Confirm the reconcile body (done/report/reset) is otherwise unchanged.

- [ ] **Step 5: Commit**

```bash
cd ~/wt-etnga-charge-wait
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_poll_states.cpp
git commit -m "etnga: debounce PISW=0 cable-removed read to survive OBC post-wake transient"
```

---

### Task 5: changes.txt entry + CI build verification

**Files:**
- Modify: `vehicle/OVMS.V3/changes.txt` (top entry).

**Interfaces:**
- Consumes: nothing.
- Produces: the user-facing changelog entry; triggers the CI build of the whole branch.

- [ ] **Step 1: Add a changes.txt entry**

Unreleased changes accumulate as `-` bullets under the existing
`????-??-?? ???  ???????  OTA release` header at the top of `vehicle/OVMS.V3/changes.txt`
(do NOT add a new dated header). Match the existing bullets exactly: a `- ` first line, with
continuation lines indented 4 spaces. Add this as a new bullet directly under that header,
above the existing e-TNGA bullets:
```
- Toyota e-TNGA (Subaru Solterra / Toyota bZ4X): the module no longer drains the 12V battery
    while plugged in waiting for a scheduled charge. The charge-wait state now polls sparsely
    and, after a sustained idle wait, sleeps until charging actually starts (waking
    automatically) — so a scheduled/overnight charge is captured in one session instead of
    being missed when the module previously browned out. No config changes.
```
(No `New configs:` block — there are none.)

- [ ] **Step 2: Commit**

```bash
cd ~/wt-etnga-charge-wait
git add vehicle/OVMS.V3/changes.txt
git commit -m "changes.txt: e-TNGA CHARGE_WAIT 12V-drain fix"
```

- [ ] **Step 3: Push the branch and verify the CI build is green**

This is the compile gate for all of Tasks 1-5 (no host build here per project setup; GitHub Actions builds it).
```bash
cd ~/wt-etnga-charge-wait
git push -u origin feature/etnga-charge-wait-12v-sleep
```
Then watch the build:
```bash
gh run watch "$(gh run list --branch feature/etnga-charge-wait-12v-sleep --limit 1 --json databaseId -q '.[0].databaseId')"
```
Expected: the e-TNGA-enabled build job completes **success**. If it fails to compile, read the log, fix the offending task's edit, recommit, re-push.

---

### Task 6: On-vehicle validation (final gate — cannot be done on host)

**Files:** none (runtime validation on the physical module/vehicle).

**Interfaces:**
- Consumes: a built image of this branch flashed to the module (see `howto_ovms_fast_deploy` / GitHub-release deploy path).

- [ ] **Step 1: Flash the branch build to the module**

Build artifact from the green CI run (Task 5). Deploy per the established fast path (`ota flash http` from os-k3s after `sd mount`). Confirm the module reboots into the new build.

- [ ] **Step 2: Scheduled-charge happy path**

Plug in at a charger set to a delayed/scheduled start (or simulate a long wait). Pull logs (`ovms-log-pull`) and confirm, in order:
- `Transitioning ... to the CHARGE_WAIT state`, then engage-watch polling at ~10s (Tier 1).
- After ~10 min: `CHARGE_WAIT idle 600s — sleeping to protect 12V`, then `... to the SLEEP state`.
- No `powermgmt: 12V battery critical` during the wait; 12V holds.
- When the schedule fires: a wake (`... to the AWAKE state`), cable seated → `... to the CHARGE_WAIT state` → then `AC charging started`.
- One continuous session: `ms_v_charge_kwh` is NOT reset across the sleep; one charge report covering the whole session.

- [ ] **Step 3: Unplug-while-asleep**

Plug in, let it reach the Tier-2 sleep, then unplug during sleep. Confirm on next wake: the session finalizes cleanly (`Charge session closed`/report written, `in_session` cleared) and that the PISW debounce did not false-close earlier (no spurious "closed" right at wake before PISW settled).

- [ ] **Step 4: AC pause/resume**

During an AC charge, induce a brief pause (charger stops; `ac_op == 0x00` → CHARGE_WAIT). Confirm Tier 1 catches the resume within ~10s (`AC charging started` again) and the session/kWh survive (no reset, no new report).

- [ ] **Step 5: Update memory**

Record the validation outcome in `project_etnga_charge_wait_12v_drain` (validated / any deltas found).

---

## Notes for the implementer

- The four state transitions touched in Tasks 2-4 all live in `etnga_poll_states.cpp`; read the whole file once before starting so the flag/counter lifecycle is clear.
- `m_charge_state_entry` is stamped by every `TransitionToCharge*State()`, so the Tier-2 idle timer in Task 2 is measured from the most recent (re-)entry into CHARGE_WAIT — exactly what's wanted.
- Do not touch AC/DC/HANDSHAKE cadences or handlers beyond what each task specifies; the charge accounting and report logic are out of scope.
