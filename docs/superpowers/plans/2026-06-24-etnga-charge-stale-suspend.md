# e-TNGA: suspend charge logging during CAN-stale (#138) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** When the CAN poll path goes quiet during a charge (e.g. car locked → OBD gateway isolates the OBC), suspend the per-sample CSV row, the `station_kwh`/`delivered_ah` accumulation, and the SVG chart sample so they don't record/integrate frozen values — and zero the live power metrics for display honesty — resuming automatically when polls return.

**Architecture:** A `m_last_poll_monotonic` timestamp stamped on every `IncomingPollReply` (poller task) is the "bus-alive" signal. `UpdateChargeSessionStats` (vehicle-ticker task) early-returns when `now − m_last_poll_monotonic > 3 s`, after zeroing the live power/current metrics and bumping the integrator dt baselines so resume doesn't bridge the gap. Fully fixes the #138 per-tick corruption; the headline `v.c.kwh` is already gap-clamped (per-reply + `EnergyIntervalHours` 60 s clamp) and needs no change.

**Tech Stack:** C++ (ESP-IDF 3.3 / older GCC), the e-TNGA vehicle component. No new polls, no new dependencies.

## Global Constraints

- **C++ for ESP-IDF 3.3 / older GCC.** Match the surrounding file's style (`StandardMetrics.ms_*`, `ESP_LOGx(TAG, …)`).
- **No host unit-test suite / not locally buildable.** Per-task gate = code-inspection check (shown `grep`/reasoning) + the GitHub CI build (`gh workflow run build.yml --ref fix/etnga-charge-stale-suspend`). Do NOT propose a local `make`.
- **`CHARGE_STALE_SECS = 3`** — the charge poll path runs at 1 s; 3 missed = clearly stale but quick. Verbatim value.
- **Detection signal is a dedicated `m_last_poll_monotonic`** stamped on *every* `IncomingPollReply` (any ECU) — NOT a metric `Age()` (the power metrics are zeroed on stale, which would be circular). It is a plain cross-task `int` (poller writes, ticker reads), consistent with the existing `m_charge_fault_pending` pattern — benign unsynchronized scalar.
- **Power-zeroing is display-only** — it does not touch the energy integrals; `v.c.kwh` recomputes from pack V×I on the next reply.
- **Suspend = the whole per-tick body** of `UpdateChargeSessionStats` (CSV, `station_kwh`/`delivered_ah`, SVG, peak/temp/ambient) via an early `return`.
- **Single-purpose PR / `changes.txt`** entry. Work in `/home/devuser/wt-etnga-stale-suspend` on `fix/etnga-charge-stale-suspend`. Paths relative to `vehicle/OVMS.V3/components/vehicle_toyota_etnga/`.

---

## Task 1: Detection — `m_last_poll_monotonic` + stamp

**Files:**
- Modify: `src/vehicle_toyota_etnga.h` (add the member)
- Modify: `src/etnga_poll_processor.cpp` (`IncomingPollReply`, stamp at the top)

**Interfaces:**
- Produces: member `int m_last_poll_monotonic = 0;` — monotonic seconds of the last poll reply (any ECU), `0` = none yet. Read by `UpdateChargeSessionStats` (Task 2).

- [ ] **Step 1: Add the member**

In `src/vehicle_toyota_etnga.h`, near the other charge/timing members (e.g. by `m_charge_state_entry` / `m_charge_session`), add:

```cpp
    int m_last_poll_monotonic = 0;   // #138: monotonic s of the last poll reply (any ECU); charge-stale detection
```

- [ ] **Step 2: Stamp it at the top of `IncomingPollReply`**

In `src/etnga_poll_processor.cpp`, `IncomingPollReply(...)` opens with `{` then `// Check if this is the first frame of the multi-frame response`. Insert the stamp as the very first statement (so it fires on every frame, including partial multi-frame frames — any frame proves the bus is alive):

```cpp
void OvmsVehicleToyotaETNGA::IncomingPollReply(const OvmsPoller::poll_job_t &job, uint8_t* data, uint8_t length)
{
    m_last_poll_monotonic = StandardMetrics.ms_m_monotonic->AsInt();   // #138: bus-alive timestamp for charge-stale detection

    // Check if this is the first frame of the multi-frame response
    if (job.mlframe == 0) {
```

- [ ] **Step 3: Inspection check**

Run: `grep -n 'm_last_poll_monotonic' src/vehicle_toyota_etnga.h src/etnga_poll_processor.cpp`
Expected: the member declared with default `0`; the stamp is the first statement of `IncomingPollReply` (before the `if (job.mlframe == 0)` block). Confirm `StandardMetrics.ms_m_monotonic->AsInt()` is the monotonic-seconds source used elsewhere in these files (it is).

- [ ] **Step 4: Commit**

```bash
git add src/vehicle_toyota_etnga.h src/etnga_poll_processor.cpp
git commit -m "etnga: stamp last-poll-reply timestamp for charge-stale detection (#138)"
```

---

## Task 2: Suspend — early-return branch in `UpdateChargeSessionStats`

**Files:**
- Modify: `src/etnga_charge_report.cpp` (`UpdateChargeSessionStats`)

**Interfaces:**
- Consumes: `m_last_poll_monotonic` (Task 1); `m_charge_session.{last_sample_monotonic,last_svg_monotonic}` (existing).

- [ ] **Step 1: Add the can-stale branch**

In `UpdateChargeSessionStats()`, the function begins:
```cpp
    if (!m_charge_session.in_session)
        return;

    int now = StandardMetrics.ms_m_monotonic->AsInt();

    float p = StandardMetrics.ms_v_charge_power->AsFloat();
```
Insert the branch BETWEEN the `int now = …;` line and the `float p = …;` line:

```cpp
    // #138: if the CAN poll path has gone quiet (e.g. car locked → OBD gateway isolated from the
    // OBC), the charge metrics are frozen. Suspend per-sample logging + accumulation so we don't
    // record/integrate stale values; resume automatically when polls return.
    const int CHARGE_STALE_SECS = 3;   // charge poll path runs at 1s; 3 missed = clearly stale but quick
    if (m_last_poll_monotonic == 0 || now - m_last_poll_monotonic > CHARGE_STALE_SECS) {
        // Display honesty: zero the live power/current so the app/server/CSV don't show a frozen value.
        // (Does not touch the energy integrals; v.c.kwh recomputes from pack V×I on the next reply.)
        StandardMetrics.ms_v_charge_power->SetValue(0);
        StandardMetrics.ms_v_bat_power->SetValue(0);
        StandardMetrics.ms_v_bat_current->SetValue(0);
        // Keep the dt baselines current so resume doesn't bridge the gap into delivered_ah / station_kwh / SVG.
        m_charge_session.last_sample_monotonic = now;
        m_charge_session.last_svg_monotonic = now;
        return;   // skip peak/temp/ambient, station_kwh/delivered_ah, CSV row, SVG sample
    }
```

- [ ] **Step 2: Inspection check**

Run: `grep -n 'CHARGE_STALE_SECS\|m_last_poll_monotonic\|last_svg_monotonic = now' src/etnga_charge_report.cpp`
Expected: the branch sits after `int now = …` and before `float p = …`; zeroes the three metrics; bumps `last_sample_monotonic` AND `last_svg_monotonic`; `return`s. Confirm `last_svg_monotonic` is the real member name (grep it in the struct). Reason: during a ≥3 s lock the function returns each tick (no CSV/accumulation/SVG), the live power reads 0; on the first poll reply `m_last_poll_monotonic` advances → branch skipped → normal logging resumes with `dt ≈ 1 s` (no bridged gap).

- [ ] **Step 3: Commit**

```bash
git add src/etnga_charge_report.cpp
git commit -m "etnga: suspend charge logging/accounting during a CAN-stale window (#138)"
```

---

## Task 3: changes.txt + CI build + validation

**Files:**
- Modify: `vehicle/OVMS.V3/changes.txt`

- [ ] **Step 1: Add the changes.txt bullet** (under the existing `????-??-?? ???  ???????  OTA release` pending block, 4-space continuation indent, NO invented dated header):

```
- Toyota e-TNGA (Subaru Solterra / Toyota bZ4X): if the CAN bus goes quiet mid-charge (e.g. the
    car is locked, isolating the diagnostic gateway from the OBC), the charge report no longer
    records frozen/stale telemetry — the per-sample log, energy accounting and chart pause until
    the bus responds again, so the reported station energy and efficiency stay honest. Live charge
    power reads 0 during such a gap.
```

- [ ] **Step 2: Commit**

```bash
git add vehicle/OVMS.V3/changes.txt
git commit -m "etnga: changes.txt for CAN-stale charge-logging suspend (#138)"
```

- [ ] **Step 3: CI build**

Run: `gh workflow run build.yml --ref fix/etnga-charge-stale-suspend`, then watch (`gh run watch <id> --exit-status`). Expected green. Fix any compile error and amend the relevant task commit.

- [ ] **Step 4: On-vehicle validation checklist (record in PR / memory)**

The repro is a lock mid-AC-charge for more than a few seconds. Expect:
1. The CSV **stops adding rows** during the lock (no frozen rows; no `battery_kw>0 @ pack_a=0`).
2. The report's `station_kwh` / efficiency are sensible — **no phantom ~0.2 kWh station energy, no >100% efficiency**.
3. Live `v.c.power` reads **0** during the lock and recovers on unlock.
4. The session/phase is **not** torn down (still gated on `pisw`/`ac_op`/`hlc`).
5. On unlock, logging/accounting **resume** cleanly (no bridged spike in `station_kwh`/`delivered_ah`).

---

## Self-Review

**Spec coverage:**
- Detection (`m_last_poll_monotonic` stamp, `> 3 s`) → Task 1. ✓
- Suspend per-tick CSV + `station_kwh`/`delivered_ah` + SVG + peak/temp via early return → Task 2. ✓
- Power-zeroing (display-only) → Task 2. ✓
- dt-baseline bump on stale → Task 2. ✓
- Resume (automatic) → Task 2 (no code; the branch simply stops firing). ✓
- `v.c.kwh` unchanged (already gap-clamped) → not touched, by design. ✓
- `changes.txt` → Task 3. ✓

**Placeholder scan:** no TBD/TODO; every step has complete code; `CHARGE_STALE_SECS = 3` is concrete.

**Type consistency:** `m_last_poll_monotonic` (`int`) declared in Task 1, read in Task 2 — same name. Zeroed metrics (`ms_v_charge_power`/`ms_v_bat_power`/`ms_v_bat_current`) and bumped members (`last_sample_monotonic`/`last_svg_monotonic`) are existing names (Task 2 Step 2 inspection re-confirms `last_svg_monotonic`). Monotonic source `StandardMetrics.ms_m_monotonic->AsInt()` consistent across both tasks.
