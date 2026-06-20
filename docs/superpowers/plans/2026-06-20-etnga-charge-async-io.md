# e-TNGA Async Charge File-I/O Worker — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move all e-TNGA charge file I/O (CSV flush, HTML report write, session-end unlink, prune) off the Events task onto a small dedicated background worker, so a slow/stalled SD card can never block charge polling.

**Architecture:** A single low-priority FreeRTOS worker task drains a short queue of heap-allocated jobs (`{op, path, data, prune_dir}`). Producers (`FlushChargeCsv`, `GenerateChargeReport`) keep master's v2 RAM `csv_buf` batching unchanged and replace their inline `std::ofstream` writes with a non-blocking enqueue. Per-job `fopen`/`fwrite`/`fclose` (no persistent handle) keeps it simple.

**Tech Stack:** C++ (ESP-IDF 3.3 / older GCC), FreeRTOS (`xQueueCreate`/`xTaskCreatePinnedToCore`/`xSemaphore`), OVMS `ESP_LOGx`.

## Global Constraints

- **Spec:** `docs/superpowers/specs/2026-06-20-etnga-charge-async-io-design.md`.
- **No host tests.** Per-task gate is the **GitHub CI build** (compile-verify); behavioral verification is a real on-vehicle charge session (Task 4). Do not propose local builds.
- **Build:** the component's `component.mk` auto-compiles every `.cpp` in `src/` (`COMPONENT_SRCDIRS := src`); there is **no** `CMakeLists.txt`. Adding a new `src/*.cpp` needs **no** build-file edit.
- **Style:** match the surrounding file exactly — 4-space indent in these files, `ESP_LOGx(TAG, ...)`, `TAG` is `"v-toyota-etnga"`. Do not reformat existing code.
- **TAG value:** `static const char* TAG = "v-toyota-etnga";` (confirm against the other `etnga_*.cpp` files).
- **CI trigger:** feature branches build on PR; if no run appears, `gh workflow run build.yml --repo kezarjg/Open-Vehicle-Monitoring-System-3 --ref feature/etnga-charge-async-io`. Bins land in MinIO (`mc` alias `sh`, `ci-artifacts/<run_id>/ovms3/`), not GitHub artifacts.
- **Branch:** `feature/etnga-charge-async-io` (worktree `~/wt-etnga-async-io`), off master `d03d4b68`.

---

### Task 1: Worker scaffolding (queue + task + enqueue/stop + relocated prune)

Adds the worker infrastructure and wires its teardown into the destructor. Not yet called by producers (Tasks 2–3), so this task changes behavior nowhere — it just must compile.

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h` (add job struct, members, method decls near the charge-report decls ~line 191)
- Create: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_charge_io.cpp`
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.cpp` (destructor ~line 175)
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_charge_report.cpp` (remove the static `PruneChargeReports` def at lines 329–372 and `CHARGE_REPORT_MAX` at line 38 — both relocate into the worker file; its sole caller at line 627 is removed in Task 3)

**Interfaces:**
- Produces (used by Tasks 2–3):
  - `struct etnga_io_job { enum Op { WRITE_APPEND, WRITE_TRUNCATE, UNLINK, STOP } op; std::string path, data, prune_dir; };`
  - `bool OvmsVehicleToyotaETNGA::ChargeIoEnqueue(etnga_io_job* job);` — heap job by pointer; worker `delete`s it; returns false (and deletes) if the queue is full.

- [ ] **Step 1: Header — add the job struct, members, and method decls.** In `vehicle_toyota_etnga.h`, immediately after the charge-report method declarations (after `void FlushChargeCsv();`, ~line 195), add:

```cpp
    // --- Async charge file-I/O worker (decouples SD writes from the Events task) ---
    // See docs/superpowers/specs/2026-06-20-etnga-charge-async-io-design.md
    struct etnga_io_job {
      enum Op { WRITE_APPEND, WRITE_TRUNCATE, UNLINK, STOP } op;
      std::string path;        // destination (producer resolves SD vs /store)
      std::string data;        // bytes to write (empty for UNLINK/STOP)
      std::string prune_dir;   // non-empty on the report write → prune that dir afterward
    };
    QueueHandle_t     m_io_queue   = NULL;
    TaskHandle_t      m_io_task     = NULL;
    SemaphoreHandle_t m_io_stopped  = NULL;
    uint32_t          m_io_dropcnt  = 0;
    static void ChargeIoTaskEntry(void* arg);
    void ChargeIoTask();
    void StartChargeIoTask();              // lazy, idempotent
    bool ChargeIoEnqueue(etnga_io_job* job);
    void StopChargeIoTask();               // sentinel + timed join (never blocks shutdown)
```

(The FreeRTOS handle types and `std::string` are already visible in this header via the existing OVMS/vehicle includes — confirm it still compiles in Step 6.)

- [ ] **Step 2: Create `etnga_charge_io.cpp`** with the worker, enqueue, lifecycle, and the relocated prune:

```cpp
/*
;    Project:       Open Vehicle Monitor System
;    e-TNGA async charge file-I/O worker.
;
; Decouples charge-report file writes (CSV flush, HTML report, session-end unlink, prune)
; from the Events task: producers enqueue heap-owned jobs; one low-priority worker task
; performs the actual SD I/O, so a stalled SD can never block charge polling.
; See docs/superpowers/specs/2026-06-20-etnga-charge-async-io-design.md
*/

#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <algorithm>
#include "ovms_log.h"
#include "vehicle_toyota_etnga.h"

static const char* TAG = "v-toyota-etnga";

static const int CHARGE_REPORT_MAX = 50;   // retain at most this many reports (moved from etnga_charge_report.cpp)

// Retain only the newest CHARGE_REPORT_MAX sessions; delete both .html and .csv for each
// pruned stem. Also removes orphan .csv files whose session never produced an .html.
// (Relocated verbatim from etnga_charge_report.cpp so it runs on the worker, not Events.)
static void PruneChargeReports(const std::string& dir)
{
    DIR* d = opendir(dir.c_str());
    if (!d) return;
    std::vector<std::string> stems;        // .html stems (a complete report)
    std::vector<std::string> csv_stems;    // .csv stems (may be orphaned)
    struct dirent* e;
    while ((e = readdir(d)) != NULL) {
        std::string n = e->d_name;
        if (n.size() > 5 && n.compare(n.size()-5, 5, ".html") == 0)
            stems.push_back(n.substr(0, n.size()-5));
        else if (n.size() > 4 && n.compare(n.size()-4, 4, ".csv") == 0)
            csv_stems.push_back(n.substr(0, n.size()-4));
    }
    closedir(d);

    for (size_t i = 0; i < csv_stems.size(); i++) {
        bool has_html = false;
        for (size_t j = 0; j < stems.size(); j++)
            if (stems[j] == csv_stems[i]) { has_html = true; break; }
        if (!has_html)
            unlink((dir + "/" + csv_stems[i] + ".csv").c_str());
    }

    if ((int)stems.size() <= CHARGE_REPORT_MAX) return;
    std::sort(stems.begin(), stems.end());
    int del = (int)stems.size() - CHARGE_REPORT_MAX;
    for (int i = 0; i < del; i++) {
        unlink((dir + "/" + stems[i] + ".html").c_str());
        unlink((dir + "/" + stems[i] + ".csv").c_str());
        ESP_LOGD(TAG, "Charge report pruned: %s", stems[i].c_str());
    }
}

void OvmsVehicleToyotaETNGA::ChargeIoTaskEntry(void* arg)
{
    static_cast<OvmsVehicleToyotaETNGA*>(arg)->ChargeIoTask();
}

void OvmsVehicleToyotaETNGA::ChargeIoTask()
{
    etnga_io_job* job = NULL;
    for (;;) {
        if (xQueueReceive(m_io_queue, &job, portMAX_DELAY) != pdTRUE) continue;
        if (job->op == etnga_io_job::STOP) { delete job; break; }
        if (job->op == etnga_io_job::UNLINK) {
            remove(job->path.c_str());
        } else {   // WRITE_APPEND / WRITE_TRUNCATE
            FILE* f = fopen(job->path.c_str(), job->op == etnga_io_job::WRITE_APPEND ? "a" : "w");
            if (f) {
                size_t n = fwrite(job->data.data(), 1, job->data.size(), f);
                fclose(f);
                if (n != job->data.size())
                    ESP_LOGW(TAG, "charge-io: short write %s (%u/%u)", job->path.c_str(),
                        (unsigned)n, (unsigned)job->data.size());
                else
                    ESP_LOGD(TAG, "charge-io: wrote %u B to %s", (unsigned)n, job->path.c_str());
                if (!job->prune_dir.empty()) PruneChargeReports(job->prune_dir);
            } else {
                ESP_LOGW(TAG, "charge-io: cannot open %s", job->path.c_str());
            }
        }
        delete job;
    }
    m_io_task = NULL;
    xSemaphoreGive(m_io_stopped);
    vTaskDelete(NULL);
}

void OvmsVehicleToyotaETNGA::StartChargeIoTask()
{
    if (m_io_task) return;
    if (!m_io_queue)   m_io_queue   = xQueueCreate(8, sizeof(etnga_io_job*));
    if (!m_io_stopped) m_io_stopped = xSemaphoreCreateBinary();
    if (m_io_queue)
        xTaskCreatePinnedToCore(ChargeIoTaskEntry, "etnga_io", 4096, this, 3, &m_io_task, CORE(1));
}

bool OvmsVehicleToyotaETNGA::ChargeIoEnqueue(etnga_io_job* job)
{
    StartChargeIoTask();
    if (!m_io_queue || xQueueSend(m_io_queue, &job, 0) != pdTRUE) {
        m_io_dropcnt++;
        ESP_LOGW(TAG, "charge-io: queue full, dropped write to %s (dropcnt=%u)",
            job->path.c_str(), m_io_dropcnt);
        delete job;
        return false;
    }
    return true;
}

void OvmsVehicleToyotaETNGA::StopChargeIoTask()
{
    if (!m_io_task) return;
    etnga_io_job* job = new etnga_io_job;
    job->op = etnga_io_job::STOP;
    if (xQueueSend(m_io_queue, &job, pdMS_TO_TICKS(100)) != pdTRUE) { delete job; return; }
    // Wait for the worker to drain and exit, but never block module shutdown forever.
    if (m_io_stopped) xSemaphoreTake(m_io_stopped, pdMS_TO_TICKS(2000));
}
```

Note on `CORE(1)`: this is the OVMS core-pinning macro used by `OvmsCommandApp::FileLog` in `main/ovms_command.cpp`. If it is not visible from this TU, replace `CORE(1)` with the literal `1` (charge I/O on core 1 like the log task). Verify in Step 6.

- [ ] **Step 3: Leave `etnga_charge_report.cpp` untouched in this task.** The worker file (Step 2) defines its own `static PruneChargeReports` and `static const int CHARGE_REPORT_MAX` with **internal linkage**; `etnga_charge_report.cpp` keeps its own `static` copies (and the line-627 call) until Task 3. Two file-local statics in separate translation units do **not** clash, and each copy is used in its own TU, so Task 1 compiles cleanly on its own. The old copies are removed in Task 3 Step 1 (alongside removing their call site).

- [ ] **Step 4: Wire teardown into the destructor.** In `vehicle_toyota_etnga.cpp`, inside `~OvmsVehicleToyotaETNGA()` (~line 175), add as the first statement of the body:

```cpp
    StopChargeIoTask();   // drain/stop the async charge-I/O worker before teardown
```

- [ ] **Step 5: Commit.**

```bash
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_charge_io.cpp \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.cpp
git commit -m "etnga: add async charge file-I/O worker (scaffolding, not yet wired)"
```

- [ ] **Step 6: Build-verify on CI.** Push and check the build (this is the per-task gate):

```bash
git push -u origin feature/etnga-charge-async-io
gh workflow run build.yml --repo kezarjg/Open-Vehicle-Monitoring-System-3 --ref feature/etnga-charge-async-io
# wait, then:
gh run list --repo kezarjg/Open-Vehicle-Monitoring-System-3 --branch feature/etnga-charge-async-io --limit 1
gh run watch <run-id> --repo kezarjg/Open-Vehicle-Monitoring-System-3 --exit-status
```
Expected: **success**. If `CORE(1)` or a FreeRTOS type fails to resolve, fix per the notes in Steps 1–2 and re-push.

---

### Task 2: Route CSV flush through the worker

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_charge_report.cpp` (`FlushChargeCsv`, lines 293–326)

**Interfaces:**
- Consumes: `ChargeIoEnqueue(etnga_io_job*)`, `struct etnga_io_job` (Task 1).

- [ ] **Step 1: Replace the body of `FlushChargeCsv()`** (currently the `std::ofstream` block at lines 293–326) with the enqueue version. Keep the existing signature and the early-return:

```cpp
void OvmsVehicleToyotaETNGA::FlushChargeCsv()
{
    if (m_charge_session.csv_buf.empty() || m_charge_session.base.empty())
        return;
    // Hand the buffered rows to the async writer instead of writing on the Events task.
    // FIFO order guarantees the first (truncate) write lands before later appends.
    etnga_io_job* job = new etnga_io_job;
    job->op   = m_charge_session.csv_file_created ? etnga_io_job::WRITE_APPEND
                                                  : etnga_io_job::WRITE_TRUNCATE;
    job->path = m_charge_session.base + ".csv";
    job->data = m_charge_session.csv_buf;
    if (ChargeIoEnqueue(job)) {
        m_charge_session.csv_buf.clear();
        m_charge_session.csv_file_created = true;
        m_charge_session.last_csv_flush = StandardMetrics.ms_m_monotonic->AsInt();
    } else if (m_charge_session.csv_buf.size() > 16384) {
        // Worker persistently backlogged: bound the pending buffer (heap safety),
        // matching the prior synchronous behavior.
        ESP_LOGE(TAG, "Charge CSV buffer over limit — discarding buffered rows");
        m_charge_session.csv_buf.clear();
    }
}
```

- [ ] **Step 2: Commit.**

```bash
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_charge_report.cpp
git commit -m "etnga: route charge CSV flush through the async I/O worker"
```

- [ ] **Step 3: Build-verify on CI** (push triggers the PR build; watch for **success**).

---

### Task 3: Route report write + session-end unlink + prune through the worker

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_charge_report.cpp` (remove static `PruneChargeReports` + `CHARGE_REPORT_MAX`; `GenerateChargeReport` no-energy unlink ~479, report stream ~501, tail ~618–627; includes)

**Interfaces:**
- Consumes: `ChargeIoEnqueue(etnga_io_job*)`, `struct etnga_io_job` (Task 1); `PruneChargeReports` now lives in `etnga_charge_io.cpp` (Task 1).

- [ ] **Step 1: Remove the relocated prune + constant.** Delete `static const int CHARGE_REPORT_MAX = 50;` (line 38) and the entire `static void PruneChargeReports(const char* tag, const std::string& dir) { ... }` definition (lines ~329–372). Both now live in `etnga_charge_io.cpp`.

- [ ] **Step 2: Fix includes.** At the top of `etnga_charge_report.cpp`, replace `#include <fstream>` (line 24) with `#include <sstream>` (the file no longer opens any `std::ofstream` after this task; it renders into a `std::ostringstream`). Keep all other includes.

- [ ] **Step 3: No-energy early-out → async unlink.** In `GenerateChargeReport`, replace lines ~479–480:

```cpp
        if (m_charge_session.csv_file_created)   // remove the streamed stub CSV (buffered rows are discarded with the session)
            unlink((m_charge_session.base + ".csv").c_str());
```
with:
```cpp
        if (m_charge_session.csv_file_created) {  // remove the streamed stub CSV via the worker
            etnga_io_job* j = new etnga_io_job;
            j->op = etnga_io_job::UNLINK;
            j->path = m_charge_session.base + ".csv";
            ChargeIoEnqueue(j);
        }
```

- [ ] **Step 4: Render to a string instead of a file.** Replace the report-file open (lines ~501–502):

```cpp
    std::ofstream f(m_charge_session.base + ".html", std::ios::out | std::ios::trunc);
    if (!f) { ESP_LOGE(TAG, "Charge report: cannot write %s.html", m_charge_session.base.c_str()); return; }
```
with:
```cpp
    std::ostringstream f;   // render into RAM; the async worker writes the file off the Events task
```
(All the subsequent `f << ...` and `RenderPowerSvg(f)` calls are unchanged — `RenderPowerSvg` already takes `std::ostream&`.)

- [ ] **Step 5: Enqueue the rendered report + prune; drop the synchronous close/log/prune.** Replace the report tail — the `f.close();` / fail-check / `ESP_LOGI(... "Charge report written" ...)` / `PruneChargeReports(TAG, ChargeReportDir());` block (lines ~618–627) — with:

```cpp
    // Hand the rendered HTML to the async writer; it writes the file and prunes off-thread.
    etnga_io_job* job = new etnga_io_job;
    job->op        = etnga_io_job::WRITE_TRUNCATE;
    job->path      = m_charge_session.base + ".html";
    job->data      = f.str();
    job->prune_dir = ChargeReportDir();
    ChargeIoEnqueue(job);
    ESP_LOGI(TAG, "Charge report queued: %s.html (%.2f kWh, %d%%->%d%%)",
        m_charge_session.base.c_str(), energy_kwh, start_soc, end_soc);
```
(Confirm the exact text of lines 618–627 first with `sed -n '614,628p'`; the variable names `energy_kwh`, `start_soc`, `end_soc` are already in scope from earlier in the function.)

- [ ] **Step 6: Commit.**

```bash
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_charge_report.cpp
git commit -m "etnga: route charge report write + unlink + prune through the async I/O worker"
```

- [ ] **Step 7: Build-verify on CI** (push, watch for **success**). After this task there must be **no `std::ofstream` / `#include <fstream>`** left in the e-TNGA component: `grep -rn 'ofstream\|<fstream>' vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/` should return nothing.

---

### Task 4: On-vehicle charge-session validation (manual gate before merge)

No host tests exist; this is the behavioral gate. Flash the branch build to the module (see `CLAUDE.local.md` for the launcher flash procedure) and run a charge.

- [ ] **Step 1:** Confirm the branch build is on the module (`ota status` shows this build) and SUBSOL is loaded, 0 crashes.
- [ ] **Step 2:** Do an AC (and, if possible, DC) charge session. During the charge, watch the log (`log level verbose v-toyota-etnga` or pull `/sd/logs/log.txt`) for `charge-io: wrote N B to .../<stem>.csv` lines roughly every 30 s — confirming flushes run on the worker.
- [ ] **Step 3:** After unplug, verify: the per-sample `<stem>.csv` exists and its rows are **continuous** across flush boundaries (no gaps/dupes); the `<stem>.html` report was written and opens; prune kept ≤ 50 reports.
- [ ] **Step 4:** Confirm **no Events-task stall / TWDT** during the charge (`boot status` crash counters unchanged; no `Task watchdog` in the log) and a **clean module shutdown** on the next vehicle switch / reset (no hang on the destructor join).
- [ ] **Step 5:** Record results in the PR; only merge after a clean charge session.

---

## Notes for the implementer
- Charge telemetry is **best-effort**: dropping a job (queue full) or a failed write must never disturb the charge. Never add blocking waits to the producers.
- Keep master's v2 CSV format and 30 s/4 KB flush cadence exactly — this change only moves *where the bytes get written*, not *what or when*.
- Out of scope (do not add here): the web `/xte/report-move` move-to-SD endpoints (Mongoose task — a second producer that would require an enqueue mutex).
