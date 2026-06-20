# e-TNGA charge file I/O — async writer (design)

**Date:** 2026-06-20
**Component:** `vehicle/OVMS.V3/components/vehicle_toyota_etnga`
**Branch:** `feature/etnga-charge-async-io` (off master `d03d4b68`)

## Problem

The e-TNGA charge code writes to the SD card from the **Events task** — the per-session
CSV during a charge and the HTML report at session end. SD writes can be slow or stall;
when one blocks, it blocks the Events task, which runs the poller ticker, so **charge
polling stalls and the Task Watchdog can fire**. Master's v2 reduced the frequency
(`AppendChargeCsvRow()` buffers rows in a RAM `std::string csv_buf`; `FlushChargeCsv()`
writes it to SD every ~30 s / 4 KB), but each flush — and the end-of-session report write
and prune — is still a **synchronous, blocking SD write on the Events task**.

## Goal

No e-TNGA charge file I/O on the Events task, **regardless of SD health**. Charge polling
must never be blocked by a file write. This is an architectural-hygiene fix (decouple I/O
from the Events/poller task), not an SD-reliability feature.

Non-goals: changing the CSV format/cadence (keep v2's `csv_buf` batching exactly),
guaranteeing report delivery on a dead SD (best-effort), or host-testability (hardware-only).

## Approach

A small dedicated background writer task owned by the e-TNGA vehicle. Producers (on the
Events task) do RAM work + a non-blocking enqueue; a single worker task performs the
actual file I/O. Chosen over a reusable shared utility (YAGNI — one consumer) and over
transient per-write tasks (create-churn, can fail under memory pressure, needs a mutex
anyway).

This is deliberately a fraction of the size of the prior `etnga-charge-monitor-fixes`
branch's 529-line worker, because master's v2 batching and a per-job open/close model
remove most of that complexity (see "Why minimal" below).

## Components

### New file: `etnga_charge_io.cpp`
Implements the worker, the enqueue helper, lazy start, and stop/join. Declarations
(job struct, queue/task handles, drop counter, method prototypes) go in
`vehicle_toyota_etnga.h`. The new source must be added to **both** the component's
`component.mk` (legacy make build) and `CMakeLists.txt`.

### Job model
Jobs are heap-allocated and passed **by pointer** through a FreeRTOS queue, then `delete`d
by the worker. (A `std::string` cannot be memcpy'd by value through `xQueue`; pointer +
RAII `delete` is the safe idiom and avoids the old worker's manual `strdup`/`free`.)

```cpp
struct etnga_io_job {
  enum Op { WRITE_APPEND, WRITE_TRUNCATE, UNLINK, STOP } op;
  std::string path;        // destination (producer already resolved SD vs /store fallback)
  std::string data;        // bytes to write (empty for UNLINK/STOP)
  std::string prune_dir;   // non-empty only on the report WRITE_TRUNCATE → prune after write
};
```

Queue depth small (e.g. 8). Enqueue is non-blocking `xQueueSend(..., 0)`; on a full queue,
drop the job, `delete` it, increment a drop counter, and `ESP_LOGW`. Charge telemetry is
best-effort — **the producer never blocks**.

### Worker task
- One task, pinned to core 1, low priority, ~3–4 KB stack; **lazy-started** on first
  enqueue (no task exists unless a charge actually writes).
- Loop: `xQueueReceive` (block forever) → per job:
  - `WRITE_APPEND` / `WRITE_TRUNCATE`: `fopen(path, "a"|"w")`, `fwrite(data)`, `fclose`.
  - `UNLINK`: `remove(path)`.
  - then, if `prune_dir` is set, prune old reports in that dir (keep newest N).
  - `STOP`: `delete` the job and exit the task.
- **Per-job open/close** — no persistent `FILE*` held across jobs.

### Producers (Events task → now non-blocking)
- `FlushChargeCsv()`: enqueue `{WRITE_APPEND, csv_path, csv_buf}`, then clear `csv_buf`.
  Keeps v2's batching/format; only the write moves off-thread.
- `GenerateChargeReport()`: render HTML into a `std::ostringstream` (master's
  `RenderPowerSvg` already takes `std::ostream&`), enqueue
  `{WRITE_TRUNCATE, html_path, html, prune_dir}`. The no-energy early-out enqueues
  `{UNLINK, csv_path}`. The existing synchronous `PruneChargeReports` is removed from the
  Events path and performed by the worker after the report write.

### Lifecycle
- **Start:** lazy, idempotent (`StartChargeIoTask` from the enqueue helper).
- **Stop:** vehicle destructor enqueues `STOP` and joins the task with a ~2 s timeout; if it
  is wedged on a dead SD, proceed anyway — never block module shutdown.

## Why minimal (vs the old 529-line worker)
- **Per-job open/close** ⇒ no `FILE*` held across jobs ⇒ **no `sd.unmounting` sync-close
  handshake** (the old worker's largest piece).
- v2 RAM batching ⇒ coarse, infrequent jobs ⇒ **no per-row streaming, no fsync cadence**.
- Single producer (Events only) ⇒ **no enqueue mutex**.
- `std::string` job payloads ⇒ **no manual `strdup`/`free`**.

## Error handling
- `fopen`/`fwrite`/`remove` failure → `ESP_LOGW` + drop; the charge session is unaffected
  (the entire point). SD-vs-`/store` path selection stays in the producer, where master
  already chooses it via `isavailable()`.
- Queue full → drop + count (best-effort telemetry).
- A single FIFO worker preserves ordering: CSV appends land before the final report write.

## Scope
**In:** every charge write currently on the Events task — CSV flush, report write,
session-end `UNLINK`, post-report prune.
**Out:** the web "move-to-SD" endpoints (`/xte/report-move`, `/xte/report-loc`) — they run
on the Mongoose task, a different thread, and are a separate feature. (If added later, the
worker gains a second producer and would then need an enqueue mutex.)

## Files touched
- `etnga_charge_io.cpp` — **new** (worker, enqueue, start/stop).
- `vehicle_toyota_etnga.h` — job struct, queue/task/dropcnt members, method decls.
- `etnga_charge_report.cpp` — `FlushChargeCsv` and `GenerateChargeReport` enqueue instead
  of writing inline; remove the static `PruneChargeReports` (relocated into the worker).
- `vehicle_toyota_etnga.cpp` — destructor calls the stop/join.
- `component.mk` **and** `CMakeLists.txt` — add `etnga_charge_io.cpp`.

## Testing / validation
- **Build:** GitHub CI (push/PR) — no local/host build.
- **Behavioral (on a real charge session, required before merge):** CSV rows are continuous
  across flushes (no gaps/dupes at flush boundaries); the HTML report is written and prune
  keeps the newest N; no Events-task stall or TWDT during the charge; clean module shutdown
  (no hang on the destructor join). The charge subsystem cannot be validated off-vehicle.

## changes.txt
No entry: this is an internal robustness change with no user-facing behavior change and no
new config. (If behavior is later observed to differ, revisit.)
