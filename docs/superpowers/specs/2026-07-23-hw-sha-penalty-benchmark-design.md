# Quantifying the cost of disabling mbedTLS hardware SHA (upstream issue #1321)

**Date:** 2026-07-23
**Status:** Design approved, not yet implemented
**Upstream issue:** openvehicles/Open-Vehicle-Monitoring-System-3#1321
**Fork commit under test:** `3c1d8598` — "hw3.1: disable mbedTLS hardware SHA to fix TLS-handshake crash"

## Problem

Issue #1321 reports an `abort()` in `esp_sha_read_digest_state()` (ESP-IDF 3.3
`esp32/hwcrypto/sha.c:272`, the fault-injection guard) during mbedTLS TLS handshakes. The
shared ESP32 hardware SHA engine returns an all-zero digest state when
`mbedtls_sha256_clone()` snapshots the running handshake transcript hash mid-handshake.

This fork mitigates it by setting `CONFIG_MBEDTLS_HARDWARE_SHA=` (off) in
`support/sdkconfig.default.hw31`, moving mbedTLS SHA to software. AES and MPI hardware
acceleration are retained. The hw3.0 preset has always shipped with hardware SHA off.

dexterbg asked: **what is the performance/load penalty of that mitigation?**

This document designs the measurement. It does not implement it.

## Scope: what actually uses hardware SHA

Established by inspection of this tree, and load-bearing for the design:

- **mbedTLS is the only hardware-SHA consumer.** wolfSSL has `WOLFSSL_ESP32WROOM32_CRYPT`
  commented out (`components/wolfssl/port/user_settings.h:27`), so SSH crypto is already
  software and is unaffected by this change. The concurrency in the crash report comes from
  wolfSSH *load* widening the race window, not from wolfSSH touching the SHA engine.
- **`CONFIG_MBEDTLS_HARDWARE_SHA` selects only whether mbedTLS's port shims replace upstream
  `sha256.c`.** IDF's `esp_sha()` in `esp32/hwcrypto/sha.c` is part of the always-compiled
  esp32 component. (Assumption — see Threats.)
- **mbedTLS TLS sites on a stock module:** server v2 (`ovms_server_v2.cpp:915`), server v3
  MQTT/TLS (`ovms_server_v3.cpp:1003`), the HTTPS web server (`ovms_webserver.cpp:189`), and
  pushover (`pushover.cpp:403`).
- **OTA is *not* a TLS workload in mainline.** `OvmsHttpClient::Request` strips an `https://`
  prefix but then opens a plain TCP connection defaulting to port 80
  (`components/ovms_http/src/ovms_http.cpp:72-97`), and the default OTA server is
  `api.openvehicles.com/firmware/ota` with no scheme. Firmware download is therefore
  cleartext and contributes no SHA load.

Consequence: the real workload is **handshake-dominated**, with modest bulk only from
web-UI asset serving. The measurement must be sensitive enough to credibly report a small
number, not just detect a large one.

## Design

Three layers, because the question has two halves ("what is the number" and "does it
matter") and neither layer answers both.

### Builds and A/B protocol

Two builds, one line apart, off a branch from current fork master:

1. Commit 1 adds the `test crypto sha` command (Layer 1).
2. Commit 2 flips `CONFIG_MBEDTLS_HARDWARE_SHA=` to `=y` in
   `support/sdkconfig.default.hw31`.

Label them **SW** (hardware SHA off, current fork) and **HW** (hardware SHA on, stock
upstream config). Identical vehicle selection and everything else. Build via
`gh workflow run build.yml --ref <branch>`; collect `ovms3.bin` from MinIO
`ci-artifacts/<run_id>/ovms3/`. Record both `ota status` hashes in the writeup.

**Run order: SW → HW → SW.** The repeat SW run is the noise control. If the two SW runs
differ by more than the SW↔HW gap, the macro layer is noise and only Layer 1 survives.

**Flashing:** launcher `/ota/upload?slot=ota_1&confirm=1` path with `autostart:false` set
first (see `CLAUDE.local.md`). The HW build goes on the daily driver for one window only,
ending in an immediate reflash back to SW.

**Held constant:** parked, ignition off, module awake, WiFi (not modem), same 12V
condition. Recorded in the log — e-TNGA poller load differs sharply between AWAKE and
CHARGE states and would contaminate Layer 3.

### Layer 1 — micro: `test crypto sha`

New subcommand under the existing `test` tree in `main/test_framework.cpp`, matching that
file's style and using `esp_timer_get_time()` as its other timed tests do.

`test crypto sha [<iterations>]`, default sized to run in a few seconds.

For each of SHA-1, SHA-256, SHA-512, four cases:

| Case | Size | Rationale |
|---|---|---|
| tiny | 64 B | Per-call overhead dominates. TLS transcript hashing is many small updates and IDF's hardware path locks per block — hardware can lose here. |
| small | 512 B | Typical record / PRF chunk. |
| bulk | 4 KB | Steady-state MB/s; the headline per-byte number. |
| clone | — | `mbedtls_sha256_clone()` alone. On HW this is the `esp_sha_read_digest_state()` call that aborts. |

Buffer allocated `MALLOC_CAP_INTERNAL` so SPIRAM latency does not pollute results. Report
**min, mean, MB/s** per case: min-of-N is the noise floor on a live module, mean is what
users experience.

The command *also* calls IDF's `esp_sha()` directly, so the SW build alone prints both
sides of the ratio; the HW build then confirms the mbedTLS-level number tracks the
primitive-level one.

### Layer 2 — macro (module as TLS server)

**Setup, once, shared by both arms.** Self-signed cert/key installed to
`/store/tls/webserver.crt` and `.key`. `/store` is the config partition and survives OTA
flashes, so this is installed once. The bind happens at `NetManInit`, so a net restart or
reboot is required before :443 is live. `curl -k` throughout — cert validation on the
devbox is irrelevant here. A ~2 MB incompressible random file goes on `/sd` for the bulk
case.

**M1 — handshake latency.** `curl -k -w '%{time_connect} %{time_appconnect}'` against
`https://10.10.10.115/`, N=200 sequential with a short gap. Metric is
`time_appconnect − time_connect` (TCP setup subtracted). Report **median and p95**; WiFi
jitter is heavy-tailed and a mean is outlier-dominated. Exercises RSA sign (hardware MPI,
unaffected), transcript SHA, and PRF HMAC-SHA.

**M2 — bulk throughput, ciphersuite-forced two ways.** Same GET of the 2 MB file, N=10 each,
`curl -k --ciphers <suite> -w '%{speed_download}'`:

- **GCM arm** (`ECDHE-RSA-AES128-GCM-SHA256`): SHA only in handshake/PRF, never per record.
  This is the control; it should show ~zero penalty.
- **CBC-SHA arm** (`ECDHE-RSA-AES128-SHA256`): HMAC-SHA256 per record. The only place a
  genuine per-byte penalty can appear.

The GCM↔CBC delta *within* a build isolates per-record MAC cost more cleanly than the
SW↔HW delta, because it holds the build constant.

Expected complication: ESP32 WiFi + mongoose throughput is often network- or stack-bound
below what the crypto can sustain. If both arms are network-bound, M2's wall-clock numbers
show nothing. That is a real finding, not a failed measurement — but it only becomes a
defensible claim paired with Layer 3, where CPU-seconds per MB still differ at identical
wall-clock throughput.

**M3 — what production negotiates.** `openssl s_client` against the real peers
(`api.openvehicles.com:6870`, the server v3 MQTT/TLS port) to record the negotiated suite.
Determines whether M2's CBC arm describes anything real or is a synthetic worst case.

**M4 — module as TLS client.** The production-representative direction and the crash path.
N=30 `server v3 stop` / `server v3 start` cycles via `/api/execute`. Log timestamp
resolution is too coarse for handshake wall-clock, so M4's primary metric is CPU-seconds
from Layer 3; its secondary output is crash count on the HW arm.

All of M1–M4 run over HTTPS with **no SSH session open**, removing wolfSSH as a confound
and improving the odds the HW arm survives its window. `/api/execute` is `PageAuth_Cookie`,
so the harness logs in once and reuses a cookie jar.

### Layer 3 — load

"Load penalty" is a CPU claim and should not be inferred from wall-clock on a jittery WiFi
link.

`module tasks` exposes per-task runtime counters from `uxTaskGetSystemState`
(`main/ovms_module.cpp:261`), backed by `CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER=y`
(microsecond resolution). Snapshot before and after each batch over `/api/execute`;
difference the mongoose / `OVMS NetMan` task counter and the total.

Derived metrics, comparable across arms:

- CPU-ms per handshake (M1 batch)
- CPU-ms per MB transferred, separately for GCM and CBC arms (M2)
- CPU-ms per server-v3 reconnect (M4)
- Idle baseline: 60 s with no TLS activity, subtracted from each of the above

Free additions: mongoose task **stack high-water** in each arm (software SHA has a
different stack footprint and that task is not generously sized), and **binary size** from
`make size` in CI, which needs no hardware.

**Protocol dependency to validate first:** that `module tasks` counters are cumulative,
monotonic, and do not wrap over a measurement window. Validate on the SW arm — three
snapshots a minute apart, confirm deltas behave — before committing the HW arm's one-shot
window. Fallback if unusable: sample `module tasks` at fixed intervals and integrate.

## Reporting format

A comment on #1321 that dexterbg can act on without re-running anything:

1. **Bottom line, first paragraph.** Raw SHA-256 throughput X→Y MB/s, handshake +Z ms /
   +W CPU-ms, bulk affected/unaffected by N%. Then the framing: hw3.0 has shipped with
   hardware SHA off for years, so this is a cost part of the fleet already pays, and it
   buys removal of an `abort()`.
2. **Micro table.** SHA-1/256/512 × {64 B, 512 B, 4 KB}; hardware MB/s, software MB/s,
   ratio. `mbedtls_sha256_clone()` on its own line. Call out the 64 B row if hardware loses
   there.
3. **Macro table.** Handshake median/p95 (M1 server-side, M4 client-side), bulk MB/s for
   GCM and CBC arms, SW vs HW — with **both** SW runs shown so the reader can see the noise
   floor directly.
4. **CPU cost table.** CPU-ms per handshake, per MB, per reconnect, idle baseline
   subtracted. The headline for a "load" question.
5. **Context and provenance.** Chip revision, both build SHAs, vehicle state, N per
   measurement, production ciphersuite (M3), binary size delta, HW-arm crash count.
6. **Reproduction.** `test crypto sha` as an upstreamable patch plus the curl one-liners —
   what makes this checkable on other hardware rather than a claim about one module in one
   driveway.
7. **What this does not settle.** The test prices the *mitigation*; it does not establish
   that disabling hardware SHA is the *right* fix. Fixing the locking in the IDF 3.3
   mbedTLS port, or avoiding the clone-mid-handshake pattern, are alternatives this data
   does not evaluate. That is the maintainer's call; the goal is to remove the cost
   question from it.

## Threats to validity

Listed in the issue comment, not buried.

| Threat | Handling |
|---|---|
| Single module, single silicon revision (ESP32 rev3) | Report `esp_chip_info` revision. Hardware SHA errata are revision-specific; does not generalize to rev1 without re-running. |
| Live vehicle background load (e-TNGA poller, CAN) | Idle-baseline subtraction, min-of-N for micro, A‑B‑A for macro. Reduced, not eliminated; CPU numbers are not representative of every vehicle module. |
| WiFi RTT jitter dominates handshake wall-clock | High N, median/p95 not mean, CPU-ms as primary metric with wall-clock corroborating. |
| Throughput may be network- or mongoose-bound, masking crypto cost | Expected; CPU-ms per MB carries the claim in that case. Stated as a finding, not hidden. |
| HW arm may crash mid-run, truncating samples | Devbox-side incremental logging, resume rather than restart, discard partial batches, report completion rate. Survival bias called out if it occurs. |
| Micro measures the primitive, not mbedTLS's real call pattern | The 64 B case approximates many-small-updates but does not replicate handshake sequencing. Neither layer is claimed to stand alone. |
| `esp_sha()` availability with `CONFIG_MBEDTLS_HARDWARE_SHA` off | Unverified — no IDF checkout on the devbox (`IDF_PATH` empty). Resolved by the CI compile of commit 1. Fallback (take the micro number from both builds) costs nothing, since both are flashed regardless. |
| Ciphersuite forcing changes cipher and key exchange, not just MAC | GCM↔CBC compared only *within* a build, never across arms. |
| SHA-384/512 software cost disproportionately worse on ESP32 | Reported but not headlined unless M3 shows production negotiates a SHA-384 suite. |
| Toolchain/compiler drift between arms | Same branch, same CI runner, two commits apart. Build SHAs recorded. |

**What this design delivers:** a defensible per-byte cost, a defensible per-handshake CPU
cost, and a defensible statement about bulk transfer.

**What it does not:** a fleet-wide claim, behaviour on rev1 silicon, or behaviour under a
heavier vehicle stack than e-TNGA.

## Out of scope

- Fixing the IDF 3.3 hardware-SHA locking, or eliminating the clone-mid-handshake pattern.
- Whether hardware AES and MPI (both retained) carry similar defects.
- The fork's HTTPS-OTA branch (`feature/ota-streaming-flash`), which *would* add a bulk TLS
  workload but is not upstream and is not what #1321 is about.
