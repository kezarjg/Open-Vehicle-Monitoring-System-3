# Hardware-SHA Penalty Benchmark Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce defensible numbers for the performance/load penalty of `CONFIG_MBEDTLS_HARDWARE_SHA=` (off) on OVMS3 hw3.1, and post them to openvehicles#1321.

**Architecture:** Two firmware builds one sdkconfig line apart (SW = hardware SHA off, HW = on), run A‑B‑A on the in-vehicle Solterra module. Three measurement layers: a new on-device micro benchmark (`test crypto sha`), a devbox-driven macro harness against the module's own HTTPS server plus a client-direction TLS fetch (`test crypto tlsget`), and CPU-seconds accounting via the existing `module tasks` FreeRTOS runtime stats.

**Tech Stack:** C++ on ESP-IDF 3.3.x (legacy `make`), mbedTLS 2.16.x, esp-tls, mongoose 6.11, GitHub Actions CI, MinIO artifact store, `curl`/`openssl` on the devbox.

## Global Constraints

- **The firmware does not build locally.** `IDF_PATH` is empty on this box. The compile gate is GitHub Actions (`.github/workflows/build.yml`), which copies `support/sdkconfig.default.hw31` to `sdkconfig` and runs `make defconfig && make -j`. Never claim a change compiles without a green CI run.
- **Match `main/test_framework.cpp` style exactly:** 2-space indent, opening brace on its own line indented 2 spaces past the declaration, `writer->printf(...)` / `writer->puts(...)` for output, `static const char *TAG = "test";` already defined at top of file.
- **NEVER run `script eval` over SSH.** Any eval, however small, kills the module's SSH server until a reboot. See memory `feedback_never_script_eval_over_ssh`.
- **One OVMS shell command per `ssh` invocation.** No `;`, `&&`, `echo`, or embedded quotes — the OVMS interpreter rejects them.
- **During measurement runs, drive the module over HTTPS `/api/execute`, never SSH.** wolfSSH load is the documented trigger that widens the hardware-SHA race; an open SSH session would both confound the numbers and raise the odds the HW arm crashes mid-run.
- **Flash only via the launcher** (`POST /autostart {"on":false}` → `POST /ota/upload?slot=ota_1&confirm=1` → restore autostart → `POST /launch`). See `CLAUDE.local.md`. Always restore `autostart:true` before `/launch`.
- **Work in a git worktree**, not the shared checkout (memory `feedback_shared_working_dir`).
- **The HW-arm sdkconfig commit is throwaway.** It must never be merged to master — it re-enables the crash under test.
- Vehicle state held constant for every run: parked, ignition off, module awake, on WiFi (not modem). Record it.

---

### Task 0: Worktree and branch

**Files:**
- Create: worktree at `/home/devuser/ovms-sha-bench` on branch `bench/hw-sha-penalty`

**Interfaces:**
- Produces: branch `bench/hw-sha-penalty` off current `master`; all later tasks commit here.

- [ ] **Step 1: Create the worktree**

```bash
cd /home/devuser/Open-Vehicle-Monitoring-System-3
git worktree add -b bench/hw-sha-penalty /home/devuser/ovms-sha-bench master
```

- [ ] **Step 2: Verify branch and clean tree**

```bash
cd /home/devuser/ovms-sha-bench && git branch --show-current && git status --short
```

Expected: prints `bench/hw-sha-penalty` and no file lines.

---

### Task 1: `test crypto sha` micro benchmark

**Files:**
- Modify: `vehicle/OVMS.V3/main/test_framework.cpp` (includes near line 55; new functions before the `TestFrameworkInit` block near line 470; registration inside `TestFrameworkInit::TestFrameworkInit()` near line 499)

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: OVMS shell command `test crypto sha [<iterations>]`. Output is one line per (algorithm, size) case plus one clone line, each `min <n> ns/op  mean <n> ns/op  <n> KB/s`. Task 7/8/9 parse these lines.

This task also resolves the plan's one load-bearing unknown: whether IDF's `esp_sha()` links when `CONFIG_MBEDTLS_HARDWARE_SHA` is off. If Step 3's CI run fails to link `esp_sha`, apply the fallback in Step 4.

- [ ] **Step 1: Add the includes**

In `vehicle/OVMS.V3/main/test_framework.cpp`, immediately after the existing `#include "file_writer.h"` line, add:

```cpp
#include "esp_heap_caps.h"
#include "hwcrypto/sha.h"
#include "mbedtls/sha1.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"
```

- [ ] **Step 2: Add the benchmark functions**

Insert immediately before the `class TestFrameworkInit` declaration near line 470:

```cpp
#define TEST_SHA_BUFSIZE 4096

typedef enum
  {
  TEST_SHA_MBED_1 = 0,
  TEST_SHA_MBED_256,
  TEST_SHA_MBED_512,
  TEST_SHA_IDF_HW256
  } test_sha_algo_t;

// One hash of <len> bytes. The explicit context API (init/starts/update/finish) is used
// rather than the one-shot mbedtls_shaN_ret() helpers because the IDF hardware port
// replaces the context functions but does not reliably provide the one-shot wrappers,
// and because init/starts/update/finish is what TLS actually does.
static void test_crypto_sha_once(test_sha_algo_t algo, const unsigned char* buf, size_t len,
                                 unsigned char* out)
  {
  switch (algo)
    {
    case TEST_SHA_MBED_1:
      {
      mbedtls_sha1_context ctx;
      mbedtls_sha1_init(&ctx);
      mbedtls_sha1_starts_ret(&ctx);
      mbedtls_sha1_update_ret(&ctx, buf, len);
      mbedtls_sha1_finish_ret(&ctx, out);
      mbedtls_sha1_free(&ctx);
      break;
      }
    case TEST_SHA_MBED_256:
      {
      mbedtls_sha256_context ctx;
      mbedtls_sha256_init(&ctx);
      mbedtls_sha256_starts_ret(&ctx, 0);
      mbedtls_sha256_update_ret(&ctx, buf, len);
      mbedtls_sha256_finish_ret(&ctx, out);
      mbedtls_sha256_free(&ctx);
      break;
      }
    case TEST_SHA_MBED_512:
      {
      mbedtls_sha512_context ctx;
      mbedtls_sha512_init(&ctx);
      mbedtls_sha512_starts_ret(&ctx, 0);
      mbedtls_sha512_update_ret(&ctx, buf, len);
      mbedtls_sha512_finish_ret(&ctx, out);
      mbedtls_sha512_free(&ctx);
      break;
      }
    case TEST_SHA_IDF_HW256:
      esp_sha(SHA2_256, buf, len, out);
      break;
    }
  }

// Times <rounds> batches of <batch> hashes. Reports the best batch (noise floor on a live
// module) and the mean. Timing a batch rather than each op keeps esp_timer_get_time()
// overhead from dominating the 64-byte case.
static void test_crypto_sha_bench(OvmsWriter* writer, const char* name, test_sha_algo_t algo,
                                  const unsigned char* buf, size_t len, int rounds, int batch)
  {
  unsigned char out[64];
  int64_t best = -1;
  int64_t total = 0;

  for (int r=0; r<rounds; r++)
    {
    int64_t t0 = esp_timer_get_time();
    for (int i=0; i<batch; i++)
      test_crypto_sha_once(algo, buf, len, out);
    int64_t dt = esp_timer_get_time() - t0;
    total += dt;
    if ((best < 0) || (dt < best)) best = dt;
    }

  int64_t minns = (best * 1000) / batch;
  int64_t meanns = (total * 1000) / ((int64_t)rounds * batch);
  int64_t kbps = (best > 0) ? (((int64_t)len * batch * 1000000LL) / best) / 1024 : 0;

  writer->printf("%-16s %5d B  min %8lld ns/op  mean %8lld ns/op  %7lld KB/s\n",
    name, (int)len, minns, meanns, kbps);
  }

// mbedtls_sha256_clone() in isolation. On a hardware-SHA build this is the
// esp_sha_read_digest_state() call that aborts in openvehicles#1321.
static void test_crypto_sha_clone(OvmsWriter* writer, const unsigned char* buf,
                                  int rounds, int batch)
  {
  mbedtls_sha256_context src;
  mbedtls_sha256_init(&src);
  mbedtls_sha256_starts_ret(&src, 0);
  mbedtls_sha256_update_ret(&src, buf, 512);

  int64_t best = -1;
  int64_t total = 0;

  for (int r=0; r<rounds; r++)
    {
    int64_t t0 = esp_timer_get_time();
    for (int i=0; i<batch; i++)
      {
      mbedtls_sha256_context dst;
      mbedtls_sha256_init(&dst);
      mbedtls_sha256_clone(&dst, &src);
      mbedtls_sha256_free(&dst);
      }
    int64_t dt = esp_timer_get_time() - t0;
    total += dt;
    if ((best < 0) || (dt < best)) best = dt;
    }

  mbedtls_sha256_free(&src);

  writer->printf("%-16s %5s    min %8lld ns/op  mean %8lld ns/op\n",
    "sha256 clone", "-",
    (best * 1000) / batch,
    (total * 1000) / ((int64_t)rounds * batch));
  }

void test_crypto_sha(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  int iters = 2000;
  if (argc > 0) iters = atoi(argv[0]);
  if (iters < 10) iters = 10;

  int rounds = 10;
  int batch = iters / rounds;
  if (batch < 1) batch = 1;

  unsigned char* buf = (unsigned char*)heap_caps_malloc(TEST_SHA_BUFSIZE,
    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (buf == NULL)
    {
    writer->puts("Error: cannot allocate internal RAM buffer");
    return;
    }
  for (int i=0; i<TEST_SHA_BUFSIZE; i++)
    buf[i] = (unsigned char)((i * 31) + 7);

#ifdef CONFIG_MBEDTLS_HARDWARE_SHA
  writer->puts("mbedTLS hardware SHA: ENABLED");
#else
  writer->puts("mbedTLS hardware SHA: disabled (software)");
#endif
  writer->printf("rounds=%d batch=%d\n", rounds, batch);

  static const size_t sizes[] = { 64, 512, 4096 };
  for (int s=0; s<3; s++)
    {
    test_crypto_sha_bench(writer, "mbed sha1", TEST_SHA_MBED_1, buf, sizes[s], rounds, batch);
    test_crypto_sha_bench(writer, "mbed sha256", TEST_SHA_MBED_256, buf, sizes[s], rounds, batch);
    test_crypto_sha_bench(writer, "mbed sha512", TEST_SHA_MBED_512, buf, sizes[s], rounds, batch);
    test_crypto_sha_bench(writer, "idf esp_sha256", TEST_SHA_IDF_HW256, buf, sizes[s], rounds, batch);
    }

  test_crypto_sha_clone(writer, buf, rounds, batch);

  free(buf);
  }
```

- [ ] **Step 3: Register the command**

Inside `TestFrameworkInit::TestFrameworkInit()`, immediately after the existing `cmd_test->RegisterCommand("filewriter", ...)` line, add:

```cpp
  OvmsCommand* cmd_crypto = cmd_test->RegisterCommand("crypto", "Test crypto performance");
  cmd_crypto->RegisterCommand("sha", "Benchmark SHA-1/256/512 (mbedTLS + IDF hardware)",
    test_crypto_sha, "[<iterations>]", 0, 1);
```

- [ ] **Step 4: Commit and push to trigger CI**

```bash
cd /home/devuser/ovms-sha-bench
git add vehicle/OVMS.V3/main/test_framework.cpp
git commit -m "test: add 'test crypto sha' SHA micro benchmark

Times mbedTLS SHA-1/256/512 at 64/512/4096 bytes plus mbedtls_sha256_clone(),
and IDF's esp_sha() directly so a single build reports both the software and
hardware sides of the ratio. For quantifying openvehicles#1321."
git push -u origin bench/hw-sha-penalty
```

- [ ] **Step 5: Verify CI compiles it — this is the real gate**

```bash
gh run list --branch bench/hw-sha-penalty --limit 3 --json workflowName,status,conclusion,databaseId
```

Wait for `Firmware build` to reach `conclusion: success`.

**If it fails with an undefined reference to `esp_sha` or a missing `hwcrypto/sha.h`:** the plan's assumption was wrong. Apply the fallback — delete the `TEST_SHA_IDF_HW256` enum value, its `case` in `test_crypto_sha_once`, and its `test_crypto_sha_bench` call, and drop the `hwcrypto/sha.h` include. The hardware number then comes from the HW build only (Task 3), which costs nothing since both builds are flashed anyway. Record the outcome in the plan file and re-run this step.

- [ ] **Step 6: Record the run id for the SW binary**

```bash
gh run list --branch bench/hw-sha-penalty --limit 1 --json databaseId --jq '.[0].databaseId'
```

Save as `SW_RUN_ID`. The binary is at MinIO `sh/ci-artifacts/$SW_RUN_ID/ovms3/ovms3.bin`.

---

### Task 2: `test crypto tlsget` client-direction TLS benchmark

**Files:**
- Modify: `vehicle/OVMS.V3/main/test_framework.cpp` (add include; add function; add registration)
- Modify: `vehicle/OVMS.V3/changes.txt` (top `????-??-?? ??? ??????? OTA release` block)

**Interfaces:**
- Consumes: `test crypto` command node created in Task 1 Step 3 (`cmd_crypto`).
- Produces: OVMS shell command `test crypto tlsget <url> [<count>]`. Prints one line per fetch: `#<n> handshake <ms> ms  <bytes> bytes in <ms> ms = <n> KB/s  cipher <name>`.

- [ ] **Step 1: Add the include**

Immediately after the `#include "mbedtls/sha512.h"` line added in Task 1, add:

```cpp
#include "esp_tls.h"
#include "ovms_tls.h"
```

- [ ] **Step 2: Add the fetch function**

Insert immediately before the `class TestFrameworkInit` declaration (after `test_crypto_sha`):

```cpp
#define TEST_TLS_RXBUF 1536

void test_crypto_tlsget(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
  {
  std::string url(argv[0]);
  int count = 1;
  if (argc > 1) count = atoi(argv[1]);
  if (count < 1) count = 1;

  // Split the URL so we can build the request line; esp_tls_conn_http_new() wants the
  // full URL, but we still need host and path for the GET.
  std::string rest(url);
  if (rest.compare(0, 8, "https://", 8) == 0)
    rest = rest.substr(8);
  std::string host(rest);
  std::string path("/");
  size_t delim = rest.find('/');
  if (delim != std::string::npos)
    {
    host = rest.substr(0, delim);
    path = rest.substr(delim);
    }

  char* ca = MyOvmsTLS.GetTrustedList();
  if (ca == NULL)
    {
    writer->puts("Error: no trusted CA list available");
    return;
    }

  unsigned char* rxbuf = (unsigned char*)heap_caps_malloc(TEST_TLS_RXBUF,
    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (rxbuf == NULL)
    {
    writer->puts("Error: cannot allocate internal RAM buffer");
    return;
    }

  std::string req("GET ");
  req.append(path);
  req.append(" HTTP/1.0\r\nHost: ");
  req.append(host);
  req.append("\r\nConnection: close\r\n\r\n");

  for (int n=0; n<count; n++)
    {
    esp_tls_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.cacert_pem_buf = (const unsigned char*)ca;
    cfg.cacert_pem_bytes = strlen(ca) + 1;

    int64_t t0 = esp_timer_get_time();
    struct esp_tls* tls = esp_tls_conn_http_new(url.c_str(), &cfg);
    if (tls == NULL)
      {
      writer->printf("#%d ERROR: TLS connect failed\n", n+1);
      continue;
      }
    int64_t t1 = esp_timer_get_time();

    size_t sent = 0;
    while (sent < req.length())
      {
      int w = esp_tls_conn_write(tls, req.c_str() + sent, req.length() - sent);
      if (w <= 0) break;
      sent += w;
      }

    int64_t total = 0;
    for (;;)
      {
      int rd = esp_tls_conn_read(tls, (char*)rxbuf, TEST_TLS_RXBUF);
      if (rd <= 0) break;
      total += rd;
      }
    int64_t t2 = esp_timer_get_time();

    const char* suite = mbedtls_ssl_get_ciphersuite(&tls->ssl);
    int64_t hs_ms = (t1 - t0) / 1000;
    int64_t body_us = t2 - t1;
    int64_t kbps = (body_us > 0) ? ((total * 1000000LL) / body_us) / 1024 : 0;

    writer->printf("#%d handshake %lld ms  %lld bytes in %lld ms = %lld KB/s  cipher %s\n",
      n+1, hs_ms, total, body_us / 1000, kbps, (suite != NULL) ? suite : "?");

    esp_tls_conn_delete(tls);
    }

  free(rxbuf);
  }
```

- [ ] **Step 3: Register the command**

Immediately after the `cmd_crypto->RegisterCommand("sha", ...)` line from Task 1, add:

```cpp
  cmd_crypto->RegisterCommand("tlsget", "Time an HTTPS GET (esp-tls/mbedTLS), discarding the body",
    test_crypto_tlsget, "<url> [<count>]", 1, 2);
```

- [ ] **Step 4: Add the changes.txt entry**

In `vehicle/OVMS.V3/changes.txt`, inside the top `????-??-?? ???  ???????  OTA release` block, add below the existing hardware-SHA bullet:

```
- New diagnostic commands `test crypto sha` (SHA-1/256/512 micro benchmark, mbedTLS and the
    ESP32 hardware engine) and `test crypto tlsget <url> [<count>]` (times an HTTPS GET via
    esp-tls, discarding the body, and reports the negotiated ciphersuite). Added to quantify
    the cost of disabling mbedTLS hardware SHA.
```

- [ ] **Step 5: Commit and push**

```bash
cd /home/devuser/ovms-sha-bench
git add vehicle/OVMS.V3/main/test_framework.cpp vehicle/OVMS.V3/changes.txt
git commit -m "test: add 'test crypto tlsget' HTTPS fetch benchmark

Times handshake and body transfer over esp-tls (the same stack the HTTPS OTA
path in feature/ota-streaming-flash uses) and reports the negotiated
ciphersuite. Body is discarded so the result isolates crypto and upper-bounds
the SHA contribution to an OTA download."
git push
```

- [ ] **Step 6: Verify CI**

```bash
gh run list --branch bench/hw-sha-penalty --limit 3 --json workflowName,status,conclusion,databaseId
```

Expected: `Firmware build` → `success`.

**If it fails on `tls->ssl` (incomplete type `struct esp_tls`):** IDF 3.3 does not expose the struct. Fallback — delete the `const char* suite = ...` line, pass `"(see server log)"` in its place, and obtain the negotiated suite from the bench server's log in Task 4 instead. Re-run this step.

- [ ] **Step 7: Update SW_RUN_ID**

```bash
gh run list --branch bench/hw-sha-penalty --limit 1 --json databaseId --jq '.[0].databaseId'
```

This run's binary is the **SW arm**. Save as `SW_RUN_ID`.

---

### Task 3: HW-arm build (throwaway commit)

**Files:**
- Modify: `vehicle/OVMS.V3/support/sdkconfig.default.hw31:772`

**Interfaces:**
- Consumes: branch state after Task 2.
- Produces: a second CI build whose binary is the **HW arm**. Commit is never merged.

- [ ] **Step 1: Flip the config line**

In `vehicle/OVMS.V3/support/sdkconfig.default.hw31`, change line 772 from:

```
CONFIG_MBEDTLS_HARDWARE_SHA=
```

to:

```
CONFIG_MBEDTLS_HARDWARE_SHA=y
```

- [ ] **Step 2: Commit on a throwaway branch, not the bench branch**

```bash
cd /home/devuser/ovms-sha-bench
git checkout -b bench/hw-sha-penalty-hwarm
git add vehicle/OVMS.V3/support/sdkconfig.default.hw31
git commit -m "TEMP DO NOT MERGE: re-enable mbedTLS hardware SHA for A/B measurement

Throwaway measurement commit for openvehicles#1321. Re-enables the crash under
test. Must never reach master."
git push -u origin bench/hw-sha-penalty-hwarm
```

- [ ] **Step 3: Verify CI and capture the HW run id**

```bash
gh run list --branch bench/hw-sha-penalty-hwarm --limit 1 --json databaseId,conclusion
```

Expected: `conclusion: success`. Save `databaseId` as `HW_RUN_ID`.

- [ ] **Step 4: Return to the bench branch and record binary size for both arms**

```bash
cd /home/devuser/ovms-sha-bench && git checkout bench/hw-sha-penalty
mc cp sh/ci-artifacts/$SW_RUN_ID/ovms3/ovms3.bin /tmp/claude-1000/-home-devuser-Open-Vehicle-Monitoring-System-3/ac54855e-229c-4710-b824-3af623364bc1/scratchpad/ovms3-sw.bin
mc cp sh/ci-artifacts/$HW_RUN_ID/ovms3/ovms3.bin /tmp/claude-1000/-home-devuser-Open-Vehicle-Monitoring-System-3/ac54855e-229c-4710-b824-3af623364bc1/scratchpad/ovms3-hw.bin
ls -l /tmp/claude-1000/-home-devuser-Open-Vehicle-Monitoring-System-3/ac54855e-229c-4710-b824-3af623364bc1/scratchpad/ovms3-*.bin
```

Record both sizes — the delta is a free reportable number (Layer 3's "binary size" item).

---

### Task 4: Bench infrastructure (module TLS server + os-k3s HTTPS endpoints)

**Files:**
- Create on devbox: `<scratchpad>/bench-ca.key`, `<scratchpad>/bench-ca.pem`, `<scratchpad>/module.key`, `<scratchpad>/module.crt`, `<scratchpad>/bench-server.key`, `<scratchpad>/bench-server.crt`
- Create on os-k3s: `~/sha-bench/serve.py`, `~/sha-bench/blob.bin`
- Create on module: `/store/tls/webserver.crt`, `/store/tls/webserver.key`, `/store/trustedca/bench-ca.pem`

**Interfaces:**
- Consumes: nothing from firmware tasks; can run in parallel with Tasks 1–3.
- Produces: module HTTPS on `https://10.10.10.115/`; GCM-only endpoint `https://10.10.5.20:8443/blob.bin`; CBC-only endpoint `https://10.10.5.20:8444/blob.bin`; a bench CA the module trusts.

- [ ] **Step 1: Generate a bench CA and two leaf certs**

```bash
cd /tmp/claude-1000/-home-devuser-Open-Vehicle-Monitoring-System-3/ac54855e-229c-4710-b824-3af623364bc1/scratchpad
openssl req -x509 -newkey rsa:2048 -nodes -days 90 \
  -keyout bench-ca.key -out bench-ca.pem -subj "/CN=OVMS SHA Bench CA"
# module webserver cert (CN only, no SAN - IDF 3.3 mbedTLS wants CN for IP peers)
openssl req -newkey rsa:2048 -nodes -keyout module.key -out module.csr -subj "/CN=10.10.10.115"
openssl x509 -req -in module.csr -CA bench-ca.pem -CAkey bench-ca.key -CAcreateserial \
  -days 90 -out module.crt
# os-k3s bench server cert
openssl req -newkey rsa:2048 -nodes -keyout bench-server.key -out bench-server.csr -subj "/CN=10.10.5.20"
openssl x509 -req -in bench-server.csr -CA bench-ca.pem -CAkey bench-ca.key -CAcreateserial \
  -days 90 -out bench-server.crt
```

- [ ] **Step 2: Install the module's HTTPS cert and the bench CA**

The module's web `/api/file` endpoint saves files. Log in first and reuse the cookie jar (replace `<pw>` with the module's web password):

```bash
cd /tmp/claude-1000/-home-devuser-Open-Vehicle-Monitoring-System-3/ac54855e-229c-4710-b824-3af623364bc1/scratchpad
curl -s -c cookies.txt -d "username=admin&password=<pw>" http://10.10.10.115/login >/dev/null
curl -s -b cookies.txt -F "path=/store/tls/webserver.crt" -F "content=<module.crt" http://10.10.10.115/api/file
curl -s -b cookies.txt -F "path=/store/tls/webserver.key" -F "content=<module.key" http://10.10.10.115/api/file
curl -s -b cookies.txt -F "path=/store/trustedca/bench-ca.pem" -F "content=<bench-ca.pem" http://10.10.10.115/api/file
```

- [ ] **Step 3: Reload the trust list and restart networking so :443 binds**

```bash
ssh solterra-ovms 'tls trust reload'
ssh solterra-ovms 'module reset'
```

Wait for the module to come back (the launcher takes ~50 s, then the guest boots).

- [ ] **Step 4: Verify the module's HTTPS server is up**

```bash
curl -sk -o /dev/null -w '%{http_code} %{time_appconnect}\n' https://10.10.10.115/
```

Expected: a `200` (or `302`) and a non-zero appconnect time. If the connection is refused, `/store/tls/webserver.crt` or `.key` is missing — re-check Step 2.

- [ ] **Step 5: Create the bench blob and cipher-pinned servers on os-k3s**

```bash
ssh os-k3s 'mkdir -p ~/sha-bench && head -c 3145728 /dev/urandom > ~/sha-bench/blob.bin'
scp bench-server.crt bench-server.key os-k3s:~/sha-bench/
```

Write `~/sha-bench/serve.py` on os-k3s:

```python
import http.server, ssl, sys, os
port, ciphers = int(sys.argv[1]), sys.argv[2]
os.chdir(os.path.expanduser("~/sha-bench"))
ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ctx.load_cert_chain("bench-server.crt", "bench-server.key")
ctx.minimum_version = ssl.TLSVersion.TLSv1_2
ctx.maximum_version = ssl.TLSVersion.TLSv1_2
ctx.set_ciphers(ciphers)
httpd = http.server.HTTPServer(("0.0.0.0", port), http.server.SimpleHTTPRequestHandler)
httpd.socket = ctx.wrap_socket(httpd.socket, server_side=True)
print("serving %d with %s" % (port, ciphers), flush=True)
httpd.serve_forever()
```

- [ ] **Step 6: Start both endpoints and verify their pinned suites**

```bash
ssh os-k3s 'cd ~/sha-bench && nohup python3 serve.py 8443 ECDHE-RSA-AES128-GCM-SHA256 >gcm.log 2>&1 &'
ssh os-k3s 'cd ~/sha-bench && nohup python3 serve.py 8444 ECDHE-RSA-AES128-SHA256 >cbc.log 2>&1 &'
openssl s_client -connect 10.10.5.20:8443 -tls1_2 </dev/null 2>/dev/null | grep '^New,'
openssl s_client -connect 10.10.5.20:8444 -tls1_2 </dev/null 2>/dev/null | grep '^New,'
```

Expected: `ECDHE-RSA-AES128-GCM-SHA256` and `ECDHE-RSA-AES128-SHA256` respectively.

---

### Task 5: Devbox measurement harness

**Files:**
- Create: `scripts/sha-bench/run-arm.sh` (committed on the bench branch only)

**Interfaces:**
- Consumes: module HTTPS from Task 4; `test crypto sha` / `test crypto tlsget` from Tasks 1–2.
- Produces: `run-arm.sh <label>` writing `results-<label>/` containing `micro.txt`, `handshake.csv`, `bulk-gcm.csv`, `bulk-cbc.csv`, `tlsget-gcm.txt`, `tlsget-cbc.txt`, `tasks-<phase>.txt`.

- [ ] **Step 1: Write the harness**

Create `scripts/sha-bench/run-arm.sh`:

```bash
#!/bin/bash
# Runs one arm (SW or HW) of the openvehicles#1321 hardware-SHA A/B.
# Everything is driven over HTTPS /api/execute; no SSH session is opened, because
# wolfSSH load is the documented trigger for the crash under test.
set -u
LABEL="${1:?usage: run-arm.sh <label>}"
MODULE="${OVMS_MODULE:-10.10.10.115}"
PW="${OVMS_PW:?set OVMS_PW}"
BENCH="${BENCH_HOST:-10.10.5.20}"
OUT="results-$LABEL"
mkdir -p "$OUT"
JAR="$OUT/cookies.txt"

curl -sk -c "$JAR" -d "username=admin&password=$PW" "https://$MODULE/login" >/dev/null

ovms() {  # one OVMS command, over HTTPS, output to stdout
  curl -sk -b "$JAR" --data-urlencode "command=$1" "https://$MODULE/api/execute"
}

snap() { ovms "module tasks" > "$OUT/tasks-$1.txt"; }

echo "== idle baseline (60s) =="
snap idle-before
sleep 60
snap idle-after

echo "== micro =="
ovms "test crypto sha 2000" | tee "$OUT/micro.txt"

echo "== M1 handshake x200 =="
snap m1-before
: > "$OUT/handshake.csv"
for i in $(seq 200); do
  curl -sk -o /dev/null -w '%{time_connect},%{time_appconnect}\n' \
    "https://$MODULE/" >> "$OUT/handshake.csv"
  sleep 0.2
done
snap m1-after

echo "== M2 bulk (module as server) =="
snap m2-before
for suite in ECDHE-RSA-AES128-GCM-SHA256:gcm ECDHE-RSA-AES128-SHA256:cbc; do
  c="${suite%%:*}"; tag="${suite##*:}"
  : > "$OUT/bulk-$tag.csv"
  for i in $(seq 10); do
    curl -sk --ciphers "$c" -o /dev/null -w '%{speed_download},%{size_download}\n' \
      "https://$MODULE/script.js.gz" >> "$OUT/bulk-$tag.csv"
  done
done
snap m2-after

echo "== M5 client bulk (module as client, esp-tls) =="
snap m5-before
ovms "test crypto tlsget https://$BENCH:8443/blob.bin 5" | tee "$OUT/tlsget-gcm.txt"
ovms "test crypto tlsget https://$BENCH:8444/blob.bin 5" | tee "$OUT/tlsget-cbc.txt"
snap m5-after

echo "== M4 server v3 reconnect x30 =="
snap m4-before
for i in $(seq 30); do
  ovms "server v3 stop" >/dev/null
  sleep 2
  ovms "server v3 start" >/dev/null
  sleep 4
done
snap m4-after

echo "== done: $OUT =="
```

- [ ] **Step 2: Make it executable and commit**

```bash
cd /home/devuser/ovms-sha-bench
chmod +x scripts/sha-bench/run-arm.sh
git add scripts/sha-bench/run-arm.sh
git commit -m "bench: devbox harness for the hardware-SHA A/B (not for upstream)"
git push
```

Note: this commit touches only `scripts/`, so `Firmware build` runs but is unaffected; no CI wait needed.

---

### Task 6: Validate the `module tasks` counter protocol

**Files:** none — verification only.

**Interfaces:**
- Consumes: SW build flashed on the module (flash it in this task).
- Produces: a go/no-go on using counter deltas as the CPU metric, which Tasks 7–9 depend on.

- [ ] **Step 1: Flash the SW build via the launcher**

```bash
cd /tmp/claude-1000/-home-devuser-Open-Vehicle-Monitoring-System-3/ac54855e-229c-4710-b824-3af623364bc1/scratchpad
ssh solterra-ovms 'module reset'
until curl -s --max-time 2 http://10.10.10.115/status | grep -q linger_remaining_ms; do sleep 0.5; done
curl -s -X POST http://10.10.10.115/autostart -d '{"on":false}'
curl -s http://10.10.10.115/status          # confirm default_slot before writing
curl -s -X POST -H "Content-Type: application/octet-stream" --data-binary "@ovms3-sw.bin" \
  "http://10.10.10.115/ota/upload?slot=ota_1&confirm=1"
curl -s -X POST http://10.10.10.115/autostart -d '{"on":true}'
curl -s -X POST http://10.10.10.115/launch -d '{"slot":1}'
```

- [ ] **Step 2: Confirm the running build**

```bash
ssh solterra-ovms 'ota status'
```

Expected: `Running partition: ota_1` and a firmware hash matching the Task 2 commit's CI build.

- [ ] **Step 3: Take three `module tasks` snapshots a minute apart**

```bash
for i in 1 2 3; do
  ssh solterra-ovms 'module tasks' > /tmp/claude-1000/-home-devuser-Open-Vehicle-Monitoring-System-3/ac54855e-229c-4710-b824-3af623364bc1/scratchpad/tasks-$i.txt
  sleep 60
done
```

- [ ] **Step 4: Verify counters are cumulative and monotonic**

Compare the runtime column for the mongoose/`OVMS NetMan` task across the three files. Expected: strictly increasing, with the two deltas within the same order of magnitude on an idle parked module.

**If the counters reset, wrap, or are percentages rather than cumulative microseconds:** switch to the documented fallback — sample `module tasks` every 10 s throughout each batch and integrate. Record which method was used; it must be the same for all three runs.

- [ ] **Step 5: Record the vehicle/environment state**

```bash
ssh solterra-ovms 'metrics list v.e.on'
ssh solterra-ovms 'wifi status'
```

Save alongside the results. Every arm must run in the same state.

---

### Task 7: Run A1 (SW arm, first pass)

**Files:** produces `results-sw1/` on the devbox.

**Interfaces:**
- Consumes: harness from Task 5, SW build already flashed in Task 6.
- Produces: baseline results; Task 9 compares against them.

- [ ] **Step 1: Confirm no SSH session is open, then run the arm**

```bash
cd /home/devuser/ovms-sha-bench/scripts/sha-bench
OVMS_PW='<pw>' ./run-arm.sh sw1
```

Runtime is roughly 15 minutes.

- [ ] **Step 2: Sanity-check the output**

```bash
head -20 results-sw1/micro.txt
wc -l results-sw1/handshake.csv results-sw1/bulk-gcm.csv results-sw1/bulk-cbc.csv
grep cipher results-sw1/tlsget-gcm.txt results-sw1/tlsget-cbc.txt
```

Expected: micro shows `mbedTLS hardware SHA: disabled (software)`; 200 handshake rows; 10 rows per bulk file; the tlsget lines report the pinned suites.

- [ ] **Step 3: Confirm the module did not reboot mid-run**

```bash
ssh solterra-ovms 'boot status'
```

Expected: no new crash. Record the boot count.

---

### Task 8: Run B (HW arm) — the exposure window

**Files:** produces `results-hw/` on the devbox.

**Interfaces:**
- Consumes: `ovms3-hw.bin` from Task 3, harness from Task 5.
- Produces: the hardware-SHA arm results plus a crash count.

This is the only window in which the daily driver runs the crash-prone build. Start it and finish it in one sitting, and reflash SW immediately afterwards.

- [ ] **Step 1: Flash the HW build**

```bash
cd /tmp/claude-1000/-home-devuser-Open-Vehicle-Monitoring-System-3/ac54855e-229c-4710-b824-3af623364bc1/scratchpad
ssh solterra-ovms 'module reset'
until curl -s --max-time 2 http://10.10.10.115/status | grep -q linger_remaining_ms; do sleep 0.5; done
curl -s -X POST http://10.10.10.115/autostart -d '{"on":false}'
curl -s -X POST -H "Content-Type: application/octet-stream" --data-binary "@ovms3-hw.bin" \
  "http://10.10.10.115/ota/upload?slot=ota_1&confirm=1"
curl -s -X POST http://10.10.10.115/autostart -d '{"on":true}'
curl -s -X POST http://10.10.10.115/launch -d '{"slot":1}'
```

- [ ] **Step 2: Record the pre-run boot count**

```bash
ssh solterra-ovms 'boot status'
```

- [ ] **Step 3: Run the arm**

```bash
cd /home/devuser/ovms-sha-bench/scripts/sha-bench
OVMS_PW='<pw>' ./run-arm.sh hw
```

**If the module reboots mid-run:** note which batch was in flight, discard that batch's partial file, and re-run only that batch. Do not restart the whole arm — record the completion rate instead.

- [ ] **Step 4: Record the post-run boot count and any crash signature**

```bash
ssh solterra-ovms 'boot status'
```

Crash count = post minus pre. If it crashed, capture the backtrace and confirm it is the `esp_sha_read_digest_state` signature — that is a reportable result in its own right.

- [ ] **Step 5: Confirm the micro output shows the hardware path**

```bash
head -3 results-hw/micro.txt
```

Expected: `mbedTLS hardware SHA: ENABLED`. If it says disabled, the wrong binary was flashed — stop and reflash.

- [ ] **Step 6: Reflash the SW build immediately**

Repeat Task 6 Step 1 with `ovms3-sw.bin`, then `ssh solterra-ovms 'ota status'` to confirm. The daily driver must not be left on the HW build.

---

### Task 9: Run A2 (SW arm, repeat) and analyse

**Files:** produces `results-sw2/` and `<scratchpad>/analysis.md`.

**Interfaces:**
- Consumes: `results-sw1/`, `results-hw/`.
- Produces: the numbers Task 10 reports.

- [ ] **Step 1: Run the arm again on the reflashed SW build**

```bash
cd /home/devuser/ovms-sha-bench/scripts/sha-bench
OVMS_PW='<pw>' ./run-arm.sh sw2
```

- [ ] **Step 2: Compute handshake median and p95 for all three runs**

```bash
for r in sw1 hw sw2; do
  awk -F, '{print ($2-$1)*1000}' results-$r/handshake.csv | sort -n > /tmp/hs-$r.txt
  n=$(wc -l < /tmp/hs-$r.txt)
  echo "$r median=$(sed -n "$((n/2))p" /tmp/hs-$r.txt) p95=$(sed -n "$((n*95/100))p" /tmp/hs-$r.txt) ms"
done
```

- [ ] **Step 3: Compute bulk throughput means**

```bash
for r in sw1 hw sw2; do
  for t in gcm cbc; do
    echo "$r $t $(awk -F, '{s+=$1} END {printf "%.0f B/s", s/NR}' results-$r/bulk-$t.csv)"
  done
done
```

- [ ] **Step 4: Apply the noise-floor gate**

Compare `sw1` against `sw2` for each metric. If `|sw1 − sw2|` exceeds `|mean(sw1,sw2) − hw|`, that macro metric is **noise** and must be reported as "below the noise floor of this setup", not as a delta. Apply this per metric, not globally — the micro layer is unaffected either way.

- [ ] **Step 5: Compute CPU-ms per unit from the task snapshots**

For each phase, subtract the `-before` snapshot's mongoose/`OVMS NetMan` runtime from the `-after` snapshot's, subtract the idle baseline scaled to the same wall-clock duration, and divide by the unit count (200 handshakes, 10 bulk fetches, 5 tlsget fetches, 30 reconnects).

- [ ] **Step 6: Extract mongoose stack high-water for both arms**

`module tasks` reports a stack column. Pull the mongoose / `OVMS NetMan` row from
`results-sw1/tasks-m1-after.txt` and `results-hw/tasks-m1-after.txt` and compare. Software
SHA has a different stack footprint and that task is not generously sized, so a materially
smaller headroom on the SW arm is worth reporting even though it is not a performance
number.

- [ ] **Step 7: Write the analysis**

Create `<scratchpad>/analysis.md` with the four tables the spec's reporting format calls for (micro, macro, CPU cost, provenance), each with sw1 / hw / sw2 columns so the noise floor is visible.

- [ ] **Step 8: Commit the raw results**

```bash
cd /home/devuser/ovms-sha-bench
mkdir -p scripts/sha-bench/results
cp -r scripts/sha-bench/results-sw1 scripts/sha-bench/results-hw scripts/sha-bench/results-sw2 scripts/sha-bench/results/
git add scripts/sha-bench/results
git commit -m "bench: raw A-B-A results for the hardware-SHA penalty measurement"
git push
```

---

### Task 10: Report to openvehicles#1321

**Files:** creates `<scratchpad>/issue-comment.md`.

**Interfaces:**
- Consumes: `<scratchpad>/analysis.md`.
- Produces: a posted comment.

- [ ] **Step 1: Draft the comment**

Follow the spec's reporting format exactly: bottom line first; micro table; macro table with both SW runs shown; CPU cost table; the HTTPS-OTA forward look; provenance (chip revision, both build SHAs, vehicle state, N per measurement, M3 ciphersuites, binary size delta, crash count); reproduction (the two commands as a patch, plus the curl one-liners); and the threats-to-validity table from the spec.

Get the chip revision for provenance:

```bash
ssh solterra-ovms 'module status'
```

- [ ] **Step 2: Have the user review it before posting**

The comment goes to an upstream issue under the user's name. Show the draft and wait for explicit approval — do not post unreviewed.

- [ ] **Step 3: Post it**

```bash
gh issue comment 1321 --repo openvehicles/Open-Vehicle-Monitoring-System-3 \
  --body-file /tmp/claude-1000/-home-devuser-Open-Vehicle-Monitoring-System-3/ac54855e-229c-4710-b824-3af623364bc1/scratchpad/issue-comment.md
```

- [ ] **Step 4: Delete the throwaway HW branch**

```bash
git push origin --delete bench/hw-sha-penalty-hwarm
git branch -D bench/hw-sha-penalty-hwarm
```

- [ ] **Step 5: Stop the bench servers**

```bash
ssh os-k3s 'pkill -f "serve.py 8443"; pkill -f "serve.py 8444"'
```

---

## Notes for the implementer

- **`main/CMakeLists.txt` is not the build path.** It calls `idf_component_register` with no `REQUIRES`, so it could not resolve `mongoose` either — CI and the devcontainer both use the legacy `make` build, where every component's include directories are global. No CMake change is needed for the new `esp_tls.h` / `mbedtls/*` / `hwcrypto/sha.h` includes.
- **Two fallbacks are pre-authorised** and must be recorded in the plan file if used: `esp_sha()` not linking with hardware SHA off (Task 1 Step 5), and `struct esp_tls` being opaque (Task 2 Step 6). Neither invalidates the design.
- **`<pw>` is the module's web admin password**, not a placeholder to invent. Ask the user for
  it, or read it from an existing local config; never commit it. The harness takes it via the
  `OVMS_PW` environment variable so it stays out of the shell history file and out of git.
- **Endpoint parameter names are verified against the source**, not guessed: `/api/execute`
  reads `command` (`components/ovms_webserver/src/web_cfg.cpp:46`), `/login` reads `username`
  and `password` (`components/ovms_webserver/src/ovms_webserver.cpp:1022-1025`).
- **The expected result is a small penalty.** M3 already established that every OVMS peer negotiates AES-GCM, so there is no per-record SHA in production and the cost is handshake-only. If the macro layer shows a large bulk delta, suspect the experiment before believing it — check that the ciphersuite pinning actually took effect.
