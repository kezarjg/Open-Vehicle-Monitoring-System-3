# WiFi Priority Networks Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let OVMS prefer known WiFi networks in a user-defined priority order, and while connected to a lower-priority network, periodically scan and upgrade to a higher-priority one once it is in range with good signal.

**Architecture:** Extend the existing OVMS "scanning mode" selection in `components/esp32wifi`. All logic stays in `esp32wifi.cpp` (which owns every WiFi behavior under `m_mutex`), with new decision logic isolated in named private helpers. Adds priority-aware selection at initial connect, plus a new "upgrade scan while connected" path driven from the per-second `EventTimer1` and resolved in `EventWifiScanDone`.

**Tech Stack:** C++ (ESP-IDF 3.3 fork `openvehicles/esp-idf`, legacy `make`), OVMS3 framework (`MyConfig`, `MyEvents`, `OvmsRecMutex`), native `esp_wifi` driver.

**Testing reality (read first):** There is **no host unit-test suite** for this firmware (per CLAUDE.md). "Build verification" = the GitHub Actions CI build (push/PR), not a local build. Behavioral verification is **on-device** via the `wifi`/`config` shell commands. Tasks below therefore use *compile-on-CI* as the per-task gate and concentrate behavioral validation in a final on-device test matrix (Task 8). This is a deliberate deviation from the skill's host-test TDD loop, forced by the platform.

**Worktree:** Work happens in the existing worktree `~/wt-wifi-priority` on branch `feature/wifi-priority-networks`. All paths below are relative to that worktree root.

**Style:** Match the surrounding file exactly — 2-space indent, opening brace on its own line, `ESP_LOGx(TAG, ...)`, `MyConfig.GetParamValue*`. Do not reformat existing code.

---

## File Structure

| File | Responsibility | Change |
|------|----------------|--------|
| `vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.h` | Class decl: new members + helper prototypes | Modify |
| `vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp` | All logic: config parse, helpers, scheduling, switch, status | Modify |
| `vehicle/OVMS.V3/changes.txt` | User-facing changelog + new configs | Modify |
| `docs/source/userguide/wifi/client.rst` | User documentation | Modify |

No new files: the component conventionally keeps all WiFi logic in the one translation unit.

---

## Task 1: Config surface, state members, and list parsing

Registers the three new `network wifi.priority*` settings, adds the member state, and parses the priority list (with trimming + min-interval clamp) in the existing `ConfigChanged` handler.

**Files:**
- Modify: `vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.h:34` (includes), `:107` (helper protos), `:133-146` (members)
- Modify: `vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp:405-432` (constructor init), `:1542-1554` (ConfigChanged)

- [ ] **Step 1: Add includes and member/prototype declarations to the header**

In `esp32wifi.h`, add `<vector>` and `<limits.h>` near the existing `<string>` include (line 34):

```cpp
#include <string>
#include <vector>
#include <limits.h>
#include <stdint.h>
```

In the `public:` event/helper block (after `void ConfigChanged(...)` at line 107), add the new helper prototypes:

```cpp
    void ConfigChanged(std::string event, void *data);

    // --- Priority networks (see docs/superpowers/specs/2026-06-05-wifi-priority-networks-design.md) ---
    void ParsePriorityList(std::string csv);   // fills m_priority_list
    bool PriorityActive();                      // feature enabled + list + scanning mode + CLIENT/APCLIENT
    int  GetNetworkPriority(const char* ssid);  // index in list (0=top); INT_MAX if unlisted
    int  SelectPriorityAP(wifi_ap_record_t* list, int count, int betterThan, int rssiFloor);
    bool CurrentIsTopPriority();                // connected SSID has priority index 0
    void StartUpgradeScan();                    // non-blocking scan tagged as upgrade probe
```

In the `protected:` members block (after `m_ap2client_active;` at line 145), add:

```cpp
    bool m_ap2client_active;                     //!< Wifi Mode APClient enabled and timeout not reached
    // --- Priority networks ---
    bool m_priority_enable;                      //!< network wifi.priority.enable
    int  m_priority_interval;                    //!< upgrade-scan period [s], min 10
    std::vector<std::string> m_priority_list;    //!< SSIDs, highest priority first (index 0 = top)
    bool m_sta_upgrade_scan;                     //!< current in-flight scan is an upgrade probe
    bool m_sta_switching;                        //!< deliberate switch in progress; suppress auto-reconnect
    uint32_t m_upgrade_scan_next;                //!< monotonic time of next allowed upgrade scan
    uint32_t m_sta_switch_deadline;              //!< failsafe: clear m_sta_switching after this time
```

- [ ] **Step 2: Initialize new members in the constructor**

In `esp32wifi.cpp`, find the constructor member init around line 417 (`m_sta_reconnect = 0;`) and add immediately after it:

```cpp
  m_sta_reconnect = 0;
  m_priority_enable = false;
  m_priority_interval = 60;
  m_sta_upgrade_scan = false;
  m_sta_switching = false;
  m_upgrade_scan_next = 0;
  m_sta_switch_deadline = 0;
```

- [ ] **Step 3: Register config param defaults**

In the constructor, after the existing `MyConfig.RegisterParam(...)` lines (429-430), the `network` param is already registered elsewhere (it is used for `wifi.sq.good` etc.), so no new RegisterParam is needed. Instead set instance defaults next to the other `network` defaults. Add a `SetParamValueIf...`-free default by relying on `GetParamValue*` defaults in `ConfigChanged` (Step 4) — no registration code change required. (Confirm `network` param exists: it is read at `esp32wifi.cpp:1548`.) No code change in this step; it documents that defaults live in `ConfigChanged`.

- [ ] **Step 4: Implement ParsePriorityList and parse config in ConfigChanged**

Add the parser as a new function (place it directly above `ConfigChanged` at line 1542):

```cpp
void esp32wifi::ParsePriorityList(std::string csv)
  {
  m_priority_list.clear();
  size_t start = 0;
  while (start <= csv.length())
    {
    size_t comma = csv.find(',', start);
    if (comma == std::string::npos) comma = csv.length();
    std::string item = csv.substr(start, comma - start);
    // trim leading/trailing whitespace:
    size_t a = item.find_first_not_of(" \t");
    size_t b = item.find_last_not_of(" \t");
    if (a != std::string::npos)
      {
      item = item.substr(a, b - a + 1);
      // first occurrence wins (skip duplicates):
      bool dup = false;
      for (size_t i = 0; i < m_priority_list.size(); i++)
        if (m_priority_list[i] == item) { dup = true; break; }
      if (!dup)
        m_priority_list.push_back(item);
      }
    start = comma + 1;
    }
  }
```

Extend `ConfigChanged` (inside the existing `if (event == "config.mounted" || !param || param->GetName() == "network")` block, after the `m_ap2client_active` line at 1552):

```cpp
    m_ap2client_active = m_ap2client_enabled;                                                              // Mirror enabled to active
    m_priority_enable = MyConfig.GetParamValueBool("network", "wifi.priority.enable", false);
    m_priority_interval = MyConfig.GetParamValueInt("network", "wifi.priority.interval", 60);
    if (m_priority_interval < 10) m_priority_interval = 10;                                                // clamp: avoid hammering
    ParsePriorityList(MyConfig.GetParamValue("network", "wifi.priority", ""));
```

- [ ] **Step 5: Verify it compiles on CI**

Commit and push (Step 6); confirm the GitHub Actions build for the branch is green. Locally there is no build. Expected: CI "build" job passes (new members compile; `<vector>`/`<limits.h>` resolve).

- [ ] **Step 6: Commit**

```bash
cd ~/wt-wifi-priority
git add vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.h vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp
git commit -m "esp32wifi: add wifi.priority config, state, and list parsing"
```

---

## Task 2: Priority ranking helpers

Implements the three pure decision helpers plus `PriorityActive()`. These are self-contained and used by Tasks 3–5.

**Files:**
- Modify: `vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp` (add functions above `EventWifiScanDone` at line 1353)

- [ ] **Step 1: Implement PriorityActive, GetNetworkPriority, SelectPriorityAP, CurrentIsTopPriority**

Insert these four functions immediately before `void esp32wifi::EventWifiScanDone(...)` (line 1353):

```cpp
bool esp32wifi::PriorityActive()
  {
  // Feature runs only in scanning mode (m_sta_ssid empty) on CLIENT/APCLIENT:
  return m_priority_enable
      && !m_priority_list.empty()
      && (m_mode == ESP32WIFI_MODE_CLIENT || m_mode == ESP32WIFI_MODE_APCLIENT)
      && m_sta_ssid.empty();
  }

int esp32wifi::GetNetworkPriority(const char* ssid)
  {
  if (ssid == NULL || ssid[0] == 0)
    return INT_MAX;
  for (size_t i = 0; i < m_priority_list.size(); i++)
    if (m_priority_list[i] == ssid)
      return (int)i;
  return INT_MAX;
  }

// Best AP in the scan list that is (a) in the priority list AND (b) has a wifi.ssid
// password, subject to priority < betterThan and rssi >= rssiFloor.
// Tie-break by RSSI (rssi is plain dBm from wifi_ap_record_t). Returns index or -1.
int esp32wifi::SelectPriorityAP(wifi_ap_record_t* list, int count, int betterThan, int rssiFloor)
  {
  int best = -1;
  int best_prio = INT_MAX;
  int best_rssi = INT_MIN;
  for (int k = 0; k < count; k++)
    {
    int prio = GetNetworkPriority((const char*)list[k].ssid);
    if (prio == INT_MAX || prio >= betterThan)
      continue;
    if (list[k].rssi < rssiFloor)
      continue;
    if (MyConfig.GetParamValue("wifi.ssid", (const char*)list[k].ssid).empty())
      continue;                                  // no stored credential -> not connectable
    if (prio < best_prio || (prio == best_prio && list[k].rssi > best_rssi))
      {
      best = k;
      best_prio = prio;
      best_rssi = list[k].rssi;
      }
    }
  return best;
  }

bool esp32wifi::CurrentIsTopPriority()
  {
  return GetNetworkPriority((const char*)m_wifi_sta_cfg.sta.ssid) == 0;
  }
```

- [ ] **Step 2: Verify it compiles on CI**

Commit/push (Step 3); confirm CI build green. Expected: PASS (helpers reference only existing members and `MyConfig`).

- [ ] **Step 3: Commit**

```bash
cd ~/wt-wifi-priority
git add vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp
git commit -m "esp32wifi: add priority ranking helpers"
```

---

## Task 3: Priority-aware initial connect

Makes the existing `!m_sta_connected` scan-done branch honor priority order when the feature is active. Legacy "first known in scan order" remains the behavior when inactive.

**Files:**
- Modify: `vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp:1387-1416` (the `if (m_mode != ESP32WIFI_MODE_AP && !m_sta_connected)` selection loop)

- [ ] **Step 1: Replace the selection inside the disconnected branch**

The current code (lines 1389-1415) computes `ap_connect` via a first-match loop. Wrap it so that when `PriorityActive()` is true, selection uses the helper instead. Replace the block that starts at `int ap_connect = -1;` (line 1389) and ends just before `// connect:` (line 1417) with:

```cpp
    int ap_connect = -1;

    if (PriorityActive())
      {
      // priority mode: best-ranked known network among all visible, no RSSI gate
      // (disconnected: any usable known network beats nothing)
      ap_connect = SelectPriorityAP(list, apCount, /*betterThan=*/INT_MAX, /*rssiFloor=*/INT_MIN);
      if (ap_connect >= 0)
        password = MyConfig.GetParamValue("wifi.ssid", (const char*)list[ap_connect].ssid);
      }
    else
      {
      // check scan results for usable networks (legacy: first known in scan order):
      for (int k=0; k<apCount; k++)
        {
        ESP_LOGV(TAG, "ScanDone: #%02d ssid='%s' bssid='" MACSTR "' chan=%d rssi=%d",
          k+1, (const char*)list[k].ssid, MAC2STR(list[k].bssid), list[k].primary, list[k].rssi);
        if (ap_connect >= 0)
          continue;
        if (m_sta_ssid.empty())
          {
          // scanning mode:
          password = MyConfig.GetParamValue("wifi.ssid", (const char*)list[k].ssid);
          if (!password.empty())
            ap_connect = k;
          }
        else
          {
          // fixed mode:
          if ((m_sta_bssid_set && memcmp(m_sta_bssid, list[k].bssid, 6) == 0)
              || (!m_sta_bssid_set && m_sta_ssid == (const char*)list[k].ssid))
            {
            password = m_sta_password;
            ap_connect = k;
            }
          }
        }
      }
```

> Note: `password` is the existing `std::string password;` declared at the top of `EventWifiScanDone` (line 1358). The connect block that follows (`if (ap_connect < 0) ... else { ... }`, lines 1417-1466) is unchanged and consumes `ap_connect` + `password` exactly as before.

- [ ] **Step 2: Verify it compiles on CI**

Commit/push (Step 3); confirm CI build green. Expected: PASS.

- [ ] **Step 3: Commit**

```bash
cd ~/wt-wifi-priority
git add vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp
git commit -m "esp32wifi: honor priority order at initial connect"
```

---

## Task 4: Upgrade-scan scheduling

Adds the periodic background scan while connected to a non-top network, and the `StartUpgradeScan()` helper that tags the scan without disturbing the reconnect timer.

**Files:**
- Modify: `vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp` (add `StartUpgradeScan` near `StartConnect` at line 1325; add scheduling block in `EventTimer1` at lines 1316-1322)

- [ ] **Step 1: Implement StartUpgradeScan**

Insert directly after `esp32wifi::StartConnect()` (after its closing brace at line 1351):

```cpp
void esp32wifi::StartUpgradeScan()
  {
  // Background scan while still connected, to look for a higher-priority network.
  // Unlike StartConnect(), this does NOT touch m_sta_reconnect and tags the scan
  // so EventWifiScanDone routes it to the upgrade-evaluation branch.
  OvmsRecMutexLock exclusive(&m_mutex);
  esp_wifi_scan_stop();

  wifi_scan_config_t scanConf;
  memset(&scanConf,0,sizeof(scanConf));
  scanConf.ssid = NULL;
  scanConf.bssid = NULL;
  scanConf.channel = 0;
  scanConf.show_hidden = true;
  scanConf.scan_type = WIFI_SCAN_TYPE_ACTIVE;
  scanConf.scan_time.active = GetScanTime();
  m_sta_upgrade_scan = true;
  esp_err_t res = esp_wifi_scan_start(&scanConf, false);
  if (res != ESP_OK)
    {
    m_sta_upgrade_scan = false;
    ESP_LOGE(TAG, "StartUpgradeScan: error 0x%x starting scan", res);
    }
  else
    ESP_LOGV(TAG, "StartUpgradeScan: scan started");
  }
```

- [ ] **Step 2: Add the scheduling block + switch-guard failsafe in EventTimer1**

In `EventTimer1`, the existing reconnect block is at lines 1317-1322:

```cpp
  // reconnect?
  if ((m_mode == ESP32WIFI_MODE_CLIENT || m_mode == ESP32WIFI_MODE_APCLIENT)
      && !m_sta_connected && m_sta_reconnect && monotonictime >= m_sta_reconnect)
    {
    StartConnect();
    }
  }
```

Replace it with (adds the switch guard `!m_sta_switching`, the switch-failsafe, and the upgrade-scan scheduler):

```cpp
  // switch-in-progress failsafe: clear the guard if a deliberate switch stalls
  if (m_sta_switching && monotonictime >= m_sta_switch_deadline)
    {
    ESP_LOGD(TAG, "priority switch timed out; clearing guard");
    m_sta_switching = false;
    }

  // reconnect? (suppressed while a deliberate switch is in progress)
  if ((m_mode == ESP32WIFI_MODE_CLIENT || m_mode == ESP32WIFI_MODE_APCLIENT)
      && !m_sta_connected && !m_sta_switching && m_sta_reconnect && monotonictime >= m_sta_reconnect)
    {
    StartConnect();
    }

  // priority upgrade scan? (connected to a non-top network, feature active)
  if (PriorityActive() && m_sta_connected && !m_sta_switching
      && m_mode != ESP32WIFI_MODE_SCAN
      && !CurrentIsTopPriority()
      && monotonictime >= m_upgrade_scan_next)
    {
    m_upgrade_scan_next = monotonictime + m_priority_interval;
    StartUpgradeScan();
    }
  }
```

- [ ] **Step 3: Verify it compiles on CI**

Commit/push (Step 4); confirm CI build green. Expected: PASS.

- [ ] **Step 4: Commit**

```bash
cd ~/wt-wifi-priority
git add vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp
git commit -m "esp32wifi: schedule periodic upgrade scan on non-top network"
```

---

## Task 5: Upgrade evaluation and the switch sequence

Adds the third dispatch case to `EventWifiScanDone` (connected + upgrade scan) and performs the guarded disconnect→reconnect switch. Clears the switch guard on successful association.

**Files:**
- Modify: `vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp:1387` (add connected branch), `:1131-1150` (`EventWifiStaConnected` clears guard)

- [ ] **Step 1: Add the upgrade-evaluation branch in EventWifiScanDone**

`EventWifiScanDone` currently has the early return for `MODE_SCAN` (line 1360) and the `if (m_mode != ESP32WIFI_MODE_AP && !m_sta_connected)` block (line 1387). Add a new branch for the connected upgrade-scan case. Immediately **after** the closing brace of the `!m_sta_connected` block (the `}` that precedes `if (list)` at line 1467), insert:

```cpp
  else if (m_sta_connected && m_sta_upgrade_scan && PriorityActive())
    {
    int rssi_floor = (int)m_good_dbm;            // wifi.sq.good, plain dBm (matches list[k].rssi)
    int cur_prio = GetNetworkPriority((const char*)m_wifi_sta_cfg.sta.ssid);
    int k = SelectPriorityAP(list, apCount, /*betterThan=*/cur_prio, /*rssiFloor=*/rssi_floor);
    if (k >= 0)
      {
      std::string ssid = (const char*)list[k].ssid;
      std::string password = MyConfig.GetParamValue("wifi.ssid", ssid);
      ESP_LOGI(TAG, "ScanDone: upgrade to higher-priority ssid='%s' bssid='" MACSTR "' chan=%d rssi=%d",
        ssid.c_str(), MAC2STR(list[k].bssid), list[k].primary, list[k].rssi);

      OvmsRecMutexLock exclusive(&m_mutex);
      // Guard the deliberate teardown so EventTimer1's reconnect block does not race:
      m_sta_switching = true;
      m_sta_switch_deadline = monotonictime + 15;
      m_sta_reconnect = monotonictime + 15;      // armed fallback if the switch fails

      esp_wifi_disconnect();

      memset(&m_wifi_sta_cfg,0,sizeof(m_wifi_sta_cfg));
      strcpy((char*)m_wifi_sta_cfg.sta.ssid, ssid.c_str());
      strcpy((char*)m_wifi_sta_cfg.sta.password, password.c_str());
      m_wifi_sta_cfg.sta.bssid_set = true;
      memcpy(m_wifi_sta_cfg.sta.bssid, list[k].bssid, sizeof(m_wifi_sta_cfg.sta.bssid));
      m_wifi_sta_cfg.sta.channel = list[k].primary;
      m_wifi_sta_cfg.sta.scan_method = WIFI_FAST_SCAN;
      m_wifi_sta_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;

      esp_err_t res = esp_wifi_set_config(WIFI_IF_STA, &m_wifi_sta_cfg);
      if (res != ESP_OK)
        {
        ESP_LOGE(TAG, "ScanDone(upgrade): esp_wifi_set_config error %d", res - ESP_ERR_WIFI_BASE);
        m_sta_switching = false;                 // let normal reconnect recover
        }
      else
        {
        res = esp_wifi_connect();
        if (res != ESP_OK)
          {
          ESP_LOGE(TAG, "ScanDone(upgrade): esp_wifi_connect error %d", res - ESP_ERR_WIFI_BASE);
          m_sta_switching = false;
          }
        else
          {
          std::string ipconfig = MyConfig.GetParamValue("wifi.ssid", ssid + ".ovms.staticip");
          if (!ipconfig.empty())
            SetSTAWifiIP();
          else
            StartDhcpClient();
          }
        }
      }
    m_sta_upgrade_scan = false;
    }
```

Also ensure `m_sta_upgrade_scan` is cleared if the scan produced no usable list: in the early-out paths of `EventWifiScanDone` (the `apCount == 0` return at line 1371 and the malloc-failure / `esp_wifi_scan_get_ap_records` error returns at lines 1378, 1384), set `m_sta_upgrade_scan = false;` before each `return;`. Concretely, change each of those three `return;` statements to:

```cpp
    m_sta_upgrade_scan = false;
    return;
```

- [ ] **Step 2: Clear the switch guard on successful association**

In `EventWifiStaConnected` (line 1131), after `m_sta_connected = true;` (line 1139), add:

```cpp
  m_sta_connected = true;
  m_sta_switching = false;                       // association resolved (incl. completed priority switch)
  m_previous_reason = 0;
```

- [ ] **Step 3: Verify it compiles on CI**

Commit/push (Step 4); confirm CI build green. Expected: PASS. Pay attention to the CI log for the new `else if` correctly chaining off the existing `if (m_mode != ESP32WIFI_MODE_AP && !m_sta_connected)` block.

- [ ] **Step 4: Commit**

```bash
cd ~/wt-wifi-priority
git add vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp
git commit -m "esp32wifi: evaluate and switch to higher-priority network while connected"
```

---

## Task 6: `wifi status` priority line

Surfaces the feature state in `wifi status` so a user can see the current rank and whether upgrade-scanning is active.

**Files:**
- Modify: `vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp:1490-1496` (STA block in `OutputStatus`)

- [ ] **Step 1: Add the priority status line**

In `OutputStatus`, inside the `if (m_mode == ESP32WIFI_MODE_CLIENT || m_mode == ESP32WIFI_MODE_APCLIENT)` block, after the existing `writer->printf("\nSTA SSID: ...")` call (ends line 1495), add:

```cpp
    if (PriorityActive())
      {
      int cur = GetNetworkPriority((const char*)m_wifi_sta_cfg.sta.ssid);
      if (cur == INT_MAX)
        writer->printf("  Priority: unlisted/%d — upgrade-scanning every %ds\n",
          (int)m_priority_list.size(), m_priority_interval);
      else if (cur == 0)
        writer->printf("  Priority: rank 1/%d (top) — not scanning\n",
          (int)m_priority_list.size());
      else
        writer->printf("  Priority: rank %d/%d — upgrade-scanning every %ds\n",
          cur+1, (int)m_priority_list.size(), m_priority_interval);
      }
```

> Use a plain ASCII `-` instead of the em-dash if the file's encoding/compiler warns; the surrounding file is ASCII. Prefer: replace `—` with `-`.

Final form to write (ASCII, to match the file):

```cpp
    if (PriorityActive())
      {
      int cur = GetNetworkPriority((const char*)m_wifi_sta_cfg.sta.ssid);
      if (cur == INT_MAX)
        writer->printf("  Priority: unlisted/%d - upgrade-scanning every %ds\n",
          (int)m_priority_list.size(), m_priority_interval);
      else if (cur == 0)
        writer->printf("  Priority: rank 1/%d (top) - not scanning\n",
          (int)m_priority_list.size());
      else
        writer->printf("  Priority: rank %d/%d - upgrade-scanning every %ds\n",
          cur+1, (int)m_priority_list.size(), m_priority_interval);
      }
```

- [ ] **Step 2: Verify it compiles on CI**

Commit/push (Step 3); confirm CI build green. Expected: PASS.

- [ ] **Step 3: Commit**

```bash
cd ~/wt-wifi-priority
git add vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp
git commit -m "esp32wifi: show priority rank in wifi status"
```

---

## Task 7: Documentation and changelog

User-facing docs and the required `changes.txt` entry with the `New configs:` sub-block.

**Files:**
- Modify: `vehicle/OVMS.V3/changes.txt` (top of file)
- Modify: `docs/source/userguide/wifi/client.rst`

- [ ] **Step 1: Add the changes.txt entry**

Open `vehicle/OVMS.V3/changes.txt` and add a new dated entry at the top following the existing format (date/author header, `-` bullets, `New configs:` sub-block). Use today's date and the repo author convention:

```
2026-06-05 (Jerry Kezar)
- WiFi: optional priority network list. When enabled, the module prefers known WiFi
  networks in a configured order, and while connected to a lower-priority network it
  periodically scans and upgrades to a higher-priority one once that network is in range
  with good signal (>= network wifi.sq.good). Active only in scanning mode (no fixed
  client SSID) in client or apclient mode. Default off.
  New configs:
    [network] wifi.priority.enable   = yes|no   (default no)
    [network] wifi.priority          = comma-separated SSIDs, highest priority first
    [network] wifi.priority.interval = upgrade-scan period in seconds while on a
                                        non-top network (default 60, minimum 10)
```

> If the most recent existing entry already has today's date and the same author header, append these bullets under it rather than creating a duplicate header.

- [ ] **Step 2: Document in the WiFi client user guide**

In `docs/source/userguide/wifi/client.rst`, add a subsection (match the file's RST heading style — check an existing heading's underline character/length first). Insert after the existing multi-SSID scanning description:

```rst
Priority networks
-----------------

Normally, in scanning mode the module connects to the first known network it finds in a
scan. You can instead define an explicit preference order and have the module upgrade to
a more-preferred network when it becomes available.

To enable, set (Config &rarr; Network, or via the shell)::

    config set network wifi.priority.enable yes
    config set network wifi.priority "home,hotspot,cafe"

The value of ``wifi.priority`` is a comma-separated list of SSIDs, highest priority
first. Each listed SSID must also have its password stored under ``wifi.ssid`` as usual.

Behaviour:

- At connect time the module joins the highest-priority known network that is in range.
- While connected to a lower-priority network, the module scans every
  ``wifi.priority.interval`` seconds (default 60, minimum 10) and switches to a
  higher-priority network once it is in range with good signal (at or above
  ``network wifi.sq.good``). The switch causes a brief disconnect.
- While connected to the top-priority network, no background scanning is performed.

This feature is active only in *scanning* mode (no fixed client SSID configured) in
``client`` or ``apclient`` mode. Networks with hidden SSIDs cannot participate in the
priority ordering. Use ``wifi status`` to see the current priority rank.
```

> Adjust the `&rarr;` / arrow notation to whatever the rest of `client.rst` uses; if unsure, write "Config > Network". Verify the heading underline length matches the title text length (RST requires this).

- [ ] **Step 3: Verify docs build (optional) and CI green**

Docs build via `cd docs && make html` (needs `sphinx_rtd_theme`, `sphinx_mdinclude`) if available; otherwise rely on CI. Confirm the firmware CI build is still green (changes.txt/docs don't affect the build, but push anyway).

- [ ] **Step 4: Commit**

```bash
cd ~/wt-wifi-priority
git add vehicle/OVMS.V3/changes.txt docs/source/userguide/wifi/client.rst
git commit -m "docs: document wifi priority networks feature"
```

---

## Task 8: On-device validation matrix (manual)

No host tests exist; this is the real behavioral verification. Flash the branch build to a module (see memory: fast deploy via `ota flash http` from os-k3s; reach module via `solterra-ovms` SSH alias). Configure two known networks (e.g. `home`, `hotspot`).

- [ ] **Step 1: Build artifact** — confirm GitHub Actions produced an `ovms3-firmware` artifact for the branch HEAD commit.
- [ ] **Step 2: Flash** — install the branch build on the test module.
- [ ] **Step 3: Configure**

```
config set network wifi.priority.enable yes
config set network wifi.priority "home,hotspot"
config set wifi.ssid home <home-password>
config set wifi.ssid hotspot <hotspot-password>
config set auto wifi.ssid.client ""        # ensure scanning mode
```

- [ ] **Step 4: Run the test matrix** (record pass/fail + `wifi status` output for each):

1. **Initial connect priority** — both networks present, reboot/`wifi reconnect` → connects to `home` (rank 1), not scan order.
2. **Upgrade** — connect to `hotspot` only (home off), then bring `home` up within range/good signal → within `interval` (~60s) it switches to `home`. Watch log for `ScanDone: upgrade to higher-priority ssid='home'`.
3. **No scanning on top** — once on `home` (rank 1/top), `wifi status` shows "not scanning"; logs show no periodic `StartUpgradeScan` lines.
4. **RSSI gate** — make `home` visible but weak (below `wifi.sq.good`, e.g. move far / lower AP power) while on `hotspot` → no switch; stays on `hotspot`.
5. **Disable** — `config set network wifi.priority.enable no` → legacy first-known behavior; no upgrade scans in log.
6. **Fixed mode** — `config set auto wifi.ssid.client home` → `wifi status` shows "fixed"; no priority line / no upgrade scans.
7. **APCLIENT** — `wifi mode apclient` with both nets; trigger an upgrade and confirm AP side stays up (a phone stays connected to the OVMS AP across the STA switch).
8. **Switch failure** — temporarily set a wrong `wifi.ssid home` password, force an upgrade attempt → module falls back and reconnects to `hotspot` (not stranded offline); guard clears within ~15s.

- [ ] **Step 5: Record results** in the PR description; capture any anomalies for follow-up.

---

## Self-review notes (addressed)

- **Spec coverage:** config surface (T1), helpers (T2), initial-connect priority (T3), upgrade scheduling (T4), evaluation+switch+guard (T5), `wifi status` (T6), changes.txt + docs (T7), on-device matrix (T8). All spec sections mapped.
- **Units:** `SelectPriorityAP` rssiFloor compared against `wifi_ap_record_t.rssi` (plain dBm); call site passes `(int)m_good_dbm` (plain dBm), not the ×10 `m_sta_rssi` metric. Consistent.
- **Names:** `m_sta_upgrade_scan`, `m_sta_switching`, `m_upgrade_scan_next`, `m_sta_switch_deadline`, `m_priority_enable`, `m_priority_interval`, `m_priority_list`, `PriorityActive`, `GetNetworkPriority`, `SelectPriorityAP`, `CurrentIsTopPriority`, `ParsePriorityList`, `StartUpgradeScan` used identically across header (T1) and all call sites (T2–T6).
- **Guard lifecycle:** set in T5 switch; cleared on success in `EventWifiStaConnected` (T5 Step 2), on set_config/connect failure inline (T5 Step 1), and by failsafe deadline in `EventTimer1` (T4 Step 2). Reconnect block suppressed while set (T4 Step 2).
