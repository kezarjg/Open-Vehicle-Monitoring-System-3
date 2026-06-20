# WiFi Priority Networks — Design

**Date:** 2026-06-05
**Component:** `vehicle/OVMS.V3/components/esp32wifi`
**Status:** Approved design (pre-implementation)

## Problem

OVMS supports multiple known WiFi networks via the `wifi.ssid` config map, and a
"scanning mode" (empty `auto wifi.ssid.client`) that connects to a known network found
in a scan. But it has two gaps for the common "home WiFi, else phone hotspot" use case:

1. **No priority ordering.** When several known networks are visible, scanning mode
   connects to the *first known SSID in scan-result order*
   (`esp32wifi.cpp:1392-1404`), not a user-defined preference. There is no way to say
   "prefer home over hotspot."
2. **No scan-while-connected upgrade.** Once associated, the firmware never rescans to
   find a more-preferred network: both the reconnect trigger (`esp32wifi.cpp:1318`) and
   the scan-done connect logic (`esp32wifi.cpp:1387`) are gated on `!m_sta_connected`.
   If the module joins the hotspot first and then comes into home range, it stays on the
   hotspot until that connection drops.

The ESP32 radio *can* scan while connected (STA/APSTA modes already do); the limitation
is purely in OVMS firmware logic.

### No upstream feature to model on

OVMS builds against the `openvehicles/esp-idf` fork (ESP-IDF v3.3.x; confirmed in
`.travis.yml` and `.devcontainer/Dockerfile`). The native `esp_wifi` driver holds a
single STA config at a time and has **no** multi-SSID credential store or cross-SSID
preference selection. `sort_method = WIFI_CONNECT_AP_BY_SIGNAL` (already used at
`esp32wifi.cpp:1441`) only ranks candidates *within the one configured SSID* by signal;
`wifi_scan_threshold_t` only filters weak APs for a single SSID. Cross-SSID priority and
"switch to a preferred network when it appears" do not exist upstream; 802.11k/v/r
roaming arrived in IDF v5.x and is same-ESSID roaming regardless. OVMS's existing
scanning-mode loop is therefore the project's own stand-in for the missing capability,
and this feature **extends that local mechanism** rather than adopting anything upstream.

> NOTE (verify during implementation): the IDF SDK is not checked out in the dev
> environment used to author this spec (the build runs on CI), so the `sort_method`
> single-SSID semantics above are from ESP-IDF knowledge, not a local header read.
> Reconfirm against `esp_wifi_types.h` if any behavior depends on it.

## Goals

- Let the user rank known networks; always prefer the highest-ranked usable one.
- While connected to a non-top network, periodically scan and **upgrade** to a
  higher-priority network when it becomes available and usable.
- Apply priority at **initial connect** too, not just upgrades.
- Default off; zero behavior change when disabled.

## Non-goals

- Roaming between APs of the *same* SSID (left to existing `sort_method`).
- Cellular/modem failover interaction.
- Changing `ap2client` timeout behavior.
- Priority in fixed mode (a pinned `auto wifi.ssid.client`) — see Scope.

## Behavioral decisions (from brainstorming)

| Decision | Choice |
|----------|--------|
| Switch trigger | **Always upgrade** — switch to a higher-priority network whenever visible & usable, even if the current link is fine (brief disconnect accepted). |
| Priority expression | **Ordered list** of SSIDs, highest first. |
| Flap guard | **Minimum RSSI gate** — only switch if the candidate's RSSI ≥ existing "good" threshold (`network wifi.sq.good`). Scan interval gives implicit dwell. |
| Scan cadence | **Only when on a non-top network.** On the top-priority network, no background scans at all. Otherwise scan every `interval` seconds. |
| Activation | **Explicit enable flag** + non-empty list + scanning mode + CLIENT/APCLIENT. |

## Configuration

All keys under the existing `network` config param (consistent with `wifi.sq.good`,
`wifi.bad.reconnect`, etc.).

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `network wifi.priority.enable` | bool | `no` | Master on/off for the feature. |
| `network wifi.priority` | string | (empty) | Comma-separated SSIDs, **highest priority first**, e.g. `home,hotspot,cafe`. |
| `network wifi.priority.interval` | int (s) | `60` | Upgrade-scan period while on a non-top network. Clamped to a minimum of 10s. |

No new RSSI knob: the switch gate reuses `network wifi.sq.good` (−87 dBm default).

### Priority semantics

- A network's rank = its index in the parsed list (index 0 = top). Networks **not** in
  the list rank below all listed ones (`INT_MAX`).
- A network is only *connectable* if it has a password entry in the existing `wifi.ssid`
  map. The priority list only **orders** among known networks; a listed SSID with no
  `wifi.ssid` credential is skipped.
- Leading/trailing whitespace around list entries is trimmed; empty entries are ignored.
- Duplicate SSIDs in the list: first occurrence wins (lowest index).

### Activation predicate

The feature is **active** only when ALL hold:

- `network wifi.priority.enable` == true, AND
- `network wifi.priority` parses to ≥1 entry, AND
- mode is `ESP32WIFI_MODE_CLIENT` or `ESP32WIFI_MODE_APCLIENT`, AND
- scanning mode — `m_sta_ssid` is empty (i.e. `auto wifi.ssid.client` unset).

If any fails, all existing behavior is unchanged (legacy "first known in scan order"
selection, no upgrade scans).

## Scope

- **CLIENT and APCLIENT** only. In APCLIENT, only the STA side is affected; the AP side
  is never reconfigured or torn down.
- **Fixed mode disables the feature** by design — pinning a client SSID is an explicit
  "use exactly this network" instruction; priority requires scanning mode.
- No interaction with cellular failover. `ap2client` logic untouched.

## Design (Approach A — extend the existing scan/connect path)

All logic stays in `components/esp32wifi/src/esp32wifi.cpp`, which conventionally owns
every WiFi behavior in one class guarded by `m_mutex`. New decision logic is isolated in
named private helpers rather than tangled inline.

### New private helpers

```cpp
// Index of ssid in the parsed priority list (0 = top); INT_MAX if unlisted.
int  GetNetworkPriority(const char* ssid);

// Best AP in the scan list that is (a) in the priority list AND (b) has a wifi.ssid
// password, subject to priority < betterThan and rssi >= rssiFloor. Tie-break by RSSI.
// Returns scan-list index, or -1 if none qualify.
int  SelectPriorityAP(wifi_ap_record_t* list, int count, int betterThan, int rssiFloor);

// True when the connected SSID's priority index is 0 (nothing can rank higher).
bool CurrentIsTopPriority();
```

### New member state

- `bool m_sta_upgrade_scan` — tags an in-flight scan as an upgrade probe.
- `bool m_sta_switching` — guards a deliberate disconnect/reconnect during a switch.
- `uint32_t m_upgrade_scan_next` — monotonic time of the next allowed upgrade scan.
- Parsed priority list cached as a member (e.g. `std::vector<std::string>
  m_priority_list`), re-parsed on config change via the existing config-event hook
  (`ConfigChanged` / the `config.changed` handler that already reads `network wifi.*`).

### Call site (a): initial connect — fix priority ordering

In `EventWifiScanDone`, existing `m_mode != AP && !m_sta_connected` branch
(`esp32wifi.cpp:1387-1416`):

- **Feature active:** select via
  `SelectPriorityAP(list, count, /*betterThan=*/INT_MAX, /*rssiFloor=*/INT_MIN)` — best
  priority among all visible known networks, **no RSSI gate** (disconnected: any usable
  known network beats nothing).
- **Feature inactive:** the existing first-known loop runs unchanged.

Priority is thus honored at first connect, not only on upgrades.

### Call site (b): upgrade scan while connected

**Scheduling** — in `EventTimer1` (`ticker.1`, per second, `esp32wifi.cpp:1300+`), add:

```
if feature active
   && m_sta_connected
   && !CurrentIsTopPriority()
   && monotonictime >= m_upgrade_scan_next
   && no user scan in progress (m_mode != MODE_SCAN):
       start scan with m_sta_upgrade_scan = true
       m_upgrade_scan_next = monotonictime + max(10, interval)
```

On the top-priority network (or feature off), no upgrade scans occur — zero link cost.
The scan uses `esp_wifi_scan_start(&conf, false)` (non-blocking) and does **not** call
`StartConnect()` (which is for the disconnected path and resets `m_sta_reconnect`).

**Evaluation** — in `EventWifiScanDone`, new `m_sta_connected && m_sta_upgrade_scan`
branch:

```
candidate = SelectPriorityAP(list, count,
              /*betterThan=*/GetNetworkPriority(current_ssid),
              /*rssiFloor=*/ (int)GetParamValueFloat("network","wifi.sq.good"))
if candidate >= 0: perform switch (below)
always: m_sta_upgrade_scan = false
```

**Units:** `SelectPriorityAP` compares its `rssiFloor` against `wifi_ap_record_t.rssi`
from the scan results, which is **plain dBm** — so the floor is passed as plain dBm
(e.g. −87), NOT scaled. The ×10 scaling applies only to the stored smoothed metric
`m_sta_rssi`/`ms_m_net_wifi_sq`, which is not used in this comparison.

### The switch sequence (the fiddly part)

Reuse the existing connect block (`esp32wifi.cpp:1424-1463`: populate `m_wifi_sta_cfg`
with ssid/password/bssid/channel, `WIFI_FAST_SCAN`, `WIFI_CONNECT_AP_BY_SIGNAL`), but
because we are already associated we must guard the deliberate teardown:

```
OvmsRecMutexLock exclusive(&m_mutex);
m_sta_switching = true;
esp_wifi_disconnect();
esp_wifi_set_config(WIFI_IF_STA, &m_wifi_sta_cfg);
esp_wifi_connect();
// IP re-acquired by existing logic on the STA_GOTIP/connected event:
//   static -> SetSTAWifiIP(); else StartDhcpClient();
```

- The `m_sta_switching` guard makes the resulting `STA_DISCONNECTED` event **not** trip
  the normal "reconnect to anything" path mid-switch. It is cleared once the new
  association resolves (on `STA_CONNECTED`/`GOTIP`) or on switch failure.
- If the new association fails, fall through to the standard reconnect timer
  (`m_sta_reconnect = monotonictime + 10`) so the module is never stranded offline; the
  disconnected path will then re-pick the best available known network.

### `EventWifiScanDone` dispatch — three explicit cases

This function is the part of the codebase easiest to get subtly wrong, so each case is
gated explicitly (no fall-through):

1. **User scan** — `m_mode == ESP32WIFI_MODE_SCAN`: return early (handled by `Scan()`),
   as today.
2. **Reconnect scan** — `!m_sta_connected`: existing connect logic, priority-aware when
   the feature is active (call site a).
3. **Upgrade scan** — `m_sta_connected && m_sta_upgrade_scan`: evaluate & maybe switch
   (call site b). Clear `m_sta_upgrade_scan`.

## Error handling & edge cases

- **Empty/malformed priority list** → feature inactive (treated as disabled).
- **Listed SSID without `wifi.ssid` credential** → skipped by `SelectPriorityAP`.
- **Connected to an unlisted network while listed networks exist** → `CurrentIsTopPriority()`
  is false (`INT_MAX` rank), so upgrade scans run and can move to a listed network. Intended.
- **Scan failure during upgrade** → log at `E`/`V`, clear `m_sta_upgrade_scan`, retry next
  interval; current connection untouched.
- **Switch/connect failure** → `m_sta_switching` cleared, standard reconnect timer engaged
  (never left offline).
- **Interval clamp** → minimum 10s to avoid hammering the link.
- **Hidden SSIDs** → priority matches by SSID string; hidden APs (empty SSID in scan
  results) cannot be ranked and won't participate in priority upgrades. Documented limitation.
- **Concurrent user `wifi scan`** → upgrade scan suppressed that tick (`m_mode != MODE_SCAN`
  check) to avoid scan collision.
- **Config change mid-operation** → list re-parsed on the config-changed event; next tick
  uses the new list. No in-flight scan is interrupted.

## User-facing surface

- **`wifi status`** gains a short line when the feature is active: current priority rank
  and whether upgrade-scanning is in effect (e.g. `Priority: rank 1/3 (top) — not scanning`
  or `Priority: rank 2/3 — upgrade-scanning every 60s`).
- **Switch event** logged at `ESP_LOGI` (`ScanDone: upgrade to higher-priority ssid=...`).
  No user notification in v1 (YAGNI; can mirror `ap2client.notify` later if wanted).

## changes.txt entry (required)

New feature + new configs, so a `changes.txt` entry is required per project convention,
including a `New configs:` sub-block:

```
- WiFi: optional priority network list. When enabled, the module prefers known networks
  in a configured order and, while connected to a lower-priority network, periodically
  scans and upgrades to a higher-priority one once it is in range with good signal.
  New configs:
    [network] wifi.priority.enable   = yes|no (default no)
    [network] wifi.priority          = comma-separated SSIDs, highest first
    [network] wifi.priority.interval = upgrade-scan seconds while on non-top net (default 60)
```

Docs: extend `docs/source/userguide/wifi/client.rst` with a priority-networks subsection.

## Testing

No host unit-test suite (per CLAUDE.md); build verification is GitHub CI. Validation is
on-device via the `test`/`wifi` commands plus bench/vehicle checks:

1. **Initial connect priority** — home + hotspot both present, feature on → connects to
   home (rank 0), not scan order.
2. **Upgrade** — on hotspot, bring home into range above `wifi.sq.good` → switches to home
   within `interval`.
3. **No scanning on top** — on home (rank 0) → logs show no background scans, no link blips.
4. **RSSI gate** — home visible but below `wifi.sq.good` → no switch; stays on hotspot.
5. **Disable** — `wifi.priority.enable = no` → legacy first-known behavior, no upgrade scans.
6. **Fixed mode** — `auto wifi.ssid.client` set → feature inert.
7. **APCLIENT** — switch on STA side does not drop or reconfigure the AP side.
8. **Switch failure** — target AP credentials wrong → falls back to reconnect, not stranded.

## Files touched

- `components/esp32wifi/src/esp32wifi.cpp` — helpers, `EventTimer1`, `EventWifiScanDone`,
  switch logic, config-change parse, `wifi status` line.
- `components/esp32wifi/src/esp32wifi.h` — new members and helper declarations.
- `components/esp32wifi/src/esp32wifi.cpp` config registration — register the three new
  `network wifi.priority*` params with defaults.
- `vehicle/OVMS.V3/changes.txt` — feature + new configs entry.
- `docs/source/userguide/wifi/client.rst` — priority-networks documentation.
