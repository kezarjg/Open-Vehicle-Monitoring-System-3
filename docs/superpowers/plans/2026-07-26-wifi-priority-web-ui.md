# WiFi Priority Networks Web UI — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add priority-network management to the `/cfg/wifi` web page so the WiFi priority list can be viewed and edited from a browser, with every silent-failure condition surfaced.

**Architecture:** Two new static methods on `OvmsWebServer` (`OutputWifiPriority` / `UpdateWifiPriority`) render and parse an additive fieldset on `/cfg/wifi`. The existing `OutputWifiTable`/`UpdateWifiTable` helpers keep their logic untouched — they are shared with the AP networks table — and are edited only to HTML-escape their existing warning strings (Task 5, separate commit). Ordering is carried in a single hidden CSV field maintained by inline page JS. One small `esp32wifi::OutputStatus` change makes `wifi status` explain why priority is inactive.

**Tech Stack:** C++ (ESP-IDF 3.3, older GCC — no C++14/17 idioms), jQuery 2.x + Bootstrap 3 (already bundled), Sphinx RST for docs.

**Spec:** `docs/superpowers/specs/2026-07-26-wifi-priority-web-ui-design.md`
**Issue:** kezarjg#165 · **Branch:** `feature/wifi-priority-web-ui` · **Worktree:** `/home/devuser/wt-wifi-priority-web-ui`

## Global Constraints

- **No host test suite exists.** The firmware compiles only in GitHub Actions or the devcontainer. "Verify" therefore means: (a) CI `Firmware build` green, (b) on-device check on the bench module. Never claim a local build verified anything.
- **Upstream-bound.** This ships as one PR to openvehicles together with the PR #120 firmware feature. No new JavaScript dependencies. Match the surrounding file's existing style exactly.
- **All page JS is inline** in `web_cfg_wifi.cpp`. Nothing goes into `ovms.js`, so no `script.js`/`script.js.gz` regeneration and the `Web assets sync check` workflow stays untriggered.
- **Every interpolated value in a warning or error string MUST be HTML-escaped** using `c.encode_html(v)`, which is `static` and returns `std::string` — safe to use directly in `+` concatenation, unlike the `_attr`/`_html` macros whose `.c_str()` result must not be stored. Literal markup in these strings stays as it is; only interpolated values are escaped. This **overrides** the file's existing convention by explicit decision of the project owner (2026-07-26), and the three pre-existing unescaped lines in `UpdateWifiTable` are corrected as part of Task 5.
- `_attr(x)` and `_html(x)` are file-local macros already defined at `web_cfg_wifi.cpp:26-27`; both expand to `c.encode_html(x).c_str()` and require a `PageContext_t& c` named `c` in scope.
- `<vector>` and `<set>` are already pulled in via `ovms_webserver.h`; `endsWith()` comes from `main/ovms_utils.h` and is already used in this file. **No new `#include` lines are needed.**
- Config defaults, which drive the delete-on-default writes: `wifi.priority.enable` = false, `wifi.priority.interval` = 60, `wifi.priority` = "".

---

### Task 1: Make `wifi status` explain priority inactivity

Independent of all web work. Do this first — it is the smallest change and it is what makes the feature debuggable from the CLI and (because the web status page renders `wifi status` verbatim) from the browser.

**Files:**
- Modify: `vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp:1668-1680`

**Interfaces:**
- Consumes: existing members `m_priority_enable`, `m_priority_list`, `m_sta_ssid`, `m_priority_interval`, and `PriorityActive()` / `GetNetworkPriority()`.
- Produces: nothing consumed by later tasks.

**Context:** The existing `Priority:` output sits inside `if (PriorityActive())`, which is itself nested inside `if (m_mode == ESP32WIFI_MODE_CLIENT || m_mode == ESP32WIFI_MODE_APCLIENT)`. So when priority is enabled but inert, the line disappears entirely. Because we are already inside the CLIENT/APCLIENT test, the only reachable inactivity reasons here are *fixed SSID* and *empty list* — a wrong-mode module never reaches this block at all, and that case is covered by a web warning in Task 5.

- [ ] **Step 1: Add the else-branch**

In `esp32wifi::OutputStatus`, directly after the closing brace of the existing `if (PriorityActive())` block (currently line 1680), add:

```cpp
    else if (m_priority_enable)
      {
      // Enabled but not active: say why, otherwise the line simply vanishes exactly
      // when the user is trying to work out why priority is doing nothing.
      if (!m_sta_ssid.empty())
        writer->printf("  Priority: inactive - fixed SSID configured\n");
      else if (m_priority_list.empty())
        writer->printf("  Priority: inactive - network list is empty\n");
      }
```

- [ ] **Step 2: Verify style**

Run: `git diff` and confirm the new block uses the file's 2-space-indent-then-brace-on-own-line style, matching the `if (PriorityActive())` block immediately above it.

- [ ] **Step 3: Commit**

```bash
git add vehicle/OVMS.V3/components/esp32wifi/src/esp32wifi.cpp
git commit -m "esp32wifi: explain why priority networks are inactive in wifi status"
```

---

### Task 2: Fieldset skeleton — enable toggle and interval

Delivers a working, savable fieldset with the two scalar settings. The network list arrives in Task 3.

**Files:**
- Modify: `vehicle/OVMS.V3/components/ovms_webserver/src/ovms_webserver.h:576-577` (after the `UpdateWifiTable` declaration)
- Modify: `vehicle/OVMS.V3/components/ovms_webserver/src/web_cfg_wifi.cpp`

**Interfaces:**
- Produces, both consumed by Tasks 3-5:
  - `static void OutputWifiPriority(PageEntry_t& p, PageContext_t& c);`
  - `static void UpdateWifiPriority(PageEntry_t& p, PageContext_t& c, std::string& warn, std::string& error);`
- Form field names established here and relied on later: `cfg_priority_enable` (checkbox, "yes"), `cfg_priority_interval` (slider, int seconds).

- [ ] **Step 1: Declare the two methods**

In `ovms_webserver.h`, immediately after the `UpdateWifiTable` declaration that ends at line 577:

```cpp
    static void OutputWifiPriority(PageEntry_t& p, PageContext_t& c);
    static void UpdateWifiPriority(PageEntry_t& p, PageContext_t& c,
      std::string& warn, std::string& error);
```

- [ ] **Step 2: Add the render function**

At the end of `web_cfg_wifi.cpp`, after `UpdateWifiTable`:

```cpp
void OvmsWebServer::OutputWifiPriority(PageEntry_t& p, PageContext_t& c)
{
  auto lock = MyConfig.Lock();
  bool enable;
  int interval;

  if (c.method == "POST") {
    enable   = (c.getvar("cfg_priority_enable") == "yes");
    interval = atoi(c.getvar("cfg_priority_interval").c_str());
  }
  else {
    enable   = MyConfig.GetParamValueBool("network", "wifi.priority.enable", false);
    interval = MyConfig.GetParamValueInt("network", "wifi.priority.interval", 60);
  }

  c.input_checkbox("Enable priority networks", "cfg_priority_enable", enable,
    "<p>Prefer known networks in the order listed below. While connected to a lower ranked"
    " network the module periodically rescans and upgrades to a higher ranked one when it is"
    " in range with a good signal.</p>");

  c.input_slider("Upgrade-scan interval", "cfg_priority_interval", 3, "s", -1,
    interval, 60, 10, 600, 1,
    "<p>How often to rescan for a higher ranked network while connected to a lower ranked"
    " one. Minimum 10 seconds.</p>");
}
```

- [ ] **Step 3: Add the update function**

Immediately after `OutputWifiPriority`:

```cpp
void OvmsWebServer::UpdateWifiPriority(PageEntry_t& p, PageContext_t& c,
  std::string& warn, std::string& error)
{
  auto lock = MyConfig.Lock();

  bool enable  = (c.getvar("cfg_priority_enable") == "yes");
  int interval = atoi(c.getvar("cfg_priority_interval").c_str());

  if (interval < 10) {
    error += "<li data-input=\"cfg_priority_interval\">Upgrade-scan interval must be at least 10 seconds</li>";
    return;
  }

  if (!enable)
    MyConfig.DeleteInstance("network", "wifi.priority.enable");
  else
    MyConfig.SetParamValueBool("network", "wifi.priority.enable", enable);
  if (interval == 60)
    MyConfig.DeleteInstance("network", "wifi.priority.interval");
  else
    MyConfig.SetParamValueInt("network", "wifi.priority.interval", interval);
}
```

- [ ] **Step 4: Wire into `HandleCfgWifi`**

Two edits. First, in the POST branch, directly after the existing `UpdateWifiTable(p, c, "client", "wifi.ssid", warn, error, 0);` at line 45:

```cpp
    UpdateWifiPriority(p, c, warn, error);
```

Second, in the form-generation section, directly after the existing "Wifi client networks" `c.fieldset_end();` at line 143:

```cpp
  c.fieldset_start("Wifi priority networks");
  OutputWifiPriority(p, c);
  c.fieldset_end();
```

- [ ] **Step 5: Push and verify CI**

```bash
git add vehicle/OVMS.V3/components/ovms_webserver/src/ovms_webserver.h \
        vehicle/OVMS.V3/components/ovms_webserver/src/web_cfg_wifi.cpp
git commit -m "webserver: wifi priority networks fieldset - enable toggle and interval"
git push -u origin feature/wifi-priority-web-ui
gh workflow run build.yml --ref feature/wifi-priority-web-ui
```

Then poll until conclusion, and treat anything other than `success` as a failure to fix before continuing:

```bash
gh run list --branch feature/wifi-priority-web-ui --limit 3 \
  --json workflowName,status,conclusion,headSha
```

Expected: `Firmware build` → `success`.

---

### Task 3: Render the ranked network list

**Files:**
- Modify: `vehicle/OVMS.V3/components/ovms_webserver/src/web_cfg_wifi.cpp`

**Interfaces:**
- Consumes: `OutputWifiPriority` from Task 2.
- Produces, consumed by Tasks 4-5:
  - `static void ParsePriorityCsv(const std::string& csv, std::vector<std::string>& list);`
  - `static bool InList(const std::vector<std::string>& list, const std::string& s);`
  - Form field `cfg_priority` — hidden input, CSV of checked SSIDs in display order.

**Intermediate state:** after this task the list renders and reorders in the browser but is not yet persisted — `UpdateWifiPriority` still ignores `cfg_priority`. That is a safe no-op, not a regression. Task 4 closes it.

- [ ] **Step 1: Add the two file-local helpers**

In `web_cfg_wifi.cpp`, directly after the `_html` macro at line 27:

```cpp
// Split a comma separated SSID list, trimming whitespace, first occurrence wins.
// Deliberately mirrors esp32wifi::ParsePriorityList() so the page orders and
// de-duplicates exactly the way the firmware will.
static void ParsePriorityCsv(const std::string& csv, std::vector<std::string>& list)
{
  list.clear();
  size_t start = 0;
  while (start <= csv.length()) {
    size_t comma = csv.find(',', start);
    if (comma == std::string::npos) comma = csv.length();
    std::string item = csv.substr(start, comma - start);
    size_t a = item.find_first_not_of(" \t");
    size_t b = item.find_last_not_of(" \t");
    if (a != std::string::npos) {
      item = item.substr(a, b - a + 1);
      bool dup = false;
      for (size_t i = 0; i < list.size(); i++) {
        if (list[i] == item) { dup = true; break; }
      }
      if (!dup)
        list.push_back(item);
    }
    start = comma + 1;
  }
}

static bool InList(const std::vector<std::string>& list, const std::string& s)
{
  for (size_t i = 0; i < list.size(); i++) {
    if (list[i] == s) return true;
  }
  return false;
}
```

- [ ] **Step 2: Read the CSV in `OutputWifiPriority`**

In `OutputWifiPriority`, extend the declarations and both branches of the `c.method == "POST"` test:

```cpp
  bool enable;
  int interval;
  std::string csv;

  if (c.method == "POST") {
    enable   = (c.getvar("cfg_priority_enable") == "yes");
    interval = atoi(c.getvar("cfg_priority_interval").c_str());
    csv      = c.getvar("cfg_priority");
  }
  else {
    enable   = MyConfig.GetParamValueBool("network", "wifi.priority.enable", false);
    interval = MyConfig.GetParamValueInt("network", "wifi.priority.interval", 60);
    csv      = MyConfig.GetParamValue("network", "wifi.priority", "");
  }
```

- [ ] **Step 3: Emit the table**

Between the `c.input_checkbox(...)` call and the `c.input_slider(...)` call:

```cpp
  OvmsConfigParam* ssidparam = MyConfig.CachedParam("wifi.ssid");
  std::vector<std::string> prio;
  ParsePriorityCsv(csv, prio);

  c.print(
    "<div class=\"table-responsive\">"
      "<table class=\"table table-condensed\" id=\"prio-table\">"
        "<thead>"
          "<tr>"
            "<th width=\"10%\">Use</th>"
            "<th width=\"12%\">Rank</th>"
            "<th width=\"52%\">Network</th>"
            "<th width=\"26%\">Order</th>"
          "</tr>"
        "</thead>"
        "<tbody>");

  auto gen_row = [&c](const std::string& ssid, bool checked, bool haspass) {
    c.printf(
          "<tr data-ssid=\"%s\">"
            "<td><input type=\"checkbox\" class=\"prio-use\"%s></td>"
            "<td class=\"prio-rank\"></td>"
            "<td>%s%s</td>"
            "<td>"
              "<button type=\"button\" class=\"btn btn-default btn-xs prio-up\"><strong>&#9650;</strong></button> "
              "<button type=\"button\" class=\"btn btn-default btn-xs prio-down\"><strong>&#9660;</strong></button>"
            "</td>"
          "</tr>"
      , _attr(ssid)
      , checked ? " checked" : ""
      , _html(ssid)
      , haspass ? "" : " <span class=\"text-warning\">(no saved password)</span>");
  };

  // listed networks first, in priority order:
  for (size_t i = 0; i < prio.size(); i++)
    gen_row(prio[i], true, !ssidparam->GetValue(prio[i]).empty());

  // then saved networks that are not on the list, alphabetically (map order):
  for (auto const& kv : ssidparam->m_instances) {
    if (endsWith(kv.first, ".ovms.staticip"))
      continue;
    if (InList(prio, kv.first))
      continue;
    gen_row(kv.first, false, !kv.second.empty());
  }

  c.printf(
        "</tbody>"
      "</table>"
    "</div>"
    "<input type=\"hidden\" name=\"cfg_priority\" id=\"cfg_priority\" value=\"%s\">"
    , _attr(csv));
```

- [ ] **Step 4: Add the inline JS**

Directly after the hidden-input `c.printf` above, still inside `OutputWifiPriority`:

```cpp
  c.print(
    "<script>"
    "(function(){"
      "var $t = $('#prio-table');"
      "function refresh(){"
        "var csv = [];"
        "$t.find('tbody > tr').each(function(){"
          "var $tr = $(this);"
          "if ($tr.find('.prio-use').prop('checked')) {"
            "csv.push($tr.attr('data-ssid'));"
            "$tr.find('.prio-rank').text(csv.length);"
          "} else {"
            "$tr.find('.prio-rank').text('—');"
          "}"
        "});"
        "$('#cfg_priority').val(csv.join(','));"
      "}"
      "$t.on('click', '.prio-up', function(){"
        "var $tr = $(this).closest('tr'), $prev = $tr.prev();"
        "if ($prev.length) { $tr.insertBefore($prev); refresh(); }"
      "});"
      "$t.on('click', '.prio-down', function(){"
        "var $tr = $(this).closest('tr'), $next = $tr.next();"
        "if ($next.length) { $tr.insertAfter($next); refresh(); }"
      "});"
      "$t.on('change', '.prio-use', refresh);"
      "refresh();"
    "})();"
    "</script>");
```

**Do not change `$tr.attr('data-ssid')` to `$tr.data('ssid')`.** jQuery's `.data()` type-coerces values that look numeric, so an SSID such as `1e5` would be read back as `100000` and silently written into the config. `.attr()` always returns the string.

- [ ] **Step 5: Push and verify CI**

```bash
git add vehicle/OVMS.V3/components/ovms_webserver/src/web_cfg_wifi.cpp
git commit -m "webserver: render ranked wifi priority network list"
git push
gh run list --branch feature/wifi-priority-web-ui --limit 3 \
  --json workflowName,status,conclusion
```

Expected: `Firmware build` → `success`.

---

### Task 4: Persist the list

**Files:**
- Modify: `vehicle/OVMS.V3/components/ovms_webserver/src/web_cfg_wifi.cpp` (`UpdateWifiPriority`)

**Interfaces:**
- Consumes: `ParsePriorityCsv`, `InList` (Task 3); form field `cfg_priority` (Task 3).

- [ ] **Step 1: Parse and validate**

In `UpdateWifiPriority`, after the existing `interval` declaration and before the `interval < 10` test:

```cpp
  std::string csv = c.getvar("cfg_priority");
  std::vector<std::string> prio;
  ParsePriorityCsv(csv, prio);
```

Then change the early-return validation block to cover both errors:

```cpp
  if (interval < 10)
    error += "<li data-input=\"cfg_priority_interval\">Upgrade-scan interval must be at least 10 seconds</li>";
  if (enable && prio.empty())
    error += "<li data-input=\"cfg_priority\">Enable priority networks: select at least one network</li>";
  if (error != "")
    return;
```

- [ ] **Step 2: Persist the list**

Between the `wifi.priority.enable` write and the `wifi.priority.interval` write:

```cpp
  if (prio.empty()) {
    MyConfig.DeleteInstance("network", "wifi.priority");
  }
  else {
    std::string out;
    for (size_t i = 0; i < prio.size(); i++) {
      if (i) out += ",";
      out += prio[i];
    }
    MyConfig.SetParamValue("network", "wifi.priority", out);
  }
```

- [ ] **Step 3: Push and verify CI**

```bash
git add vehicle/OVMS.V3/components/ovms_webserver/src/web_cfg_wifi.cpp
git commit -m "webserver: persist wifi priority network list and order"
git push
gh run list --branch feature/wifi-priority-web-ui --limit 3 \
  --json workflowName,status,conclusion
```

Expected: `Firmware build` → `success`.

---

### Task 5: The four warnings

**Files:**
- Modify: `vehicle/OVMS.V3/components/ovms_webserver/src/web_cfg_wifi.cpp` (`UpdateWifiPriority`)

**Interfaces:**
- Consumes: `prio` and `ssidparam` in `UpdateWifiPriority`; `InList` (Task 3).

- [ ] **Step 1: Add the config-param handle**

At the top of `UpdateWifiPriority`, after the `auto lock = MyConfig.Lock();` line:

```cpp
  OvmsConfigParam* ssidparam = MyConfig.CachedParam("wifi.ssid");
```

- [ ] **Step 2: Emit the warnings**

After the `if (error != "") return;` guard and before the first `MyConfig` write:

```cpp
  // listed but no stored credential -> SelectPriorityAP() will skip it:
  for (size_t i = 0; i < prio.size(); i++) {
    if (ssidparam->GetValue(prio[i]).empty()) {
      warn += "<li>Priority network <code>" + c.encode_html(prio[i])
            + "</code> has no saved password and will be skipped</li>";
    }
  }

  if (enable) {
    // saved but unlisted -> never joined while priority is on. Aggregated into one
    // message on purpose: one warning per network trains the user to ignore the block.
    std::string unlisted;
    int n = 0;
    for (auto const& kv : ssidparam->m_instances) {
      if (endsWith(kv.first, ".ovms.staticip"))
        continue;
      if (InList(prio, kv.first))
        continue;
      if (n++) unlisted += ", ";
      unlisted += "<code>" + c.encode_html(kv.first) + "</code>";
    }
    if (n) {
      warn += "<li>While priority networks are enabled only listed networks are joined."
              " These saved networks will be ignored: " + unlisted + "</li>";
    }

    // conditions that make PriorityActive() false regardless of the list:
    std::string autossid = MyConfig.GetParamValue("auto", "wifi.ssid.client");
    std::string automode = MyConfig.GetParamValue("auto", "wifi.mode", "ap");
    if (!autossid.empty()) {
      warn += "<li>Priority networks are inactive: a fixed client SSID (<code>"
            + c.encode_html(autossid)
            + "</code>) is configured. Clear it on the <a href=\"/cfg/autostart\" target=\"#main\">"
              "Autostart configuration page</a> to enable scanning mode.</li>";
    }
    if (automode != "client" && automode != "apclient") {
      warn += "<li>Priority networks are inactive: Wifi mode is <code>"
            + c.encode_html(automode)
            + "</code>. Select client or access point + client mode on the"
              " <a href=\"/cfg/autostart\" target=\"#main\">Autostart configuration page</a>.</li>";
    }
  }
```

Every interpolated value goes through `c.encode_html`. Do not shorten these to the
`_html()` macro: that macro yields a `const char*` into a temporary, which is only safe
inside a single `printf`-style call, not in `+` concatenation chains like these.

- [ ] **Step 3: Commit the warnings**

```bash
git add vehicle/OVMS.V3/components/ovms_webserver/src/web_cfg_wifi.cpp
git commit -m "webserver: warn on wifi priority conditions that silently disable it"
```

- [ ] **Step 4: Escape the three pre-existing warning strings**

Separate commit — this corrects existing code, not new code, and upstream should be able to
read it on its own.

In `UpdateWifiTable`, three lines interpolate an unescaped, user-supplied SSID. At
`web_cfg_wifi.cpp:390,392,396` (line numbers before this task's edits), change:

```cpp
        error += "<li>Autostart SSID <code>" + ssid + "</code> has no password</li>";
      else
        warn += "<li>SSID <code>" + ssid + "</code> has no password</li>";
```
```cpp
      error += "<li>SSID <code>" + ssid + "</code>: password is too short (min " + buf + " chars)</li>";
```

to:

```cpp
        error += "<li>Autostart SSID <code>" + c.encode_html(ssid) + "</code> has no password</li>";
      else
        warn += "<li>SSID <code>" + c.encode_html(ssid) + "</code> has no password</li>";
```
```cpp
      error += "<li>SSID <code>" + c.encode_html(ssid) + "</code>: password is too short (min " + buf + " chars)</li>";
```

`buf` is a locally-formatted integer and needs no escaping. Do not change anything else in
`UpdateWifiTable` — its logic stays exactly as it is.

- [ ] **Step 5: Push and verify CI**

```bash
git add vehicle/OVMS.V3/components/ovms_webserver/src/web_cfg_wifi.cpp
git commit -m "webserver: HTML-escape SSIDs in wifi config warning messages"
git push
gh run list --branch feature/wifi-priority-web-ui --limit 3 \
  --json workflowName,status,conclusion
```

Expected: `Firmware build` → `success`.

---

### Task 6: Documentation

**Files:**
- Modify: `docs/source/userguide/wifi/client.rst`
- Modify: `vehicle/OVMS.V3/changes.txt`

- [ ] **Step 1: Extend the user guide**

Append to the priority-networks section of `docs/source/userguide/wifi/client.rst`:

```rst
Configuring priority networks from the web UI
---------------------------------------------

The priority list can be managed on the **Config → Wifi** page, in the
*Wifi priority networks* section. Tick the networks to prioritise and use the
▲/▼ buttons to order them; rank 1 is preferred. Only networks already saved
under *Wifi client networks* can be selected, so an SSID cannot be mistyped.

Priority networks are inactive — regardless of the list — when any of these
apply. The web page warns about each:

- a fixed client SSID is configured on the Autostart page (the module then
  connects to that SSID only, instead of scanning),
- the Wifi mode is not *client* or *access point + client*,
- the priority list is empty.

A listed network with no saved password is skipped, and while priority
networks are enabled a saved network that is **not** on the list is never
joined.
```

Match the underline lengths to the heading text exactly — RST underlines shorter than their title produce Sphinx warnings, and the `Docs build` workflow runs on any `*.rst` change.

- [ ] **Step 2: Amend the existing changes.txt entry**

`changes.txt` already carries a WiFi priority entry from PR #120. **Do not add a second entry** — amend the existing one, since this file is the single largest upstream merge-conflict source. Extend its first paragraph so it ends:

```
    Hidden SSIDs cannot participate. The list can be configured from the web UI on the
    Config → Wifi page, which also warns when the feature is inactive (fixed client SSID
    or non-client Wifi mode), when a listed network has no saved password, and when a
    saved network is not on the list.
```

- [ ] **Step 3: Push and verify BOTH workflows**

A `*.rst` change triggers `Docs build` in addition to `Firmware build`. A green PR is not a green master here — the Pages `deploy` job is gated on `refs/heads/master` and is skipped on PRs.

```bash
git add docs/source/userguide/wifi/client.rst vehicle/OVMS.V3/changes.txt
git commit -m "docs: document wifi priority network web configuration"
git push
gh run list --branch feature/wifi-priority-web-ui --limit 6 \
  --json workflowName,status,conclusion,headSha
```

Expected: both `Firmware build` and `Docs build` → `success`.

---

### Task 7: On-device validation

Nothing here is verifiable from CI. Flash the bench module and work the matrix.

**Files:** none — this task produces a validation record, not a diff.

- [ ] **Step 1: Fetch the CI binary**

The build artifacts land in MinIO, not GitHub artifacts:

```bash
mc cp sh/ci-artifacts/<run_id>/ovms3/ovms3.bin ./ovms3.bin
```

A PR build embeds the merge SHA, not the branch head, so `ota status` will report a hash you do not recognise. That is expected.

- [ ] **Step 2: Flash via the launcher**

Use the launcher `/ota/upload` path (`autostart:false` → upload to the default slot → restore `autostart:true` → `/launch`), per `CLAUDE.local.md`. Read `/status` for `default_slot` rather than assuming it.

- [ ] **Step 3: Work the matrix**

Record pass/fail for each:

| # | Check | Expected |
|---|---|---|
| 1 | Toggle enable on, save, reload | Checkbox stays on; `config list network` shows `wifi.priority.enable` |
| 2 | Toggle enable off, save | Instance **deleted**, not set to `no` |
| 3 | Tick two networks, order them, save, reload | Order preserved and matches `wifi.priority` CSV exactly |
| 4 | Set interval 60, save | `wifi.priority.interval` **deleted** (delete-on-default) |
| 5 | Set interval 5, save | Rejected with the interval error; nothing written |
| 6 | Enable with nothing ticked, save | Rejected with the empty-list error |
| 7 | `config set network wifi.priority "A,B"` then load page | Both rows ticked, in that order, ranks 1 and 2 |
| 8 | Add a list entry with no saved password | Row shows "(no saved password)"; warning names it on save |
| 9 | Save with a saved-but-unlisted network | One aggregated warning naming every such network |
| 10 | Set a fixed client SSID on Autostart, save wifi page | Inactive warning with the Autostart link |
| 11 | Set Wifi mode to `ap`, save wifi page | Mode inactive warning |
| 12 | `wifi status` with fixed SSID set | `Priority: inactive - fixed SSID configured` |
| 13 | `wifi status` with an empty list | `Priority: inactive - network list is empty` |
| 14 | An SSID containing only digits, e.g. `1e5` | Round-trips through the CSV unchanged (the `.attr()` guard) |

- [ ] **Step 4: Record the result**

Append the outcome to the spec document under a new "Validation" heading, then commit:

```bash
git add docs/superpowers/specs/2026-07-26-wifi-priority-web-ui-design.md
git commit -m "docs: record on-device validation of wifi priority web UI"
git push
```

---

## Upstream PR shaping (after Task 7 passes)

Not part of the implementation. Recorded so it is not lost: the upstream PR must be built as `upstream/master` + the PR #120 firmware commits + Tasks 1-6, with no fork-local content — no `.github/`, `.devcontainer/`, `CLAUDE.md`, `docs/superpowers/`, `partitions.csv`, `sdkconfig` or `support/a2l` changes. Note that `gh pr create` cannot open upstream PRs with the current token; prepare the branch and hand over the compare URL.
