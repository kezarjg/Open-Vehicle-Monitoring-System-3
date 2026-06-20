# e-TNGA Charge Report v2 — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade the e-TNGA end-of-charge report to a richer artifact: a self-contained HTML report (location, ambient, human-readable outcome, inline-SVG power-vs-time, event log, estimates) plus a fine-grained companion CSV for offline analysis, written to `/sd` (fallback `/store`); and fix the peak-power bug.

**Architecture:** Extends the v1 report code (`etnga_charge_report.cpp`, `ChargeSessionState`). Per-tick aggregation in `UpdateChargeSessionStats()` (peak/temp/ambient/event samples + stream a CSV row + feed a small SVG buffer); `GenerateChargeReport()` at session close writes the HTML (incl. generated SVG) and finalises the CSV, then prunes to 50 sessions. A one-line fix makes `ms_v_charge_power` reflect battery-delivered power in both AC and DC.

**Tech Stack:** C++ (ESP-IDF 3.3 / older GCC), OVMS metrics + VFS (`std::ofstream`, POSIX `opendir`/`mkdir`/`unlink`), `vehicle_toyota_etnga` component.

**Base branch:** Develop against a base containing #83/#85/#86/#88 (merged master, or `test/etnga-combined`). This worktree (`feature/etnga-charge-report-v2`) is off #86; rebase onto the combined/merged base before the web-viewer task (Task 10), which needs #88's `etnga_web.cpp`.

**Test model:** No host test suite exists (CLAUDE.md). Verification is: (a) each task compiles — confirmed by the CI build in Task 11; (b) on-vehicle behavioural checks in Task 12. Commit after each task.

---

## File Structure

- `components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h` — extend `ChargeSessionState`; declare new helpers.
- `components/vehicle_toyota_etnga/src/etnga_metrics.cpp` — charge-power-source fix (`SetBatteryChargingPower`).
- `components/vehicle_toyota_etnga/src/etnga_poll_states.cpp` — capture location/ambient at open; append event-log entries in transitions.
- `components/vehicle_toyota_etnga/src/etnga_charge_report.cpp` — owns aggregation, CSV streaming, SVG generation, HTML, dir resolution, retention, outcome-label table.
- `components/vehicle_toyota_etnga/src/etnga_web.cpp` — extend `/xte/report` to serve `.csv`; `/xte/reports` lists CSV link (needs #88 base).
- `changes.txt` — user-facing entry.

---

## Task 1: Fix charge-power source (peak-power bug)

**Files:**
- Modify: `components/vehicle_toyota_etnga/src/etnga_metrics.cpp` (`SetBatteryChargingPower`, ~line 394)

- [ ] **Step 1: Set delivered power from 0x10D4 in both AC+DC**

In `SetBatteryChargingPower()`, add the live-power set right after the log line:

```cpp
void OvmsVehicleToyotaETNGA::SetBatteryChargingPower(float power)
{
    ESP_LOGD(TAG, "Battery Charging Power: %f", power);

    // Delivered charge power (valid in AC and DC; 0x161D only answers on AC). This is the
    // authoritative "power delivered to the battery" used for peak/avg, the chart and the CSV.
    StandardMetrics.ms_v_charge_power->SetValue(power);

    float hoursSinceLastUpdate = 1.0f / 60.0f / 60.0f; // Default value of 1 second
    // ... (existing energy integration unchanged) ...
```

Leave `SetChargerInputPower()` (`0x161D`) as-is for grid energy, but remove its `ms_v_charge_power->SetValue(power)` line (it set grid-input power into the delivered metric — the source of the bug):

```cpp
// in SetChargerInputPower(): DELETE this line
StandardMetrics.ms_v_charge_power->SetValue(power);
```

- [ ] **Step 2: Commit**

```bash
git add components/vehicle_toyota_etnga/src/etnga_metrics.cpp
git commit -m "etnga: fix charge power source — ms_v_charge_power from 0x10D4 (AC+DC), not AC-only grid input"
```

---

## Task 2: Extend ChargeSessionState

**Files:**
- Modify: `components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h` (`ChargeSessionState`, ~line 60; add `#include <vector>`, `#include <utility>` near the top if not present)

- [ ] **Step 1: Replace the struct with the v2 fields**

```cpp
    struct ChargeSessionState {
        bool  in_session = false;
        int   start_monotonic = 0;
        int   start_soc = -1;
        int   start_utc = 0;
        bool  is_dc = false;
        float peak_power = 0.0f;
        bool  temp_seen = false;
        float temp_min = 0.0f;
        float temp_max = 0.0f;
        // v2: location + ambient captured at open
        bool  has_loc = false;
        float start_lat = 0.0f;
        float start_lon = 0.0f;
        bool  amb_seen = false;
        float amb_min = 0.0f;
        float amb_max = 0.0f;
        // v2: charge-side coulomb counter (Ah) for the implied-capacity estimate
        float delivered_ah = 0.0f;
        int   last_sample_monotonic = 0;   // dt for delivered_ah + CSV row cadence
        // v2: event log (monotonic seconds, static label string)
        std::vector<std::pair<int,const char*>> events;
        // v2: downsampled chart buffer
        struct Sample { int t_s; float kw; int soc; };
        std::vector<Sample> svg;
        int   svg_interval_s = 20;
        int   last_svg_monotonic = 0;
        // v2: file basename (resolved "<dir>/<timestamp>", no extension) + CSV state
        std::string base;
        bool  csv_started = false;
    };
    ChargeSessionState m_charge_session;
```

- [ ] **Step 2: Declare new helpers (in the private section, near the existing report decls)**

```cpp
    void LogChargeEvent(const char* label);            // append a timestamped event
    void AppendChargeCsvRow();                          // stream one CSV row (opens+header on first call)
    std::string ChargeReportDir();                      // "/sd/charge-reports" if SD mounted else "/store/..."
    static const char* ChargeOutcomeLabel(int code);    // 0x1688 enum -> human text
```

- [ ] **Step 3: Commit**

```bash
git add components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h
git commit -m "etnga: extend ChargeSessionState for report v2 (location, ambient, event log, chart/csv state)"
```

---

## Task 3: Directory resolution + outcome-label table

**Files:**
- Modify: `components/vehicle_toyota_etnga/src/etnga_charge_report.cpp` (add includes `<sys/statvfs.h>` not needed; reuse existing `<sys/stat.h>`)

- [ ] **Step 1: Add ChargeReportDir() (SD if mounted, else /store)**

At top of the file replace the fixed `#define CHARGE_REPORT_DIR` usage with a resolver. Add:

```cpp
// Prefer the SD card (GBs, removable) for reports+CSV; fall back to internal flash.
std::string OvmsVehicleToyotaETNGA::ChargeReportDir()
{
    struct stat st;
    if (stat("/sd", &st) == 0 && S_ISDIR(st.st_mode))
        return "/sd/charge-reports";
    return "/store/charge-reports";
}
```

Note: `/sd` exists as a mount point only when an SD card is mounted; `stat` returns success on the mounted dir. (If you prefer an explicit check, `MyConfig`/sdcard status APIs exist, but `stat("/sd")` is sufficient and dependency-free.)

- [ ] **Step 2: Add the 0x1688 outcome-label table**

Source the enum from `solterra-can/docs/ecus/plug-in-charge-control.md`. Add:

```cpp
const char* OvmsVehicleToyotaETNGA::ChargeOutcomeLabel(int code)
{
    switch (code & 0xFF) {
        case 0x21: return "AC Charging Complete (Full Charge)";
        // TODO-AT-IMPLEMENTATION: fill the remaining entries from
        // solterra-can/docs/ecus/plug-in-charge-control.md (0x1688 26-state enum).
        default:   return "";   // unknown -> caller shows raw hex
    }
}
```

> When implementing, replace the single case with the full decoded table from the doc. The report falls back to raw hex when the label is empty, so partial coverage is safe.

- [ ] **Step 3: Commit**

```bash
git add components/vehicle_toyota_etnga/src/etnga_charge_report.cpp
git commit -m "etnga charge report: /sd-with-/store-fallback dir resolver + 0x1688 outcome label table"
```

---

## Task 4: Capture location + ambient at session open; reset

**Files:**
- Modify: `components/vehicle_toyota_etnga/src/etnga_poll_states.cpp` (`TransitionToChargeHandshakeState`, the `if (!in_session)` block)

- [ ] **Step 1: Snapshot location + ambient at open**

In `TransitionToChargeHandshakeState()`, inside the `if (!m_charge_session.in_session)` block, after `start_soc`:

```cpp
        m_charge_session.start_soc = (int) StandardMetrics.ms_v_bat_soc->AsFloat();
        if (StandardMetrics.ms_v_pos_gpslock->AsBool()) {
            m_charge_session.has_loc = true;
            m_charge_session.start_lat = StandardMetrics.ms_v_pos_latitude->AsFloat();
            m_charge_session.start_lon = StandardMetrics.ms_v_pos_longitude->AsFloat();
        }
        float amb = StandardMetrics.ms_v_env_temp->AsFloat();
        m_charge_session.amb_seen = true;
        m_charge_session.amb_min = m_charge_session.amb_max = amb;
        m_charge_session.svg_interval_s = 20;
        m_charge_session.last_sample_monotonic = 0;
        m_charge_session.last_svg_monotonic = 0;
        ESP_LOGI(TAG, "Charge session opened (SOC %d%%)", m_charge_session.start_soc);
```

- [ ] **Step 2: Commit**

```bash
git add components/vehicle_toyota_etnga/src/etnga_poll_states.cpp
git commit -m "etnga charge report: capture plug-in location + ambient at session open"
```

---

## Task 5: Event log

**Files:**
- Modify: `components/vehicle_toyota_etnga/src/etnga_charge_report.cpp` (add `LogChargeEvent`)
- Modify: `components/vehicle_toyota_etnga/src/etnga_poll_states.cpp` (call it in transitions)

- [ ] **Step 1: Implement LogChargeEvent**

```cpp
void OvmsVehicleToyotaETNGA::LogChargeEvent(const char* label)
{
    if (!m_charge_session.in_session)
        return;
    m_charge_session.events.push_back(
        std::make_pair(StandardMetrics.ms_m_monotonic->AsInt(), label));
}
```

- [ ] **Step 2: Call it at the transitions**

In `etnga_poll_states.cpp`:
- `TransitionToChargeHandshakeState()` (inside the open block): `LogChargeEvent("Plugged in — handshake");`
- `TransitionToChargeAcState()` (or where CHARGE_AC is entered): `LogChargeEvent("AC charging started");`
- `TransitionToChargeDcState()`: `LogChargeEvent("DC charging started");`
- `TransitionToChargeWaitState()`: `LogChargeEvent("Charging paused / phase ended");`
- In `TransitionToAwakeState()`, the `oldState == CHARGE_HANDSHAKE` re-arm branch: `LogChargeEvent("Unplug bounce — re-armed (DCFC retry)");`
- At session close (both branches, just before `GenerateChargeReport()`): `LogChargeEvent("Unplugged");`

(Place each call after the state is set / inside the relevant branch, only meaningful while `in_session`.)

- [ ] **Step 3: Commit**

```bash
git add components/vehicle_toyota_etnga/src/etnga_charge_report.cpp components/vehicle_toyota_etnga/src/etnga_poll_states.cpp
git commit -m "etnga charge report: timestamped session event log"
```

---

## Task 6: Per-tick aggregation — ambient, delivered-Ah, CSV row, SVG sample

**Files:**
- Modify: `components/vehicle_toyota_etnga/src/etnga_charge_report.cpp` (`UpdateChargeSessionStats`, `AppendChargeCsvRow`)

- [ ] **Step 1: Extend UpdateChargeSessionStats**

Replace the v1 body with:

```cpp
void OvmsVehicleToyotaETNGA::UpdateChargeSessionStats()
{
    if (!m_charge_session.in_session)
        return;

    int now = StandardMetrics.ms_m_monotonic->AsInt();

    float p = StandardMetrics.ms_v_charge_power->AsFloat();
    if (p > m_charge_session.peak_power)
        m_charge_session.peak_power = p;

    float t = StandardMetrics.ms_v_bat_temp->AsFloat();
    if (!m_charge_session.temp_seen) {
        m_charge_session.temp_min = m_charge_session.temp_max = t;
        m_charge_session.temp_seen = true;
    } else if (t < m_charge_session.temp_min) m_charge_session.temp_min = t;
    else if (t > m_charge_session.temp_max) m_charge_session.temp_max = t;

    float amb = StandardMetrics.ms_v_env_temp->AsFloat();
    if (!m_charge_session.amb_seen) {
        m_charge_session.amb_min = m_charge_session.amb_max = amb;
        m_charge_session.amb_seen = true;
    } else if (amb < m_charge_session.amb_min) m_charge_session.amb_min = amb;
    else if (amb > m_charge_session.amb_max) m_charge_session.amb_max = amb;

    m_charge_session.is_dc = (static_cast<PollState>(m_poll_state) == PollState::CHARGE_DC);

    // Delivered-Ah (charge-side coulomb): integrate pack current over dt.
    if (m_charge_session.last_sample_monotonic != 0) {
        float dt_h = (now - m_charge_session.last_sample_monotonic) / 3600.0f;
        m_charge_session.delivered_ah += fabsf(StandardMetrics.ms_v_bat_current->AsFloat()) * dt_h;
    }
    m_charge_session.last_sample_monotonic = now;

    // Stream a CSV row every tick (~1 s while charging).
    AppendChargeCsvRow();

    // Feed the downsampled SVG buffer at svg_interval_s.
    if (m_charge_session.last_svg_monotonic == 0 ||
        now - m_charge_session.last_svg_monotonic >= m_charge_session.svg_interval_s) {
        ChargeSessionState::Sample s;
        s.t_s = now - m_charge_session.start_monotonic;
        s.kw  = p;
        s.soc = (int) StandardMetrics.ms_v_bat_soc->AsFloat();
        m_charge_session.svg.push_back(s);
        m_charge_session.last_svg_monotonic = now;
        // Cap ~300 points: when exceeded, double the interval and decimate (keep every other).
        if (m_charge_session.svg.size() > 300) {
            std::vector<ChargeSessionState::Sample> dec;
            for (size_t i = 0; i < m_charge_session.svg.size(); i += 2)
                dec.push_back(m_charge_session.svg[i]);
            m_charge_session.svg.swap(dec);
            m_charge_session.svg_interval_s *= 2;
        }
    }
}
```

Add `#include <math.h>` (for `fabsf`) to the file's includes.

- [ ] **Step 2: Implement AppendChargeCsvRow**

```cpp
void OvmsVehicleToyotaETNGA::AppendChargeCsvRow()
{
    if (m_charge_session.base.empty())
        return;
    std::string path = m_charge_session.base + ".csv";
    std::ofstream f(path, m_charge_session.csv_started
        ? (std::ios::out | std::ios::app)
        : (std::ios::out | std::ios::trunc));
    if (!f) return;

    if (!m_charge_session.csv_started) {
        f << "elapsed_s,soc_pct,delivered_kw,pack_v,pack_a,batt_temp_c,ambient_c,state,"
             "station_max_kw,station_max_a,station_max_v,car_perm_kw,target_a,grid_kw,present_v,present_a\n";
        m_charge_session.csv_started = true;
    }

    int elapsed = StandardMetrics.ms_m_monotonic->AsInt() - m_charge_session.start_monotonic;
    char row[256];
    snprintf(row, sizeof(row),
        "%d,%.0f,%.3f,%.1f,%.1f,%.1f,%.1f,%s,"
        "%.2f,%.0f,%.0f,%.2f,%.0f,%.3f,%.0f,%.0f\n",
        elapsed,
        StandardMetrics.ms_v_bat_soc->AsFloat(),
        StandardMetrics.ms_v_charge_power->AsFloat(),
        StandardMetrics.ms_v_bat_voltage->AsFloat(),
        StandardMetrics.ms_v_bat_current->AsFloat(),
        StandardMetrics.ms_v_bat_temp->AsFloat(),
        StandardMetrics.ms_v_env_temp->AsFloat(),
        (m_charge_session.is_dc ? "DC" : "AC"),
        m_v_charge_sta_max_p->AsFloat(), m_v_charge_sta_max_i->AsFloat(), m_v_charge_sta_max_v->AsFloat(),
        m_v_charge_perm->AsFloat(), m_v_charge_tgti->AsFloat(),
        StandardMetrics.ms_v_charge_kwh_grid->AsFloat() == 0 ? 0.0f : 0.0f, // grid live power: see note
        StandardMetrics.ms_v_charge_voltage->AsFloat(), StandardMetrics.ms_v_charge_current->AsFloat());
    f << row;
}
```

> Note on `grid_kw`: there is no live grid-power metric after the Task 1 fix (we removed it from `ms_v_charge_power`). Either (a) add a small `xte.v.c.gridpower` metric set in `SetChargerInputPower`, or (b) drop the `grid_kw` column. Recommended: add `xte.v.c.gridpower` (one `InitFloat` + one `SetValue` in `SetChargerInputPower`) and log it here. Decide at implementation; if dropped, remove the column from both the header and the row.

- [ ] **Step 3: Set the file basename at session open**

In `etnga_poll_states.cpp` `TransitionToChargeHandshakeState()` open block, after the ambient capture, set the basename once the timestamp is known. Because the dir + timestamp are needed, compute it here:

```cpp
        // basename = "<dir>/<UTC timestamp>" (no extension); files are <base>.html / <base>.csv
        {
            char ts[40];
            int utc = m_charge_session.start_utc;
            if (utc > 1000000000) {
                time_t st = (time_t) utc; struct tm tmv; gmtime_r(&st, &tmv);
                strftime(ts, sizeof(ts), "%Y%m%dT%H%M%SZ", &tmv);
            } else {
                snprintf(ts, sizeof(ts), "charge-%d", m_charge_session.start_monotonic);
            }
            mkdir(ChargeReportDir().c_str(), 0755);
            m_charge_session.base = ChargeReportDir() + "/" + ts;
        }
```

Add includes to `etnga_poll_states.cpp` if missing: `<time.h>`, `<sys/stat.h>`. (`ChargeReportDir()` is a member, callable here.)

- [ ] **Step 4: Commit**

```bash
git add components/vehicle_toyota_etnga/src/etnga_charge_report.cpp components/vehicle_toyota_etnga/src/etnga_poll_states.cpp
git commit -m "etnga charge report: per-tick aggregation, delivered-Ah, fine CSV streaming, SVG buffer"
```

---

## Task 7: Inline SVG generation

**Files:**
- Modify: `components/vehicle_toyota_etnga/src/etnga_charge_report.cpp` (add a static `RenderPowerSvg`)

- [ ] **Step 1: Add the SVG renderer**

```cpp
// Render a self-contained inline SVG line chart of delivered power (+ light SOC overlay) vs time.
static std::string RenderPowerSvg(const std::vector<OvmsVehicleToyotaETNGA::ChargeSessionState::Sample>& s)
{
    if (s.size() < 2)
        return "<p>(not enough samples for a chart)</p>";
    const int W = 640, H = 240, PADL = 44, PADB = 24, PADT = 10, PADR = 10;
    int tmax = s.back().t_s > 0 ? s.back().t_s : 1;
    float kwmax = 1.0f;
    for (auto& p : s) if (p.kw > kwmax) kwmax = p.kw;
    auto X = [&](int t){ return PADL + (float)(W-PADL-PADR) * t / tmax; };
    auto Yk = [&](float kw){ return PADT + (float)(H-PADT-PADB) * (1.0f - kw / kwmax); };
    auto Ys = [&](int soc){ return PADT + (float)(H-PADT-PADB) * (1.0f - soc / 100.0f); };

    std::string out;
    char b[128];
    snprintf(b, sizeof(b), "<svg viewBox=\"0 0 %d %d\" style=\"width:100%%;max-width:%dpx;height:auto\" "
        "xmlns=\"http://www.w3.org/2000/svg\">\n", W, H, W);
    out += b;
    out += "<rect width=\"100%\" height=\"100%\" fill=\"#fff\"/>\n";
    // axes
    snprintf(b, sizeof(b), "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"#ccc\"/>\n",
        PADL, PADT, PADL, H-PADB); out += b;
    snprintf(b, sizeof(b), "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"#ccc\"/>\n",
        PADL, H-PADB, W-PADR, H-PADB); out += b;
    snprintf(b, sizeof(b), "<text x=\"4\" y=\"%d\" font-size=\"10\" fill=\"#666\">%.0f kW</text>\n",
        PADT+8, kwmax); out += b;
    // SOC overlay (light)
    out += "<polyline fill=\"none\" stroke=\"#9cf\" stroke-width=\"1\" points=\"";
    for (auto& p : s) { snprintf(b, sizeof(b), "%.1f,%.1f ", X(p.t_s), Ys(p.soc)); out += b; }
    out += "\"/>\n";
    // delivered power
    out += "<polyline fill=\"none\" stroke=\"#06c\" stroke-width=\"2\" points=\"";
    for (auto& p : s) { snprintf(b, sizeof(b), "%.1f,%.1f ", X(p.t_s), Yk(p.kw)); out += b; }
    out += "\"/>\n";
    out += "<text x=\"50\" y=\"" + std::to_string(H-6) + "\" font-size=\"10\" fill=\"#06c\">power (kW)</text>"
           "<text x=\"140\" y=\"" + std::to_string(H-6) + "\" font-size=\"10\" fill=\"#9cf\">SOC %</text>\n";
    out += "</svg>\n";
    return out;
}
```

> The `Sample` type is nested in `ChargeSessionState`; reference it as shown. If the compiler objects to the nested-type access from a free function, move `RenderPowerSvg` to a member or pass a `std::vector<std::pair<int,float>>` of (t,kw) plus a SOC vector instead. Decide at implementation; member is simplest.

- [ ] **Step 2: Commit**

```bash
git add components/vehicle_toyota_etnga/src/etnga_charge_report.cpp
git commit -m "etnga charge report: inline SVG power-vs-time renderer"
```

---

## Task 8: Rewrite GenerateChargeReport (HTML)

**Files:**
- Modify: `components/vehicle_toyota_etnga/src/etnga_charge_report.cpp` (`GenerateChargeReport`)

- [ ] **Step 1: Replace the v1 body**

```cpp
void OvmsVehicleToyotaETNGA::GenerateChargeReport()
{
    const float energy_kwh = StandardMetrics.ms_v_charge_kwh->AsFloat();
    if (energy_kwh < 0.05f || m_charge_session.base.empty()) {
        ESP_LOGD(TAG, "Charge report skipped (%.3f kWh)", energy_kwh);
        return;
    }
    const float grid_kwh  = StandardMetrics.ms_v_charge_kwh_grid->AsFloat();
    const int   end_soc   = (int) StandardMetrics.ms_v_bat_soc->AsFloat();
    const int   start_soc = m_charge_session.start_soc;
    int dur = StandardMetrics.ms_m_monotonic->AsInt() - m_charge_session.start_monotonic;
    if (dur < 0) dur = 0;
    const float avg_kw = (dur > 0) ? energy_kwh / (dur / 3600.0f) : 0.0f;
    const int   outcome = m_v_charge_outcome->AsInt();
    const bool  time_ok = (m_charge_session.start_utc > 1000000000);

    char sbuf[40] = "(clock not synced)", ebuf[40] = "(clock not synced)";
    if (time_ok) {
        time_t st = (time_t) m_charge_session.start_utc, et = st + dur; struct tm tmv;
        gmtime_r(&st, &tmv); strftime(sbuf, sizeof(sbuf), "%Y-%m-%d %H:%M:%S UTC", &tmv);
        gmtime_r(&et, &tmv); strftime(ebuf, sizeof(ebuf), "%Y-%m-%d %H:%M:%S UTC", &tmv);
    }
    int dh = dur/3600, dm = (dur%3600)/60, ds = dur%60;

    std::ofstream f(m_charge_session.base + ".html", std::ios::out | std::ios::trunc);
    if (!f) { ESP_LOGE(TAG, "Charge report: cannot write %s.html", m_charge_session.base.c_str()); return; }

    char b[96];
    f << "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">\n"
      << "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
      << "<title>Charge report " << sbuf << "</title>\n"
      << "<style>body{font:14px/1.4 system-ui,sans-serif;margin:1rem;max-width:46rem}"
      << "h1{font-size:1.3rem}h2{font-size:1.05rem;margin-top:1.3rem}"
      << "dl{display:grid;grid-template-columns:max-content 1fr;gap:.2rem .8rem}dt{font-weight:600}"
      << "table{border-collapse:collapse}td,th{border:1px solid #ddd;padding:.15rem .4rem;font-size:13px}"
      << ".est{color:#555}.note{color:#888;font-size:12px;margin-top:1.2rem}</style></head><body>\n"
      << "<h1>Charging session report</h1>\n";

    // --- Measured summary ---
    f << "<h2>Summary</h2>\n<dl>\n"
      << "<dt>Plug-in</dt><dd>" << sbuf << "</dd>\n<dt>Unplug</dt><dd>" << ebuf << "</dd>\n";
    snprintf(b, sizeof(b), "%dh %02dm %02ds", dh, dm, ds);
    f << "<dt>Duration</dt><dd>" << b << "</dd>\n";
    if (m_charge_session.has_loc) {
        snprintf(b, sizeof(b), "%.5f, %.5f", m_charge_session.start_lat, m_charge_session.start_lon);
        f << "<dt>Location</dt><dd>" << b
          << " (<a target=\"_blank\" href=\"https://www.openstreetmap.org/?mlat="
          << m_charge_session.start_lat << "&mlon=" << m_charge_session.start_lon
          << "#map=17/" << m_charge_session.start_lat << "/" << m_charge_session.start_lon << "\">map</a>)</dd>\n";
    }
    if (m_charge_session.amb_seen) {
        snprintf(b, sizeof(b), "%.0f&deg;C &rarr; %.0f&deg;C", m_charge_session.amb_min, m_charge_session.amb_max);
        f << "<dt>Ambient</dt><dd>" << b << "</dd>\n";
    }
    f << "<dt>Type</dt><dd>" << (m_charge_session.is_dc ? "DC fast" : "AC") << "</dd>\n";
    snprintf(b, sizeof(b), "%d%% &rarr; %d%% (+%d%%)", start_soc, end_soc, start_soc>=0?end_soc-start_soc:0);
    f << "<dt>SOC</dt><dd>" << b << "</dd>\n";
    snprintf(b, sizeof(b), "%.2f kWh delivered, %.2f kWh from grid", energy_kwh, grid_kwh);
    f << "<dt>Energy</dt><dd>" << b << "</dd>\n";
    snprintf(b, sizeof(b), "%.1f kW peak / %.2f kW avg", m_charge_session.peak_power, avg_kw);
    f << "<dt>Power</dt><dd>" << b << "</dd>\n";
    if (m_charge_session.temp_seen) {
        snprintf(b, sizeof(b), "%.0f&deg;C &rarr; %.0f&deg;C", m_charge_session.temp_min, m_charge_session.temp_max);
        f << "<dt>Battery temp</dt><dd>" << b << "</dd>\n";
    }
    {
        const char* lbl = ChargeOutcomeLabel(outcome);
        if (lbl[0]) f << "<dt>Outcome</dt><dd>" << lbl << "</dd>\n";
        else { snprintf(b, sizeof(b), "0x%02X", outcome & 0xFF); f << "<dt>Outcome</dt><dd>" << b << " (raw)</dd>\n"; }
    }
    f << "</dl>\n";

    // --- Chart ---
    f << "<h2>Charging power</h2>\n" << RenderPowerSvg(m_charge_session.svg);

    // --- Event log ---
    f << "<h2>Session events</h2>\n<table><tr><th>Time</th><th>Event</th></tr>\n";
    for (auto& ev : m_charge_session.events) {
        int rel = ev.first - m_charge_session.start_monotonic; if (rel < 0) rel = 0;
        snprintf(b, sizeof(b), "%d:%02d", rel/60, rel%60);
        f << "<tr><td>" << b << "</td><td>" << ev.second << "</td></tr>\n";
    }
    f << "</table>\n";

    // --- Estimates (labelled) ---
    f << "<h2 class=\"est\">Estimates</h2>\n<dl class=\"est\">\n";
    if (grid_kwh > 0.01f) {
        snprintf(b, sizeof(b), "%.0f%% (%.2f kWh loss)", energy_kwh/grid_kwh*100.0f, grid_kwh-energy_kwh);
        f << "<dt>Charging efficiency</dt><dd>" << b << "</dd>\n";
    }
    if (start_soc >= 0 && end_soc > start_soc && m_charge_session.delivered_ah > 0.5f) {
        float cap = m_charge_session.delivered_ah / ((end_soc - start_soc) / 100.0f);
        snprintf(b, sizeof(b), "%.0f Ah implied (~%.0f%% of 201 Ah nominal)", cap, cap/201.1f*100.0f);
        f << "<dt>Implied capacity</dt><dd>" << b << "</dd>\n";
    }
    f << "</dl>\n";

    // --- CSV link ---
    {
        std::string csv = m_charge_session.base + ".csv";
        std::string name = csv.substr(csv.find_last_of('/') + 1);
        f << "<p><a href=\"/xte/report?file=" << name << "\">Download per-sample CSV</a></p>\n";
    }

    f << "<p class=\"note\">Generated on-module by OVMS (Toyota e-TNGA). Single-phase; avg power is over the "
         "whole plug-in interval. Estimates are provisional.</p>\n</body></html>\n";
    f.close();

    ESP_LOGI(TAG, "Charge report written: %s.html (%.2f kWh, %d%%->%d%%)",
        m_charge_session.base.c_str(), energy_kwh, start_soc, end_soc);
    PruneChargeReports(TAG);
}
```

- [ ] **Step 2: Commit**

```bash
git add components/vehicle_toyota_etnga/src/etnga_charge_report.cpp
git commit -m "etnga charge report: v2 HTML (summary+location+ambient+outcome, SVG, event log, estimates, CSV link)"
```

---

## Task 9: Retention — prune by session (html+csv pairs) in the resolved dir

**Files:**
- Modify: `components/vehicle_toyota_etnga/src/etnga_charge_report.cpp` (`PruneChargeReports`)

- [ ] **Step 1: Update PruneChargeReports to use the resolved dir and delete pairs**

```cpp
static void PruneChargeReports(const char* tag, const std::string& dir)
{
    DIR* d = opendir(dir.c_str());
    if (!d) return;
    std::vector<std::string> stems;   // basenames without extension
    struct dirent* e;
    while ((e = readdir(d)) != NULL) {
        std::string n = e->d_name;
        if (n.size() > 5 && n.compare(n.size()-5, 5, ".html") == 0)
            stems.push_back(n.substr(0, n.size()-5));
    }
    closedir(d);
    if ((int)stems.size() <= CHARGE_REPORT_MAX) return;
    std::sort(stems.begin(), stems.end());
    int del = (int)stems.size() - CHARGE_REPORT_MAX;
    for (int i = 0; i < del; i++) {
        unlink((dir + "/" + stems[i] + ".html").c_str());
        unlink((dir + "/" + stems[i] + ".csv").c_str());
        ESP_LOGD(tag, "Charge report pruned: %s", stems[i].c_str());
    }
}
```

Update the call in `GenerateChargeReport()` from `PruneChargeReports(TAG);` to `PruneChargeReports(TAG, ChargeReportDir());`.

- [ ] **Step 2: Commit**

```bash
git add components/vehicle_toyota_etnga/src/etnga_charge_report.cpp
git commit -m "etnga charge report: retention prunes html+csv pairs in the resolved dir (keep newest 50)"
```

---

## Task 10: Web viewer — serve CSV (needs #88 base)

**Files:**
- Modify: `components/vehicle_toyota_etnga/src/etnga_web.cpp` (`WebChargeReport`, `WebChargeReports`)

> Prerequisite: rebase this branch onto a base containing #88 (`etnga_web.cpp`). Update the hard-coded `CHARGE_REPORT_DIR` in `etnga_web.cpp` to resolve `/sd` first, matching `ChargeReportDir()` (duplicate the small resolver, or share via a header constant).

- [ ] **Step 1: Serve .csv with the right content type**

In `WebChargeReport`, accept `.csv` as well as `.html`, and pick the content type:

```cpp
    bool is_html = (file.size() > 5 && file.compare(file.size()-5, 5, ".html") == 0);
    bool is_csv  = (file.size() > 4 && file.compare(file.size()-4, 4, ".csv")  == 0);
    bool valid = (is_html || is_csv)
                 && file.find('/') == std::string::npos
                 && file.find("..") == std::string::npos;
    if (!valid) { c.head(400, "Content-Type: text/plain; charset=utf-8"); c.print("Invalid report name\n"); c.done(); return; }
    // ... load_file(dir + "/" + file, content) ...
    if (is_csv)
        c.head(200, "Content-Type: text/csv; charset=utf-8\r\nContent-Disposition: attachment");
    else
        c.head(200, "Content-Type: text/html; charset=utf-8\r\nCache-Control: no-cache");
    c.print(content);
    c.done();
```

- [ ] **Step 2: In WebChargeReports, list the CSV link next to each report**

For each `.html` stem, also emit a `(csv)` link to `/xte/report?file=<stem>.csv`.

- [ ] **Step 3: Commit**

```bash
git add components/vehicle_toyota_etnga/src/etnga_web.cpp
git commit -m "etnga web: serve charge-report CSV (text/csv) and link it from the reports index"
```

---

## Task 11: changes.txt + CI build gate

**Files:**
- Modify: `changes.txt`

- [ ] **Step 1: Add a changelog entry** under the unreleased section:

```
- Toyota e-TNGA (Subaru Solterra / Toyota bZ4X): charge report v2 — each session now writes an
    HTML report (location, ambient, human-readable outcome, inline power-vs-time chart, event log,
    efficiency/implied-capacity estimates) plus a fine-grained per-sample CSV (offered/allowed/
    delivered power) to /sd/charge-reports/ (fallback /store). Fixes peak charging power on DC.
```

- [ ] **Step 2: Push and open a PR to trigger the CI firmware build**

```bash
git push -u origin feature/etnga-charge-report-v2
gh pr create --base master --head feature/etnga-charge-report-v2 \
  --title "Toyota e-TNGA: charge report v2 (SVG chart, fine CSV, peak-power fix)" \
  --body "Implements docs/superpowers/specs/2026-06-05-etnga-charge-report-v2-design.md. See spec for scope."
```

Expected: the `Firmware build` check passes. If it fails, fix compile errors (most likely: nested-`Sample`-type access in Task 7 — move `RenderPowerSvg` to a member; or a missing include) and push again.

---

## Task 12: On-vehicle validation

- [ ] Do an **AC** charge: confirm `/sd/charge-reports/<ts>.html` + `.csv` appear; report opens and renders the SVG; CSV opens in a spreadsheet with offered/allowed/delivered columns populated.
- [ ] Do a **DC** charge: confirm **peak power is now correct** (non-zero, matches observed kW) — the primary bug fix.
- [ ] Confirm location + ambient + human-readable outcome are correct; event log shows the handshake→charge→unplug sequence with timestamps.
- [ ] Copy a report off-module and confirm the **SVG still renders** (self-contained).
- [ ] Plug-in-then-unplug (no charge) → no files written. After 50 sessions, oldest pairs pruned.
- [ ] `/xte/reports` lists each session with working report + CSV links.

---

## Self-Review notes

- **Spec coverage:** power-fix (T1), location/ambient (T2/T4/T6), event log (T5), CSV fine + offered/allowed/delivered (T6), SVG (T7), HTML two-tier + outcome label + estimates + CSV link (T3/T8), /sd-fallback + retention (T3/T9), web CSV serving (T10), changes.txt + CI (T11), on-vehicle (T12). Multi-phase / sleep-survival / error-dump explicitly out of scope (spec).
- **Open implementation decisions flagged inline:** full 0x1688 table (T3), `grid_kw` live metric vs drop column (T6), nested-Sample access for the SVG free function (T7), `/sd` resolver duplication in etnga_web.cpp (T10).
- **Type consistency:** `ChargeSessionState::Sample{int t_s; float kw; int soc;}` used in T6/T7/T8; `ChargeReportDir()`/`ChargeOutcomeLabel()`/`LogChargeEvent()`/`AppendChargeCsvRow()` declared T2, defined T3/T5/T6.
