# e-TNGA Charging Monitor (AC/DC views + live chart + state history) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rework the e-TNGA "Charging monitor" web page (`/xte/charge`) into auto-selected AC vs DC vs idle views with a live Highcharts power/SOC chart and a live "charging state history", and add AC-Op (0x1684) transition logging so AC sessions get a state history in the report too (DC already has HLC 0x1666 from #95).

**Architecture:** All work is in the e-TNGA component. The static page handler reads the active vehicle instance (`MyVehicleFactory.ActiveVehicle()` cast to `OvmsVehicleToyotaETNGA*`) and, at render time, embeds the in-memory session sample buffer (`m_charge_session.svg`) and event log (`m_charge_session.events`) as JS literals (no new HTTP route). Highcharts (bundled, lazy-loaded) draws the backfilled curve, then appends points/rows live on the framework's `msg:metrics` WebSocket event.

**Tech Stack:** C++ (ESP-IDF 3.3, GCC), OVMS web framework (`PageContext_t`, `data-metric` widgets, `msg:metrics`), Highcharts 6.0.7, jQuery 1.12.4.

**Testing reality:** No host unit tests exist; firmware builds only on GitHub CI. Each task is a self-contained compilable increment + commit. The build gate is CI after the code tasks (Task 6); behavior is validated on-vehicle after deploy (Task 7).

**Worktree:** `/home/devuser/wt-etnga-report-fixes`, branch `feature/etnga-charge-report-fixes` (off `origin/master`). All paths below are relative to that worktree.

---

## File Structure

- `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h`
  — add `last_acop` to `ChargeSessionState`; declare `AcOpStatusLabel` and the four new static web helpers.
- `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_charge_report.cpp`
  — add `AcOpStatusLabel(int)` decoder (next to `HlcStateLabel`).
- `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_metrics.cpp`
  — `SetAcOpStatus`: log AC-Op transitions as session events (mirror of `SetHlcState`).
- `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_web.cpp`
  — restructure `WebDispChgMetrics` into a dispatcher; add `WebChgRenderAc`, `WebChgRenderDc`,
    `WebChgChartJs`, `WebChgStateHistoryJs`.
- `vehicle/OVMS.V3/changes.txt` — user-facing entry.

---

## Task 1: AC-Op (0x1684) state decoder + transition logging

Gives AC sessions a state history in the report (symmetric with DC's HLC), and provides the label
map the monitor reuses.

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h`
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_charge_report.cpp`
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_metrics.cpp`

- [ ] **Step 1: Add `last_acop` field to `ChargeSessionState`**

In `vehicle_toyota_etnga.h`, find:

```cpp
        int   last_hlc = -1;               // last 0x1666 HLC state logged as an event (change detection)
```

Add immediately after it:

```cpp
        int   last_acop = -1;              // last 0x1684 AC-Op state logged as an event (change detection)
```

- [ ] **Step 2: Declare `AcOpStatusLabel` in the header**

In `vehicle_toyota_etnga.h`, find:

```cpp
    static const char* HlcStateLabel(int code);         // 0x1666 DC HLC state enum -> human text ("" if unknown)
```

Add immediately after it:

```cpp
    static const char* AcOpStatusLabel(int code);       // 0x1684 AC-Op state enum -> human text ("" for Stop/unknown)
```

- [ ] **Step 3: Implement `AcOpStatusLabel` in `etnga_charge_report.cpp`**

Find the end of `HlcStateLabel` (the closing brace after the `0xFF`/`default` switch). Immediately
after that function's closing `}`, add:

```cpp
// Map a 0x1684 "AC Charging Operation Status" enum (confirmed 4-state: Stop/Startup/Running/
// Finishing, per solterra-can) to a human-readable label. Returns "" for Stop(0)/unknown so the
// caller skips it — that keeps DC sessions (where AC-Op stays Stop) free of spurious AC entries;
// the final stop is already covered by the outcome / "Charging paused" / "Unplugged" events.
const char* OvmsVehicleToyotaETNGA::AcOpStatusLabel(int code)
{
    switch (code & 0xFF) {
        case 0x01: return "AC: Startup";
        case 0x02: return "AC: Running";
        case 0x03: return "AC: Finishing";
        default:   return "";   // 0x00 Stop / unknown -> skipped
    }
}
```

- [ ] **Step 4: Log AC-Op transitions in `SetAcOpStatus`**

In `etnga_metrics.cpp`, find:

```cpp
void OvmsVehicleToyotaETNGA::SetAcOpStatus(int v) { m_v_charge_ac_op->SetValue(v); }
```

Replace it with:

```cpp
void OvmsVehicleToyotaETNGA::SetAcOpStatus(int v)
{
    m_v_charge_ac_op->SetValue(v);
    // Log each new 0x1684 AC-Op state as a session event (mirrors SetHlcState for DC).
    // Stop/unknown return "" from AcOpStatusLabel and are skipped.
    if (m_charge_session.in_session && v != m_charge_session.last_acop) {
        m_charge_session.last_acop = v;
        const char* lbl = AcOpStatusLabel(v);
        if (lbl && lbl[0])
            LogChargeEvent(lbl);
    }
}
```

- [ ] **Step 5: Commit**

```bash
cd /home/devuser/wt-etnga-report-fixes
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_charge_report.cpp \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_metrics.cpp
git commit -m "etnga: log 0x1684 AC-Op state transitions to the charge session event log"
```

---

## Task 2: Restructure `WebDispChgMetrics` into AC/DC/idle dispatcher + metric panels

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h`
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_web.cpp`

- [ ] **Step 1: Declare the new static web helpers**

In `vehicle_toyota_etnga.h`, find:

```cpp
    static void WebDispChgMetrics(PageEntry_t& p, PageContext_t& c);
```

Add immediately after it:

```cpp
    static void WebChgRenderAc(PageContext_t& c, OvmsVehicleToyotaETNGA* v);   // AC charging panels
    static void WebChgRenderDc(PageContext_t& c, OvmsVehicleToyotaETNGA* v);   // DC charging panels
    static void WebChgChartJs(PageContext_t& c, OvmsVehicleToyotaETNGA* v, bool dc);          // live chart
    static void WebChgStateHistoryJs(PageContext_t& c, OvmsVehicleToyotaETNGA* v, bool dc);   // live state history
```

- [ ] **Step 2: Replace `WebDispChgMetrics` with the dispatcher + idle view**

In `etnga_web.cpp`, replace the entire current `WebDispChgMetrics` function (from its leading comment
`// WebDispChgMetrics: live charging dashboard...` through its closing `}`) with:

```cpp
// WebDispChgMetrics: live charging monitor. Auto-selects an AC or DC layout from the active
// session (idle when not charging). Metric panels auto-update via the framework data-metric
// mechanism; the chart + state history are seeded from the in-memory session buffers and updated
// live on msg:metrics. The whole body is wrapped in a ".receiver" element (id "chgmon") so the
// framework delivers metric updates to it.
void OvmsVehicleToyotaETNGA::WebDispChgMetrics(PageEntry_t& p, PageContext_t& c)
{
    OvmsVehicleToyotaETNGA* v = (OvmsVehicleToyotaETNGA*) MyVehicleFactory.ActiveVehicle();
    bool in_session = (v && v->m_charge_session.in_session);
    bool dc = (in_session && v->m_charge_session.is_dc);

    c.head(200);
    PAGE_HOOK("body.pre");

    c.print(
        "<style>\n"
        "h6.metric-head { margin-bottom: 0; color: #676767; font-size: 15px; }\n"
        ".night h6.metric-head { color: unset; }\n"
        "#chghist td, #chghist th { padding: .1rem .5rem; font-size: 13px; }\n"
        "</style>\n");

    if (!in_session) {
        c.print(
            "<div class=\"panel panel-primary\">"
              "<div class=\"panel-heading\">");
        c.print(etnga_vehicle_name() + " charging monitor");
        c.print(
              "</div>"
              "<div class=\"panel-body\">"
                "<p>No active charge session.</p>"
                "<p><a class=\"btn btn-default\" href=\"/xte/reports\">View saved charge reports</a></p>"
              "</div>"
            "</div>");
        PAGE_HOOK("body.post");
        c.done();
        return;
    }

    c.print(
        "<div class=\"panel panel-primary receiver\" id=\"chgmon\">"
          "<div class=\"panel-heading\">");
    c.print(etnga_vehicle_name() + (dc ? " — DC fast charging" : " — AC charging"));
    c.print(
          "</div>"
          "<div class=\"panel-body\">");

    if (dc) WebChgRenderDc(c, v);
    else    WebChgRenderAc(c, v);

    WebChgChartJs(c, v, dc);
    WebChgStateHistoryJs(c, v, dc);

    c.print(
          "</div>"
        "</div>");

    PAGE_HOOK("body.post");
    c.done();
}
```

- [ ] **Step 3: Add the AC panel renderer**

In `etnga_web.cpp`, immediately after the `WebDispChgMetrics` function you just wrote, add:

```cpp
// AC charging metric panels (data-metric widgets auto-update). No station-max (DC-only PID).
void OvmsVehicleToyotaETNGA::WebChgRenderAc(PageContext_t& c, OvmsVehicleToyotaETNGA* v)
{
    c.print(
        "<div class=\"clearfix\">"
          "<div class=\"metric progress\" data-metric=\"v.b.soc\" data-prec=\"1\">"
            "<div class=\"progress-bar value-low text-left\" role=\"progressbar\" aria-valuenow=\"0\" aria-valuemin=\"0\" aria-valuemax=\"100\" style=\"width:0%\">"
              "<div><span class=\"label\">SoC</span><span class=\"value\">?</span><span class=\"unit\">%</span></div>"
            "</div>"
          "</div>"
        "</div>"

        "<div class=\"clearfix\">"
          "<h6 class=\"metric-head\">Charger (AC)</h6>"
          "<div class=\"metric text\" data-metric=\"v.c.state\"><span class=\"label\">State</span><span class=\"value\">?</span></div>"
          "<div class=\"metric number\" data-metric=\"v.c.power\" data-prec=\"3\"><span class=\"label\">Delivered</span><span class=\"value\">?</span><span class=\"unit\">kW</span></div>"
          "<div class=\"metric number\" data-metric=\"xte.v.c.gridpower\" data-prec=\"3\"><span class=\"label\">Grid input</span><span class=\"value\">?</span><span class=\"unit\">kW</span></div>"
          "<div class=\"metric number\" data-metric=\"v.c.voltage\" data-prec=\"1\"><span class=\"label\">Voltage</span><span class=\"value\">?</span><span class=\"unit\">V</span></div>"
          "<div class=\"metric number\" data-metric=\"v.c.current\" data-prec=\"1\"><span class=\"label\">Current</span><span class=\"value\">?</span><span class=\"unit\">A</span></div>"
        "</div>"

        "<div class=\"clearfix\">"
          "<h6 class=\"metric-head\">Battery</h6>"
          "<div class=\"metric number\" data-metric=\"v.b.power\" data-prec=\"3\"><span class=\"label\">Power</span><span class=\"value\">?</span><span class=\"unit\">kW</span></div>"
          "<div class=\"metric number\" data-metric=\"v.b.voltage\" data-prec=\"1\"><span class=\"label\">Voltage</span><span class=\"value\">?</span><span class=\"unit\">V</span></div>"
          "<div class=\"metric number\" data-metric=\"v.b.current\" data-prec=\"1\"><span class=\"label\">Current</span><span class=\"value\">?</span><span class=\"unit\">A</span></div>"
          "<div class=\"metric number\" data-metric=\"v.b.temp\" data-prec=\"1\"><span class=\"label\">Temp</span><span class=\"value\">?</span><span class=\"unit\">&deg;C</span></div>"
        "</div>"

        "<div class=\"clearfix\">"
          "<h6 class=\"metric-head\">Session energy</h6>"
          "<div class=\"metric number\" data-metric=\"v.c.kwh\" data-prec=\"2\"><span class=\"label\">Charged</span><span class=\"value\">?</span><span class=\"unit\">kWh</span></div>"
          "<div class=\"metric number\" data-metric=\"v.c.kwh.grid\" data-prec=\"2\"><span class=\"label\">From grid</span><span class=\"value\">?</span><span class=\"unit\">kWh</span></div>"
          "<div class=\"metric number\" data-metric=\"xte.v.e.hvac.kwh\" data-prec=\"3\"><span class=\"label\">Cabin (My Room)</span><span class=\"value\">?</span><span class=\"unit\">kWh</span></div>"
        "</div>");
}
```

- [ ] **Step 4: Add the DC panel renderer**

Immediately after `WebChgRenderAc`, add:

```cpp
// DC fast-charging metric panels. Adds station + car-permitted; live HLC state shown as text
// (decoded client-side in WebChgStateHistoryJs's label map via a dedicated span updated there).
void OvmsVehicleToyotaETNGA::WebChgRenderDc(PageContext_t& c, OvmsVehicleToyotaETNGA* v)
{
    c.print(
        "<div class=\"clearfix\">"
          "<div class=\"metric progress\" data-metric=\"v.b.soc\" data-prec=\"1\">"
            "<div class=\"progress-bar value-low text-left\" role=\"progressbar\" aria-valuenow=\"0\" aria-valuemin=\"0\" aria-valuemax=\"100\" style=\"width:0%\">"
              "<div><span class=\"label\">SoC</span><span class=\"value\">?</span><span class=\"unit\">%</span></div>"
            "</div>"
          "</div>"
        "</div>"

        "<div class=\"clearfix\">"
          "<h6 class=\"metric-head\">Handshake</h6>"
          "<div class=\"metric text\"><span class=\"label\">HLC state</span><span class=\"value\" id=\"hlcstate\">?</span></div>"
        "</div>"

        "<div class=\"clearfix\">"
          "<h6 class=\"metric-head\">Station</h6>"
          "<div class=\"metric number\" data-metric=\"xte.v.c.stamaxp\" data-prec=\"1\"><span class=\"label\">Max power</span><span class=\"value\">?</span><span class=\"unit\">kW</span></div>"
          "<div class=\"metric number\" data-metric=\"xte.v.c.stamaxv\" data-prec=\"0\"><span class=\"label\">Max V</span><span class=\"value\">?</span><span class=\"unit\">V</span></div>"
          "<div class=\"metric number\" data-metric=\"xte.v.c.stamaxi\" data-prec=\"0\"><span class=\"label\">Max A</span><span class=\"value\">?</span><span class=\"unit\">A</span></div>"
          "<div class=\"metric number\" data-metric=\"v.c.voltage\" data-prec=\"1\"><span class=\"label\">Present V</span><span class=\"value\">?</span><span class=\"unit\">V</span></div>"
          "<div class=\"metric number\" data-metric=\"v.c.current\" data-prec=\"1\"><span class=\"label\">Present A</span><span class=\"value\">?</span><span class=\"unit\">A</span></div>"
        "</div>"

        "<div class=\"clearfix\">"
          "<h6 class=\"metric-head\">Car</h6>"
          "<div class=\"metric number\" data-metric=\"v.c.power\" data-prec=\"3\"><span class=\"label\">Delivered</span><span class=\"value\">?</span><span class=\"unit\">kW</span></div>"
          "<div class=\"metric number\" data-metric=\"xte.v.c.perm\" data-prec=\"2\"><span class=\"label\">Permitted</span><span class=\"value\">?</span><span class=\"unit\">kW</span></div>"
          "<div class=\"metric number\" data-metric=\"xte.v.c.tgti\" data-prec=\"0\"><span class=\"label\">Target</span><span class=\"value\">?</span><span class=\"unit\">A</span></div>"
        "</div>"

        "<div class=\"clearfix\">"
          "<h6 class=\"metric-head\">Battery</h6>"
          "<div class=\"metric number\" data-metric=\"v.b.power\" data-prec=\"3\"><span class=\"label\">Power</span><span class=\"value\">?</span><span class=\"unit\">kW</span></div>"
          "<div class=\"metric number\" data-metric=\"v.b.voltage\" data-prec=\"1\"><span class=\"label\">Voltage</span><span class=\"value\">?</span><span class=\"unit\">V</span></div>"
          "<div class=\"metric number\" data-metric=\"v.b.current\" data-prec=\"1\"><span class=\"label\">Current</span><span class=\"value\">?</span><span class=\"unit\">A</span></div>"
          "<div class=\"metric number\" data-metric=\"v.b.temp\" data-prec=\"1\"><span class=\"label\">Temp</span><span class=\"value\">?</span><span class=\"unit\">&deg;C</span></div>"
        "</div>"

        "<div class=\"clearfix\">"
          "<h6 class=\"metric-head\">Session energy</h6>"
          "<div class=\"metric number\" data-metric=\"v.c.kwh\" data-prec=\"2\"><span class=\"label\">Charged</span><span class=\"value\">?</span><span class=\"unit\">kWh</span></div>"
        "</div>");
}
```

- [ ] **Step 5: Add temporary empty stubs for the JS helpers (so it compiles before Tasks 3–4)**

Immediately after `WebChgRenderDc`, add temporary stubs (replaced in Tasks 3 and 4):

```cpp
void OvmsVehicleToyotaETNGA::WebChgChartJs(PageContext_t& c, OvmsVehicleToyotaETNGA* v, bool dc) {}
void OvmsVehicleToyotaETNGA::WebChgStateHistoryJs(PageContext_t& c, OvmsVehicleToyotaETNGA* v, bool dc) {}
```

- [ ] **Step 6: Commit**

```bash
cd /home/devuser/wt-etnga-report-fixes
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/vehicle_toyota_etnga.h \
        vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_web.cpp
git commit -m "etnga web: AC/DC/idle charging-monitor dispatcher + per-type metric panels"
```

---

## Task 3: Live chart (`WebChgChartJs`)

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_web.cpp`

- [ ] **Step 1: Replace the `WebChgChartJs` stub with the real implementation**

Replace:

```cpp
void OvmsVehicleToyotaETNGA::WebChgChartJs(PageContext_t& c, OvmsVehicleToyotaETNGA* v, bool dc) {}
```

with:

```cpp
// Emit the chart container, an inline JS literal backfill of the in-memory sample buffer, and the
// Highcharts init + live-append wiring. Series mirror the report: delivered power, car-permitted
// (|0x16A1|), station-max (DC only), and SOC on a fixed 0-100 right axis. Power axis fixed per type.
void OvmsVehicleToyotaETNGA::WebChgChartJs(PageContext_t& c, OvmsVehicleToyotaETNGA* v, bool dc)
{
    // Build the backfill literal from the session sample buffer: [t_s, kw, soc, sta_max, car_perm].
    std::string init = "[";
    const std::vector<ChargeSessionState::Sample>& s = v->m_charge_session.svg;
    char b[96];
    for (size_t i = 0; i < s.size(); i++) {
        snprintf(b, sizeof(b), "%s[%d,%.3f,%d,%.2f,%.2f]",
                 (i ? "," : ""), s[i].t_s, s[i].kw, s[i].soc, s[i].sta_max, s[i].car_perm);
        init += b;
    }
    init += "]";

    int elapsed = StandardMetrics.ms_m_monotonic->AsInt() - v->m_charge_session.start_monotonic;
    if (elapsed < 0) elapsed = 0;

    c.print("<h6 class=\"metric-head\">Power &amp; SOC</h6>\n"
            "<div id=\"chgchart\" style=\"width:100%;height:300px\"></div>\n");

    c.printf(
        "<script>\n"
        "(function(){\n"
        "  var DC=%d, PMAX=DC?150:11;\n"
        "  var INIT=%s;\n"
        "  var loadT=Date.now()/1000, baseT=%d;\n"
        "  var deliv=[],perm=[],sta=[],soc=[];\n"
        "  for(var i=0;i<INIT.length;i++){var t=INIT[i][0]/60;\n"
        "    deliv.push([t,INIT[i][1]]); soc.push([t,INIT[i][2]]);\n"
        "    if(DC) sta.push([t,INIT[i][3]]); perm.push([t,Math.abs(INIT[i][4])]);}\n"
        "  var chart=null;\n"
        "  function mv(m){var x=metrics[m]; return (x==null)?null:parseFloat(x);}\n"
        "  function build(){\n"
        "    var series=[{name:'Delivered kW',data:deliv,color:'#0066cc',zIndex:3},\n"
        "                {name:'Car permitted',data:perm,color:'#00aa00',dashStyle:'ShortDash'}];\n"
        "    if(DC) series.push({name:'Station max',data:sta,color:'#ee8800',dashStyle:'ShortDash'});\n"
        "    series.push({name:'SOC %%',data:soc,color:'#3399cc',yAxis:1});\n"
        "    chart=Highcharts.chart('chgchart',{\n"
        "      chart:{type:'line',animation:false},title:{text:null},credits:{enabled:false},\n"
        "      xAxis:{title:{text:'minutes'}},\n"
        "      yAxis:[{title:{text:'kW'},min:0,max:PMAX},\n"
        "             {title:{text:'SOC %%'},min:0,max:100,opposite:true}],\n"
        "      tooltip:{shared:true,valueDecimals:1},\n"
        "      plotOptions:{series:{marker:{enabled:false}}},\n"
        "      series:series});\n"
        "  }\n"
        "  function onUpd(){\n"
        "    if(!chart) return;\n"
        "    var t=baseT/60+(Date.now()/1000-loadT)/60;\n"
        "    var d=mv('v.c.power'),sc=mv('v.b.soc'),pm=mv('xte.v.c.perm'),sm=mv('xte.v.c.stamaxp');\n"
        "    var cap=600, sidx=DC?3:2;\n"
        "    if(d!=null) chart.series[0].addPoint([t,d],false,chart.series[0].data.length>cap);\n"
        "    chart.series[1].addPoint([t,pm==null?0:Math.abs(pm)],false,chart.series[1].data.length>cap);\n"
        "    if(DC) chart.series[2].addPoint([t,sm==null?0:sm],false,chart.series[2].data.length>cap);\n"
        "    chart.series[sidx].addPoint([t,sc],true,chart.series[sidx].data.length>cap);\n"
        "  }\n"
        "  function init(){build(); $('#chgmon').on('msg:metrics',onUpd);}\n"
        "  if(window.Highcharts) init();\n"
        "  else $.ajax({url:window.assets.charts_js,dataType:'script',cache:true,success:init});\n"
        "})();\n"
        "</script>\n",
        dc ? 1 : 0, init.c_str(), elapsed);
}
```

- [ ] **Step 2: Commit**

```bash
cd /home/devuser/wt-etnga-report-fixes
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_web.cpp
git commit -m "etnga web: live Highcharts power/SOC chart on the charging monitor (backfill + live append)"
```

Note for the implementer: `c.printf` format uses `%%` for a literal `%` (the SOC axis titles and the
`SOC %` series name). The `%d`/`%s`/`%d` args are `dc`, `init`, `elapsed` in that order.

---

## Task 4: Live charging state history (`WebChgStateHistoryJs`)

**Files:**
- Modify: `vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_web.cpp`

- [ ] **Step 1: Replace the `WebChgStateHistoryJs` stub with the real implementation**

Replace:

```cpp
void OvmsVehicleToyotaETNGA::WebChgStateHistoryJs(PageContext_t& c, OvmsVehicleToyotaETNGA* v, bool dc) {}
```

with:

```cpp
// Emit the state-history table, a backfill literal of the in-memory event log, and JS that seeds
// the table then appends a row whenever the watched state metric changes (xte.v.c.hlc on DC,
// xte.v.c.acop on AC). On DC it also keeps the #hlcstate header span current. Labels come from
// inline maps so no extra string metric is needed.
void OvmsVehicleToyotaETNGA::WebChgStateHistoryJs(PageContext_t& c, OvmsVehicleToyotaETNGA* v, bool dc)
{
    // Backfill literal from the event log: [relative_seconds, "label"].
    std::string evt = "[";
    const std::vector<std::pair<int,const char*>>& ev = v->m_charge_session.events;
    int start = v->m_charge_session.start_monotonic;
    char b[160];
    for (size_t i = 0; i < ev.size(); i++) {
        int rel = ev[i].first - start; if (rel < 0) rel = 0;
        snprintf(b, sizeof(b), "%s[%d,\"%s\"]", (i ? "," : ""), rel, ev[i].second);
        evt += b;
    }
    evt += "]";

    int elapsed = StandardMetrics.ms_m_monotonic->AsInt() - start;
    if (elapsed < 0) elapsed = 0;
    int curstate = dc ? v->m_v_charge_hlc->AsInt() : v->m_v_charge_ac_op->AsInt();

    c.print(
        "<h6 class=\"metric-head\">Charging state history</h6>\n"
        "<table id=\"chghist\" class=\"table table-condensed\"><thead><tr><th>Time</th><th>State</th></tr></thead><tbody></tbody></table>\n");

    c.printf(
        "<script>\n"
        "(function(){\n"
        "  var DC=%d;\n"
        "  var EVT=%s;\n"
        "  var loadT=Date.now()/1000, baseT=%d, lastState=%d;\n"
        "  var HLC={0:'SLAC',1:'SDP',2:'App-protocol negotiation',3:'Session setup',4:'Service discovery',"
        "5:'Service detail',6:'Payment service selection',7:'Certificate installation',8:'Certificate update',"
        "9:'Payment details',10:'Authorization',11:'Charge-parameter discovery',12:'Cable check',13:'Precharge',"
        "14:'Power delivery (start)',15:'Current demand',16:'Power delivery (stop)',17:'Welding detection',"
        "18:'Session stop',255:'Unconnected'};\n"
        "  var ACOP={1:'Startup',2:'Running',3:'Finishing'};\n"
        "  function fmt(sec){var m=Math.floor(sec/60),s=Math.floor(sec%%60);return m+':'+(s<10?'0':'')+s;}\n"
        "  function row(sec,label){$('#chghist tbody').append('<tr><td>'+fmt(sec)+'</td><td>'+label+'</td></tr>');}\n"
        "  for(var i=0;i<EVT.length;i++) row(EVT[i][0],EVT[i][1]);\n"
        "  if(DC){var hn=HLC[lastState]; if(hn) $('#hlcstate').text(hn);}\n"
        "  function onUpd(){\n"
        "    var m=DC?metrics['xte.v.c.hlc']:metrics['xte.v.c.acop'];\n"
        "    if(m==null) return; m=parseInt(m);\n"
        "    if(DC){var hn=HLC[m]; if(hn) $('#hlcstate').text(hn);}\n"
        "    if(m===lastState) return; lastState=m;\n"
        "    var label=DC?('HLC: '+(HLC[m]||('state 0x'+m.toString(16)))):(ACOP[m]?('AC: '+ACOP[m]):null);\n"
        "    if(label==null) return;\n"
        "    row(baseT+(Date.now()/1000-loadT),label);\n"
        "  }\n"
        "  $('#chgmon').on('msg:metrics',onUpd);\n"
        "})();\n"
        "</script>\n",
        dc ? 1 : 0, evt.c_str(), elapsed, curstate);
}
```

- [ ] **Step 2: Commit**

```bash
cd /home/devuser/wt-etnga-report-fixes
git add vehicle/OVMS.V3/components/vehicle_toyota_etnga/src/etnga_web.cpp
git commit -m "etnga web: live charging state history (HLC on DC / AC-Op on AC) with backfill"
```

Note for the implementer: in `c.printf`, every literal `%` must be doubled — `%%60` (modulo) and the
`%%` are intentional. Format args are `dc`, `evt`, `elapsed`, `curstate` in order.

---

## Task 5: changes.txt entry

**Files:**
- Modify: `vehicle/OVMS.V3/changes.txt`

- [ ] **Step 1: Add the entry under the pending OTA-release section**

In `vehicle/OVMS.V3/changes.txt`, find the line beginning:

```
- Toyota e-TNGA (Subaru Solterra / Toyota bZ4X): charge report refinements
```

Immediately BEFORE that line, insert:

```
- Toyota e-TNGA (Subaru Solterra / Toyota bZ4X): the Charging monitor web page now shows distinct
    AC and DC layouts (auto-selected from the active session), a live power/SOC chart that backfills
    the whole session and updates in real time (delivered power, car-permitted limit, and — on DC —
    station-offered max, matching the report chart), and a live "charging state history" of the DC
    HLC handshake states (0x1666) or the AC operation states (0x1684). The AC operation-state
    transitions are also recorded in the saved charge report's event log (DC HLC states already were).
```

- [ ] **Step 2: Commit**

```bash
cd /home/devuser/wt-etnga-report-fixes
git add vehicle/OVMS.V3/changes.txt
git commit -m "changes: e-TNGA charging-monitor AC/DC views + live chart + state history"
```

---

## Task 6: Build verification (GitHub CI)

There is no local build. Push the branch and let CI compile it; treat green CI as the compile gate.

- [ ] **Step 1: Push the branch**

```bash
cd /home/devuser/wt-etnga-report-fixes
git push origin feature/etnga-charge-report-fixes
```

- [ ] **Step 2: Watch the CI run to completion**

```bash
runid=$(gh run list --repo kezarjg/Open-Vehicle-Monitoring-System-3 --branch feature/etnga-charge-report-fixes --limit 1 --json databaseId -q '.[0].databaseId')
gh run watch "$runid" --repo kezarjg/Open-Vehicle-Monitoring-System-3 --exit-status --interval 20
```

Expected: exit code 0 (`conclusion: success`). If the build fails, read the log
(`gh run view "$runid" --log-failed`), fix the C++/format-string error, commit, and re-push.

Common failure to check first: `c.printf` percent-escaping (every literal `%` must be `%%`), and the
`window.assets.charts_js` reference (must be the global set by the framework, not a macro).

---

## Task 7: On-vehicle validation (after merge + deploy)

Validation requires a real charge. Merge and deploy using the established flow, then validate on the
next sessions.

- [ ] **Step 1: Open a PR (if not already), merge on green CI, build master, deploy**

Use the same procedure as PR #94/#95: `gh pr create`/`gh pr merge --merge`; watch the master CI run;
`gh run download <master-run> -n ovms3-firmware`; stage on os-k3s; `ota flash http`; `module reset`;
poll `ota status` until the new commit is `Running partition`. (See memory `howto_ovms_fast_deploy`.)

- [ ] **Step 2: Validate on a DC fast charge**

Open `/xte/charge` during a DC session. Confirm: DC layout (Station + Car panels, no grid); the chart
backfills the session so far and then extends live; power axis tops at 150; station-max + car-permitted
dashed traces and the SOC line render; the "Charging state history" lists HLC states and grows live;
the `HLC state` header span tracks the current state. After unplug, the saved report shows the same
HLC states in its event log.

- [ ] **Step 3: Validate on an AC charge**

Open `/xte/charge` during an AC session. Confirm: AC layout (Charger/grid + efficiency-relevant
energy panels, no station-max); chart shows delivered + car-permitted + SOC with the power axis topping
at 11; the state history lists `AC: Startup/Running/Finishing` live. After unplug, the saved report's
event log contains the `AC: …` transitions.

- [ ] **Step 4: Validate the idle view**

While parked, `/xte/charge` shows "No active charge session." and a working link to `/xte/reports`.

- [ ] **Step 5: Validate mid-session open (backfill)**

Open the page well into a charge; confirm the chart shows the full curve from t=0 (backfill) and the
state history shows prior transitions, then both continue live.

---

## Self-Review

**Spec coverage:**
- AC/DC/idle auto-selected views → Task 2 (dispatcher + idle + AC/DC renderers). ✓
- Live chart mirroring report series, fixed axes, backfill + live append → Task 3. ✓
- State history live (HLC/DC, AC-Op/AC) in the monitor → Task 4; in the report → Task 1. ✓
- No new routes / metrics / CAN changes → confirmed (backfill embedded; reads existing metrics). ✓
- Edge cases (null/wrong vehicle → idle; empty buffers → start empty; charts.js failure → panels still
  work) → handled in Tasks 2–4 code. ✓
- changes.txt → Task 5. ✓
- Build via CI, on-vehicle validation → Tasks 6–7. ✓

Note: the spec mentioned a "charge type changed — reload" hint; this is intentionally dropped from
the plan as YAGNI (the page is opened during a known session; a manual refresh re-selects the view).
This is a deliberate scope reduction, not a gap.

**Placeholder scan:** Task 2 introduces temporary stubs for `WebChgChartJs`/`WebChgStateHistoryJs`,
explicitly replaced in Tasks 3–4 — these keep each task compilable and are not residual placeholders.
No other TODO/TBD content.

**Type/name consistency:** Helper names (`WebChgRenderAc/Dc`, `WebChgChartJs`, `WebChgStateHistoryJs`),
the receiver id `chgmon`, the chart div `chgchart`, the history table `chghist`, and the DC header span
`hlcstate` are used consistently across the dispatcher and the helper definitions. Metric names match
the codebase (`xte.v.c.stamaxp/stamaxv/stamaxi`, `xte.v.c.perm`, `xte.v.c.tgti`, `xte.v.c.hlc`,
`xte.v.c.acop`, `xte.v.c.gridpower`, `xte.v.e.hvac.kwh`). `AcOpStatusLabel` and `last_acop` defined in
Task 1 are used by `SetAcOpStatus` (Task 1) and the history seeding (Task 4).
