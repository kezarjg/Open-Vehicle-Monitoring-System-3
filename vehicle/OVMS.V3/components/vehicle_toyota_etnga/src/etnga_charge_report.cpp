/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Toyota e-TNGA platform — charge session report
   Date:          5th June 2026

   (C) 2026       Jerry Kezar <solterra@kezarnet.com>

   Licensed under the MIT License. See the LICENSE file for details.

   Writes a self-contained HTML report to /store/charge-reports/ at the end of each
   charging session (plug-in to unplug). This is the first increment: a single-phase,
   live-telemetry summary. Multi-phase tracking, sleep-survival / summary-mode,
   limiting-side attribution, the per-sample timeline and the error DID-dump
   (see solterra-can docs/charging_state_machine_architecture.md §6) are follow-ups.
*/

#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#include <sdkconfig.h>
#include "ovms_log.h"
#include "metrics_standard.h"
#ifdef CONFIG_OVMS_COMP_LOCATION
#include "ovms_location.h"
#endif
#include "vehicle_toyota_etnga.h"

static const int CHARGE_REPORT_MAX = 50;   // retain at most this many reports

// Prefer the SD card (GBs, removable) for reports+CSV; fall back to internal flash.
std::string OvmsVehicleToyotaETNGA::ChargeReportDir()
{
    struct stat st;
    if (stat("/sd", &st) == 0 && S_ISDIR(st.st_mode))
        return "/sd/charge-reports";
    return "/store/charge-reports";
}

// Map a 0x1688 "Charging History Information" enum code to a human-readable label.
// All 26 states sourced from solterra-can/ecus/plug-in-charge-control.md (Known enum values).
// 6 of 26 are empirically confirmed; the rest are Techstream-inferred labels.
// Returns "" for unknown codes so the caller can fall back to showing raw hex.
const char* OvmsVehicleToyotaETNGA::ChargeOutcomeLabel(int code)
{
    switch (code & 0xFF) {
        case 0x00: return "Default";
        case 0x20: return "Default (AC Charging)";
        case 0x21: return "AC Charging Complete (Full Charge)";
        case 0x23: return "AC Charging Stop (Abnormal)";
        case 0x24: return "AC Charging Stop (Battery)";
        case 0x25: return "AC Charging Stop (High Power Consumption)";
        case 0x26: return "AC Charging Stop (Operation)";
        case 0x27: return "AC Charging Stop (Power Outage/Unplugged)";
        case 0x28: return "AC Charging Stop (Reduced Supply Power)";
        case 0x29: return "AC Charging Stop (System)";
        case 0x2A: return "AC Charging Complete (Freeze Prevention)";
        case 0x2C: return "AC Charging Stop (Connector Unlock)";
        case 0x30: return "Default (DC Charging)";
        case 0x31: return "DC Charging Complete";
        case 0x32: return "DC Charging Stop (Abnormal)";
        case 0x33: return "DC Charging Stop (Battery)";
        case 0x38: return "DC Charging Stop (Over 60 Minutes)";
        case 0x39: return "DC Charging Stop (System)";
        case 0x3A: return "DC Charging Stop (Vehicle/System)";
        case 0x40: return "Default (Power Feeding)";
        case 0x41: return "Battery Level Low";
        case 0x42: return "Fuel Level Low";
        case 0x43: return "VPC Unplugged";
        case 0x44: return "IG OFF Operation";
        case 0x45: return "Remote Stop";
        case 0x46: return "Vehicle Factor";
        default:   return "";   // unknown -> caller shows raw hex
    }
}

// Map a 0x1666 "DC HLC state" enum code (ISO 15118 high-level-communication state machine)
// to a human-readable label. Codes sourced from solterra-can (2026-05 DC-charge captures;
// 6 empirically confirmed, the rest inferred from the CCS state sequence).
// Returns "" for unknown codes so the caller skips logging them.
const char* OvmsVehicleToyotaETNGA::HlcStateLabel(int code)
{
    switch (code & 0xFF) {
        case 0x00: return "HLC: SLAC";
        case 0x01: return "HLC: SDP";
        case 0x02: return "HLC: App-protocol negotiation";
        case 0x03: return "HLC: Session setup";
        case 0x04: return "HLC: Service discovery";
        case 0x05: return "HLC: Service detail";
        case 0x06: return "HLC: Payment service selection";
        case 0x07: return "HLC: Certificate installation";
        case 0x08: return "HLC: Certificate update";
        case 0x09: return "HLC: Payment details";
        case 0x0A: return "HLC: Authorization";
        case 0x0B: return "HLC: Charge-parameter discovery";
        case 0x0C: return "HLC: Cable check";
        case 0x0D: return "HLC: Precharge";
        case 0x0E: return "HLC: Power delivery (start)";
        case 0x0F: return "HLC: Current demand";
        case 0x10: return "HLC: Power delivery (stop)";
        case 0x11: return "HLC: Welding detection";
        case 0x12: return "HLC: Session stop";
        case 0xFF: return "HLC: Unconnected";
        default:   return "";
    }
}

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

// Return the name of the first defined OVMS location (geofence) that contains (lat,lon),
// or "" if none match. Used to show a friendly place name in the report alongside coords.
std::string OvmsVehicleToyotaETNGA::LookupLocationName(float lat, float lon)
{
#ifdef CONFIG_OVMS_COMP_LOCATION
    for (LocationMap::iterator it = MyLocations.m_locations.begin();
         it != MyLocations.m_locations.end(); ++it) {
        if (it->second && it->second->IsInLocation(lat, lon))
            return it->first;
    }
#else
    (void)lat; (void)lon;
#endif
    return "";
}

// Append a monotonic-timestamped event to the session event log.
// No-op outside a session (guard matches the in_session flag set in TransitionToChargeHandshakeState).
void OvmsVehicleToyotaETNGA::LogChargeEvent(const char* label)
{
    if (!m_charge_session.in_session)
        return;
    m_charge_session.events.push_back(
        std::make_pair(StandardMetrics.ms_m_monotonic->AsInt(), label));
}

// Live aggregation while charging: peak power, battery-temp range, ambient range,
// delivered-Ah, per-tick CSV row, and downsampled SVG buffer.
// Called every tick from HandleChargeAcState / HandleChargeDcState.
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

    // Only fold in ambient when a fresh reading exists (env temp is undefined when parked
    // until the in-charge 0x1F46 poll answers) — prevents a bogus 0 C dragging the range.
    if (StandardMetrics.ms_v_env_temp->IsDefined()) {
        float amb = StandardMetrics.ms_v_env_temp->AsFloat();
        if (!m_charge_session.amb_seen) {
            m_charge_session.amb_min = m_charge_session.amb_max = amb;
            m_charge_session.amb_seen = true;
        } else if (amb < m_charge_session.amb_min) m_charge_session.amb_min = amb;
        else if (amb > m_charge_session.amb_max) m_charge_session.amb_max = amb;
    }

    m_charge_session.is_dc = (static_cast<PollState>(m_poll_state) == PollState::CHARGE_DC);

    // Delivered-Ah (charge-side coulomb): integrate pack current over dt.
    if (m_charge_session.last_sample_monotonic != 0) {
        int dt = now - m_charge_session.last_sample_monotonic;
        if (dt > 0 && dt <= 10) {   // ignore gaps (pause / lock-isolation / sleep) so delivered_ah stays accurate
            m_charge_session.delivered_ah += fabsf(StandardMetrics.ms_v_bat_current->AsFloat()) * (dt / 3600.0f);
        }
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
        s.sta_max  = m_v_charge_sta_max_p->AsFloat();        // station-offered max kW (0x166A, DC only; 0 on AC)
        s.car_perm = fabsf(m_v_charge_perm->AsFloat());      // car-permitted kW (0x16A1 is signed: |.| is the charge limit)
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

void OvmsVehicleToyotaETNGA::AppendChargeCsvRow()
{
    if (m_charge_session.base.empty())
        return;

    if (!m_charge_session.csv_started) {
        // Column semantics:
        //   car's asks      : car_perm_kw (0x16A1), car_target_a (0x166D) — both on the OBC / Plug-In Control ECU (0x745)
        //   station's caps  : station_max_kw/_a/_v
        //   actuals         : station_kw (from EVSE), battery_kw (into battery), hvac_kw (into cabin),
        //                     plus raw station_grid_kw / station_present_v / station_present_a
        //   obc_kw          : diagnostic — raw 0x10D4 (under-reads on DC, issue #109)
        m_charge_session.csv_buf =
            "elapsed_s,soc_pct,bms_soc_pct,station_kw,battery_kw,hvac_kw,pack_v,pack_a,"
            "batt_temp_c,ambient_c,state,station_max_kw,station_max_a,station_max_v,"
            "car_perm_kw,car_target_a,station_grid_kw,station_present_v,station_present_a,obc_kw\n";
        m_charge_session.csv_started = true;
    }

    int now = StandardMetrics.ms_m_monotonic->AsInt();
    int elapsed = now - m_charge_session.start_monotonic;
    // Ambient is undefined until the first in-charge 0x1F46 reply (~30 s in): write an
    // empty field rather than 0.0, which is indistinguishable from a real 0 C reading.
    char amb[16] = "";
    if (StandardMetrics.ms_v_env_temp->IsDefined())
        snprintf(amb, sizeof(amb), "%.1f", StandardMetrics.ms_v_env_temp->AsFloat());
    char row[320];
    float present_v = StandardMetrics.ms_v_charge_voltage->AsFloat();
    float present_a = StandardMetrics.ms_v_charge_current->AsFloat();
    float grid_kw   = m_v_charge_grid_power->AsFloat();
    float station_kw = m_charge_session.is_dc ? (present_v * present_a / 1000.0f) : grid_kw;
    snprintf(row, sizeof(row),
        "%d,%.0f,%.1f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%s,%s,"
        "%.2f,%.0f,%.0f,%.2f,%.0f,%.3f,%.0f,%.0f,%.3f\n",
        elapsed,
        StandardMetrics.ms_v_bat_soc->AsFloat(),
        m_v_bat_soc_bms->AsFloat(),
        station_kw,
        StandardMetrics.ms_v_charge_power->AsFloat(),   // battery_kw (corrected, pack V×I)
        m_v_env_hvac_power->AsFloat(),                   // hvac_kw
        StandardMetrics.ms_v_bat_voltage->AsFloat(),
        StandardMetrics.ms_v_bat_current->AsFloat(),
        StandardMetrics.ms_v_bat_temp->AsFloat(),
        amb,
        (m_charge_session.is_dc ? "DC" : "AC"),
        m_v_charge_sta_max_p->AsFloat(), m_v_charge_sta_max_i->AsFloat(), m_v_charge_sta_max_v->AsFloat(),
        m_v_charge_perm->AsFloat(), m_v_charge_tgti->AsFloat(),
        grid_kw, present_v, present_a,
        m_charge_obc_kw);
    m_charge_session.csv_buf += row;

    // Flush at most every 30 s (or ~4 KB): a 1 Hz open/append/close per row would put
    // thousands of write cycles on the internal flash when /store is the report target.
    if (m_charge_session.last_csv_flush == 0)
        m_charge_session.last_csv_flush = now;
    if (now - m_charge_session.last_csv_flush >= 30 || m_charge_session.csv_buf.size() >= 4096)
        FlushChargeCsv();
}

// Write the buffered CSV rows to <base>.csv (truncating on the first flush of a session).
void OvmsVehicleToyotaETNGA::FlushChargeCsv()
{
    if (m_charge_session.csv_buf.empty() || m_charge_session.base.empty())
        return;
    std::ofstream f(m_charge_session.base + ".csv", m_charge_session.csv_file_created
        ? (std::ios::out | std::ios::app)
        : (std::ios::out | std::ios::trunc));
    if (!f) {
        // Can't open (SD removed/unmounted mid-charge?): keep the rows for retry, but
        // bound the pending buffer — at ~100 B/row this otherwise grows without limit
        // for the rest of the session (heap safety).
        if (m_charge_session.csv_buf.size() > 16384) {
            ESP_LOGE(TAG, "Charge CSV unwritable, buffer over limit — discarding buffered rows: %s.csv",
                m_charge_session.base.c_str());
            m_charge_session.csv_buf.clear();
        }
        return;
    }
    f << m_charge_session.csv_buf;
    f.flush();
    if (f.fail()) {
        // Keep the rows for the next attempt (disk full / SD hiccup) — clearing here
        // silently lost them. last_csv_flush stays old, so retry on the next row.
        ESP_LOGE(TAG, "Charge CSV write failed (disk full?): %s.csv", m_charge_session.base.c_str());
        if (m_charge_session.csv_buf.size() > 16384) {
            // Filesystem persistently unwritable: bound the pending buffer (heap safety).
            ESP_LOGE(TAG, "Charge CSV buffer over limit — discarding buffered rows");
            m_charge_session.csv_buf.clear();
        }
        return;
    }
    m_charge_session.csv_buf.clear();
    m_charge_session.csv_file_created = true;
    m_charge_session.last_csv_flush = StandardMetrics.ms_m_monotonic->AsInt();
}

// Retain only the newest CHARGE_REPORT_MAX sessions; delete both .html and .csv for each pruned stem.
// Also removes orphan .csv files whose session never produced an .html (aborted/sleep-ended sessions).
static void PruneChargeReports(const char* tag, const std::string& dir)
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

    // Remove orphan CSVs (streamed CSV whose session never produced an HTML report).
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
        ESP_LOGD(tag, "Charge report pruned: %s", stems[i].c_str());
    }
}

// Render a self-contained inline SVG chart vs time, with standardized axes:
//   - left axis: delivered power (kW), fixed full-scale per charge type (AC 0-11, DC 0-150)
//   - right axis: SOC, fixed 0-100 %
//   - traces: delivered power, station-offered max (DC only) and car-permitted limit (the
//     0x16A1 taper curve), plus the SOC overlay. All powers plot as magnitudes (0x16A1 is
//     signed: negative = charging), so the three power traces share one positive axis.
// Streams directly to the (already open) report file: a 300-sample DC session built a
// ~25-30 KB contiguous std::string here, a needless heap spike on the ESP32.
void OvmsVehicleToyotaETNGA::RenderPowerSvg(std::ostream& out)
{
    const std::vector<ChargeSessionState::Sample>& s = m_charge_session.svg;
    if (s.size() < 2) {
        out << "<p>(not enough samples for a chart)</p>";
        return;
    }

    const bool  dc    = m_charge_session.is_dc;
    const float pmax  = dc ? 150.0f : 11.0f;   // fixed power-axis full scale (kW)
    const float pstep = dc ? 25.0f  : 2.0f;    // power gridline / tick spacing (kW)

    const int W = 640, H = 260, PADL = 44, PADB = 28, PADT = 12, PADR = 44;
    const int PW = W - PADL - PADR, PH = H - PADT - PADB;
    int tmax = s.back().t_s > 0 ? s.back().t_s : 1;

    char b[256];
    snprintf(b, sizeof(b), "<svg viewBox=\"0 0 %d %d\" style=\"width:100%%;max-width:%dpx;height:auto\" xmlns=\"http://www.w3.org/2000/svg\">\n", W, H, W);
    out << b;
    out << "<rect width=\"100%\" height=\"100%\" fill=\"#fff\"/>\n";

    // Horizontal gridlines + left power-axis (kW) labels.
    for (float kw = 0.0f; kw <= pmax + 0.01f; kw += pstep) {
        float y = PADT + (float)PH * (1.0f - kw / pmax);
        snprintf(b, sizeof(b), "<line x1=\"%d\" y1=\"%.1f\" x2=\"%d\" y2=\"%.1f\" stroke=\"#eee\"/>\n", PADL, y, W-PADR, y); out << b;
        snprintf(b, sizeof(b), "<text x=\"%d\" y=\"%.1f\" font-size=\"10\" fill=\"#06c\" text-anchor=\"end\">%g</text>\n", PADL-3, y+3, kw); out << b;
    }
    // Right SOC-axis (%) labels, fixed 0-100.
    for (int soc = 0; soc <= 100; soc += 25) {
        float y = PADT + (float)PH * (1.0f - soc / 100.0f);
        snprintf(b, sizeof(b), "<text x=\"%d\" y=\"%.1f\" font-size=\"10\" fill=\"#39c\">%d</text>\n", W-PADR+3, y+3, soc); out << b;
    }
    // Axis lines (left power, right SOC, bottom time).
    snprintf(b, sizeof(b), "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"#ccc\"/>\n", PADL, PADT, PADL, H-PADB); out << b;
    snprintf(b, sizeof(b), "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"#ccc\"/>\n", W-PADR, PADT, W-PADR, H-PADB); out << b;
    snprintf(b, sizeof(b), "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"#ccc\"/>\n", PADL, H-PADB, W-PADR, H-PADB); out << b;

    // SOC overlay (right axis, 0-100).
    out << "<polyline fill=\"none\" stroke=\"#39c\" stroke-width=\"1\" stroke-dasharray=\"1 3\" points=\"";
    for (size_t i = 0; i < s.size(); i++) {
        float x = PADL + (float)PW * s[i].t_s / tmax;
        float soc = s[i].soc < 0 ? 0.0f : (s[i].soc > 100 ? 100.0f : (float)s[i].soc);
        float y = PADT + (float)PH * (1.0f - soc / 100.0f);
        snprintf(b, sizeof(b), "%.1f,%.1f ", x, y); out << b;
    }
    out << "\"/>\n";

    // Station-offered max (DC only — 0x166A is not polled on AC).
    if (dc) {
        out << "<polyline fill=\"none\" stroke=\"#e80\" stroke-width=\"1\" stroke-dasharray=\"5 3\" points=\"";
        for (size_t i = 0; i < s.size(); i++) {
            float x = PADL + (float)PW * s[i].t_s / tmax;
            float v = s[i].sta_max; if (v < 0) v = 0; if (v > pmax) v = pmax;
            float y = PADT + (float)PH * (1.0f - v / pmax);
            snprintf(b, sizeof(b), "%.1f,%.1f ", x, y); out << b;
        }
        out << "\"/>\n";
    }

    // Car-permitted limit (the BMS taper curve).
    out << "<polyline fill=\"none\" stroke=\"#0a0\" stroke-width=\"1\" stroke-dasharray=\"5 3\" points=\"";
    for (size_t i = 0; i < s.size(); i++) {
        float x = PADL + (float)PW * s[i].t_s / tmax;
        float v = s[i].car_perm; if (v < 0) v = 0; if (v > pmax) v = pmax;
        float y = PADT + (float)PH * (1.0f - v / pmax);
        snprintf(b, sizeof(b), "%.1f,%.1f ", x, y); out << b;
    }
    out << "\"/>\n";

    // Delivered power (primary trace).
    out << "<polyline fill=\"none\" stroke=\"#06c\" stroke-width=\"2\" points=\"";
    for (size_t i = 0; i < s.size(); i++) {
        float x = PADL + (float)PW * s[i].t_s / tmax;
        float v = s[i].kw; if (v < 0) v = 0; if (v > pmax) v = pmax;
        float y = PADT + (float)PH * (1.0f - v / pmax);
        snprintf(b, sizeof(b), "%.1f,%.1f ", x, y); out << b;
    }
    out << "\"/>\n";

    // Legend (bottom).
    int ly = H - 6;
    snprintf(b, sizeof(b), "<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"#06c\">delivered kW</text>\n", PADL+6, ly); out << b;
    if (dc) { snprintf(b, sizeof(b), "<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"#e80\">station max</text>\n", PADL+96, ly); out << b; }
    snprintf(b, sizeof(b), "<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"#0a0\">car permitted</text>\n", PADL+186, ly); out << b;
    snprintf(b, sizeof(b), "<text x=\"%d\" y=\"%d\" font-size=\"10\" fill=\"#39c\">SOC %%</text>\n", PADL+282, ly); out << b;
    out << "</svg>\n";
}

void OvmsVehicleToyotaETNGA::GenerateChargeReport()
{
    const float energy_kwh = StandardMetrics.ms_v_charge_kwh->AsFloat();
    if (energy_kwh < 0.05f || m_charge_session.base.empty()) {
        ESP_LOGD(TAG, "Charge report skipped (%.3f kWh)", energy_kwh);
        if (m_charge_session.csv_file_created)   // remove the streamed stub CSV (buffered rows are discarded with the session)
            unlink((m_charge_session.base + ".csv").c_str());
        return;
    }
    FlushChargeCsv();   // complete the per-sample CSV before linking it from the report
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

    const char* vn = MyVehicleFactory.ActiveVehicleName();
    std::string vname = (vn && *vn) ? vn : "Toyota e-TNGA";

    char b[96];
    f << "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">\n"
      << "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
      << "<title>" << vname << " charge report " << sbuf << "</title>\n"
      << "<style>body{font:14px/1.4 system-ui,sans-serif;margin:1rem;max-width:46rem}"
      << "h1{font-size:1.3rem}h2{font-size:1.05rem;margin-top:1.3rem}"
      << "dl{display:grid;grid-template-columns:max-content 1fr;gap:.2rem .8rem}dt{font-weight:600}"
      << "table{border-collapse:collapse}td,th{border:1px solid #ddd;padding:.15rem .4rem;font-size:13px}"
      << ".est{color:#555}.note{color:#888;font-size:12px;margin-top:1.2rem}</style></head><body>\n"
      << "<h1>" << vname << " — charging session report</h1>\n";

    f << "<h2>Summary</h2>\n<dl>\n"
      << "<dt>Plug-in</dt><dd>" << sbuf << "</dd>\n<dt>Unplug</dt><dd>" << ebuf << "</dd>\n";
    snprintf(b, sizeof(b), "%dh %02dm %02ds", dh, dm, ds);
    f << "<dt>Duration</dt><dd>" << b << "</dd>\n";
    if (m_charge_session.has_loc) {
        std::string locname = LookupLocationName(m_charge_session.start_lat, m_charge_session.start_lon);
        snprintf(b, sizeof(b), "%.5f, %.5f", m_charge_session.start_lat, m_charge_session.start_lon);
        f << "<dt>Location</dt><dd>";
        if (!locname.empty()) f << locname << " &mdash; ";
        f << b
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
    // "From grid" is an AC-charge concept (the 0x161D grid-input poll only answers on AC);
    // on DC the energy meter is the station's, so show delivered only.
    if (m_charge_session.is_dc)
        snprintf(b, sizeof(b), "%.2f kWh delivered", energy_kwh);
    else
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

    f << "<h2>Charging power</h2>\n";
    RenderPowerSvg(f);

    f << "<h2>Session events</h2>\n<table><tr><th>Time</th><th>Event</th></tr>\n";
    for (size_t i = 0; i < m_charge_session.events.size(); i++) {
        int rel = m_charge_session.events[i].first - m_charge_session.start_monotonic; if (rel < 0) rel = 0;
        snprintf(b, sizeof(b), "%d:%02d", rel/60, rel%60);
        f << "<tr><td>" << b << "</td><td>" << m_charge_session.events[i].second << "</td></tr>\n";
    }
    f << "</table>\n";

    f << "<h2 class=\"est\">Estimates</h2>\n<dl class=\"est\">\n";
    if (!m_charge_session.is_dc && grid_kwh > 0.01f) {   // efficiency needs grid energy (AC only)
        snprintf(b, sizeof(b), "%.0f%% (%.2f kWh loss)", energy_kwh/grid_kwh*100.0f, grid_kwh-energy_kwh);
        f << "<dt>Charging efficiency</dt><dd>" << b << "</dd>\n";
    }
    if (start_soc >= 0 && end_soc > start_soc && m_charge_session.delivered_ah > 0.5f) {
        float cap = m_charge_session.delivered_ah / ((end_soc - start_soc) / 100.0f);
        snprintf(b, sizeof(b), "%.0f Ah implied (~%.0f%% of 201 Ah nominal)", cap, cap/201.1f*100.0f);
        f << "<dt>Implied capacity</dt><dd>" << b << "</dd>\n";
    }
    f << "</dl>\n";

    {
        std::string csv = m_charge_session.base + ".csv";
        std::string name = csv.substr(csv.find_last_of('/') + 1);
        // Include the storage location (the write side knows it) so WebChargeReport
        // serves the right file without a fallback scan over every location.
        const char* loc = (m_charge_session.base.compare(0, 4, "/sd/") == 0) ? "sd" : "store";
        f << "<p><a href=\"/xte/report?file=" << name << "&amp;loc=" << loc
          << "\">Download per-sample CSV</a></p>\n";
    }

    f << "<p class=\"note\">Generated on-module by OVMS (Toyota e-TNGA). Single-phase; avg power is over the "
         "whole plug-in interval. Estimates are provisional.</p>\n</body></html>\n";
    f.close();
    if (f.fail()) {
        // The stream writes above are individually unchecked; surface a disk-full /
        // truncated report here rather than logging success.
        ESP_LOGE(TAG, "Charge report write failed (disk full?): %s.html", m_charge_session.base.c_str());
    } else {
        ESP_LOGI(TAG, "Charge report written: %s.html (%.2f kWh, %d%%->%d%%)",
            m_charge_session.base.c_str(), energy_kwh, start_soc, end_soc);
    }
    PruneChargeReports(TAG, ChargeReportDir());
}
