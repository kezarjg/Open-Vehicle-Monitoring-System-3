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

#include "ovms_log.h"
#include "metrics_standard.h"
#include "vehicle_toyota_etnga.h"

#define CHARGE_REPORT_DIR  "/store/charge-reports"
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
        m_v_charge_grid_power->AsFloat(),
        StandardMetrics.ms_v_charge_voltage->AsFloat(), StandardMetrics.ms_v_charge_current->AsFloat());
    f << row;
}

// Retain only the newest CHARGE_REPORT_MAX .html reports (timestamp filenames sort chronologically).
static void PruneChargeReports(const char* tag)
{
    DIR* dir = opendir(CHARGE_REPORT_DIR);
    if (!dir)
        return;

    std::vector<std::string> files;
    struct dirent* ent;
    while ((ent = readdir(dir)) != NULL) {
        std::string name = ent->d_name;
        if (name.size() > 5 && name.compare(name.size() - 5, 5, ".html") == 0)
            files.push_back(name);
    }
    closedir(dir);

    if ((int) files.size() <= CHARGE_REPORT_MAX)
        return;

    std::sort(files.begin(), files.end());
    int to_delete = (int) files.size() - CHARGE_REPORT_MAX;
    for (int i = 0; i < to_delete; i++) {
        std::string path = std::string(CHARGE_REPORT_DIR "/") + files[i];
        if (unlink(path.c_str()) == 0)
            ESP_LOGD(tag, "Charge report pruned: %s", files[i].c_str());
    }
}

// Render a self-contained inline SVG line chart of delivered power (+ light SOC overlay) vs time.
std::string OvmsVehicleToyotaETNGA::RenderPowerSvg()
{
    const std::vector<ChargeSessionState::Sample>& s = m_charge_session.svg;
    if (s.size() < 2)
        return "<p>(not enough samples for a chart)</p>";

    const int W = 640, H = 240, PADL = 44, PADB = 24, PADT = 10, PADR = 10;
    const int PW = W - PADL - PADR, PH = H - PADT - PADB;
    int tmax = s.back().t_s > 0 ? s.back().t_s : 1;
    float kwmax = 1.0f;
    for (size_t i = 0; i < s.size(); i++)
        if (s[i].kw > kwmax) kwmax = s[i].kw;

    std::string out;
    char b[160];
    snprintf(b, sizeof(b), "<svg viewBox=\"0 0 %d %d\" style=\"width:100%%;max-width:%dpx;height:auto\" xmlns=\"http://www.w3.org/2000/svg\">\n", W, H, W);
    out += b;
    out += "<rect width=\"100%\" height=\"100%\" fill=\"#fff\"/>\n";
    snprintf(b, sizeof(b), "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"#ccc\"/>\n", PADL, PADT, PADL, H-PADB); out += b;
    snprintf(b, sizeof(b), "<line x1=\"%d\" y1=\"%d\" x2=\"%d\" y2=\"%d\" stroke=\"#ccc\"/>\n", PADL, H-PADB, W-PADR, H-PADB); out += b;
    snprintf(b, sizeof(b), "<text x=\"4\" y=\"%d\" font-size=\"10\" fill=\"#666\">%.0f kW</text>\n", PADT+8, kwmax); out += b;

    // SOC overlay (light)
    out += "<polyline fill=\"none\" stroke=\"#9cf\" stroke-width=\"1\" points=\"";
    for (size_t i = 0; i < s.size(); i++) {
        float x = PADL + (float)PW * s[i].t_s / tmax;
        float y = PADT + (float)PH * (1.0f - s[i].soc / 100.0f);
        snprintf(b, sizeof(b), "%.1f,%.1f ", x, y); out += b;
    }
    out += "\"/>\n";

    // delivered power
    out += "<polyline fill=\"none\" stroke=\"#06c\" stroke-width=\"2\" points=\"";
    for (size_t i = 0; i < s.size(); i++) {
        float x = PADL + (float)PW * s[i].t_s / tmax;
        float y = PADT + (float)PH * (1.0f - s[i].kw / kwmax);
        snprintf(b, sizeof(b), "%.1f,%.1f ", x, y); out += b;
    }
    out += "\"/>\n";

    snprintf(b, sizeof(b), "<text x=\"50\" y=\"%d\" font-size=\"10\" fill=\"#06c\">power (kW)</text><text x=\"140\" y=\"%d\" font-size=\"10\" fill=\"#9cf\">SOC %%</text>\n", H-6, H-6);
    out += b;
    out += "</svg>\n";
    return out;
}

// Write the session-end report. No-op if no meaningful energy was delivered (plug-in then unplug).
void OvmsVehicleToyotaETNGA::GenerateChargeReport()
{
    const float energy_kwh = StandardMetrics.ms_v_charge_kwh->AsFloat();
    if (energy_kwh < 0.05f) {
        ESP_LOGD(TAG, "Charge report skipped: only %.3f kWh delivered", energy_kwh);
        return;
    }

    const float grid_kwh  = StandardMetrics.ms_v_charge_kwh_grid->AsFloat();
    const int   end_soc   = (int) StandardMetrics.ms_v_bat_soc->AsFloat();
    const int   start_soc = m_charge_session.start_soc;
    int duration_s = StandardMetrics.ms_m_monotonic->AsInt() - m_charge_session.start_monotonic;
    if (duration_s < 0)
        duration_s = 0;
    const float avg_kw  = (duration_s > 0) ? energy_kwh / (duration_s / 3600.0f) : 0.0f;
    const int   outcome = m_v_charge_outcome->AsInt();

    // Time formatting (UTC). start_utc is 0 / tiny if the clock was never synced.
    const bool time_ok = (m_charge_session.start_utc > 1000000000);   // ~2001 sanity floor
    char start_buf[40] = "(clock not synced)";
    char end_buf[40]   = "(clock not synced)";
    char fname[80];
    if (time_ok) {
        time_t st = (time_t) m_charge_session.start_utc;
        time_t et = st + duration_s;
        struct tm tmv;
        gmtime_r(&st, &tmv); strftime(start_buf, sizeof(start_buf), "%Y-%m-%d %H:%M:%S UTC", &tmv);
        gmtime_r(&et, &tmv); strftime(end_buf,   sizeof(end_buf),   "%Y-%m-%d %H:%M:%S UTC", &tmv);
        gmtime_r(&st, &tmv); strftime(fname, sizeof(fname), CHARGE_REPORT_DIR "/%Y%m%dT%H%M%SZ.html", &tmv);
    } else {
        snprintf(fname, sizeof(fname), CHARGE_REPORT_DIR "/charge-%d.html", m_charge_session.start_monotonic);
    }

    const int dh = duration_s / 3600, dm = (duration_s % 3600) / 60, ds = duration_s % 60;

    mkdir(CHARGE_REPORT_DIR, 0755);   // ensure the directory exists (ignores EEXIST)

    std::ofstream f(fname, std::ios::out | std::ios::trunc);
    if (!f) {
        ESP_LOGE(TAG, "Charge report: cannot write %s", fname);
        return;
    }

    char buf[64];
    f << "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">\n"
      << "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
      << "<title>Charge report " << start_buf << "</title>\n"
      << "<style>body{font:14px/1.4 system-ui,sans-serif;margin:1rem;max-width:42rem}"
      << "h1{font-size:1.3rem}dl{display:grid;grid-template-columns:max-content 1fr;gap:.2rem .8rem}"
      << "dt{font-weight:600}.note{color:#888;font-size:12px;margin-top:1.2rem}</style></head><body>\n"
      << "<h1>Charging session report</h1>\n<dl>\n"
      << "<dt>Plug-in</dt><dd>" << start_buf << "</dd>\n"
      << "<dt>Unplug</dt><dd>"  << end_buf   << "</dd>\n";

    snprintf(buf, sizeof(buf), "%dh %02dm %02ds", dh, dm, ds);
    f << "<dt>Duration</dt><dd>" << buf << "</dd>\n"
      << "<dt>Type</dt><dd>" << (m_charge_session.is_dc ? "DC fast" : "AC") << "</dd>\n";

    snprintf(buf, sizeof(buf), "%d%% &rarr; %d%% (+%d%%)", start_soc, end_soc,
             (start_soc >= 0 ? end_soc - start_soc : 0));
    f << "<dt>SOC</dt><dd>" << buf << "</dd>\n";

    snprintf(buf, sizeof(buf), "%.2f kWh", energy_kwh);
    f << "<dt>Energy delivered</dt><dd>" << buf << "</dd>\n";
    snprintf(buf, sizeof(buf), "%.2f kWh", grid_kwh);
    f << "<dt>Energy from grid</dt><dd>" << buf << "</dd>\n";
    snprintf(buf, sizeof(buf), "%.1f kW peak / %.2f kW avg", m_charge_session.peak_power, avg_kw);
    f << "<dt>Power</dt><dd>" << buf << "</dd>\n";

    if (m_charge_session.temp_seen) {
        snprintf(buf, sizeof(buf), "%.0f&deg;C &rarr; %.0f&deg;C",
                 m_charge_session.temp_min, m_charge_session.temp_max);
        f << "<dt>Battery temp</dt><dd>" << buf << "</dd>\n";
    }

    snprintf(buf, sizeof(buf), "0x%02X", outcome & 0xFF);
    f << "<dt>Outcome (0x1688)</dt><dd>" << buf << "</dd>\n</dl>\n";

    f << "<p class=\"note\">Generated on-module by OVMS (Toyota e-TNGA). Single-phase summary; "
         "average power is over the whole plug-in interval. Multi-phase, sleep-survival, "
         "limiting-side attribution and the per-sample timeline are not yet included.</p>\n"
         "</body></html>\n";
    f.close();

    ESP_LOGI(TAG, "Charge report written: %s (%.2f kWh, %d%%->%d%%)", fname, energy_kwh, start_soc, end_soc);

    PruneChargeReports(TAG);
}
