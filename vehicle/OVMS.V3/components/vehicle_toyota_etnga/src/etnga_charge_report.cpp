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
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include "ovms_log.h"
#include "metrics_standard.h"
#include "vehicle_toyota_etnga.h"

#define CHARGE_REPORT_DIR  "/store/charge-reports"
static const int CHARGE_REPORT_MAX = 50;   // retain at most this many reports

// Live aggregation while charging: peak power, battery-temp range, and the current phase type.
// Called every tick from HandleChargeAcState / HandleChargeDcState.
void OvmsVehicleToyotaETNGA::UpdateChargeSessionStats()
{
    if (!m_charge_session.in_session)
        return;

    float p = StandardMetrics.ms_v_charge_power->AsFloat();
    if (p > m_charge_session.peak_power)
        m_charge_session.peak_power = p;

    float t = StandardMetrics.ms_v_bat_temp->AsFloat();
    if (!m_charge_session.temp_seen) {
        m_charge_session.temp_min = m_charge_session.temp_max = t;
        m_charge_session.temp_seen = true;
    } else if (t < m_charge_session.temp_min) {
        m_charge_session.temp_min = t;
    } else if (t > m_charge_session.temp_max) {
        m_charge_session.temp_max = t;
    }

    m_charge_session.is_dc = (static_cast<PollState>(m_poll_state) == PollState::CHARGE_DC);
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
