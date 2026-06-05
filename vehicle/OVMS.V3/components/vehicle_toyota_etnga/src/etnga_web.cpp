/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Toyota e-TNGA platform — web UI
   Date:          5th June 2026

   (C) 2026       Jerry Kezar <solterra@kezarnet.com>

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include <sdkconfig.h>
#ifdef CONFIG_OVMS_COMP_WEBSERVER

#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "ovms_config.h"
#include "ovms_metrics.h"
#include "ovms_webserver.h"
#include "metrics_standard.h"

#include "vehicle_toyota_etnga.h"

// WebInit: register the e-TNGA pages in the vehicle menu.
// Shared by all e-TNGA vehicles (Subaru Solterra, Toyota bZ4X).
void OvmsVehicleToyotaETNGA::WebInit()
{
    MyWebServer.RegisterPage("/bms/cellmon", "BMS cell monitor", OvmsWebServer::HandleBmsCellMonitor, PageMenu_Vehicle, PageAuth_Cookie);
    MyWebServer.RegisterPage("/xte/charge", "Charging metrics", WebDispChgMetrics, PageMenu_Vehicle, PageAuth_Cookie);
    MyWebServer.RegisterPage("/xte/config", "Configuration", WebCfgFeatures, PageMenu_Vehicle, PageAuth_Cookie);
}

void OvmsVehicleToyotaETNGA::WebDeInit()
{
    MyWebServer.DeregisterPage("/bms/cellmon");
    MyWebServer.DeregisterPage("/xte/charge");
    MyWebServer.DeregisterPage("/xte/config");
}

// WebCfgFeatures: configure the e-TNGA TPMS alert thresholds (config namespace "xte").
// These are otherwise only settable from the shell `config` command.
void OvmsVehicleToyotaETNGA::WebCfgFeatures(PageEntry_t& p, PageContext_t& c)
{
    std::string error;
    char buf[32];

    if (c.method == "POST") {
        float p_warn  = atof(c.getvar("tpms_p_warn").c_str());
        float p_alert = atof(c.getvar("tpms_p_alert").c_str());
        float t_warn  = atof(c.getvar("tpms_t_warn").c_str());
        float t_alert = atof(c.getvar("tpms_t_alert").c_str());

        // Validate: pressures positive; alert at/below warn (low pressure is worse),
        // temperature alert at/above warn (high temperature is worse).
        if (p_warn <= 0 || p_alert <= 0)
            error += "<li>Tyre pressure thresholds must be greater than zero.</li>";
        if (p_alert > p_warn)
            error += "<li>Pressure alert should be at or below the warning threshold (lower pressure is worse).</li>";
        if (t_alert < t_warn)
            error += "<li>Temperature alert should be at or above the warning threshold (higher temperature is worse).</li>";

        if (error == "") {
            MyConfig.SetParamValueFloat("xte", "tpms.pressure.warn",  p_warn);
            MyConfig.SetParamValueFloat("xte", "tpms.pressure.alert", p_alert);
            MyConfig.SetParamValueFloat("xte", "tpms.temp.warn",      t_warn);
            MyConfig.SetParamValueFloat("xte", "tpms.temp.alert",     t_alert);

            c.head(200);
            c.alert("success", "<p class=\"lead\">Toyota e-TNGA configuration saved.</p>");
        } else {
            error = "<p class=\"lead\">Error:</p><ul class=\"errorlist\">" + error + "</ul>";
            c.head(400);
            c.alert("danger", error.c_str());
        }
    } else {
        c.head(200);
    }

    // Read current config (defaults mirror etnga_tpms.cpp):
    float p_warn  = MyConfig.GetParamValueFloat("xte", "tpms.pressure.warn",  240.0f);
    float p_alert = MyConfig.GetParamValueFloat("xte", "tpms.pressure.alert", 220.0f);
    float t_warn  = MyConfig.GetParamValueFloat("xte", "tpms.temp.warn",       90.0f);
    float t_alert = MyConfig.GetParamValueFloat("xte", "tpms.temp.alert",     100.0f);

    c.panel_start("primary", "Toyota e-TNGA configuration");
    c.form_start(p.uri);

    c.fieldset_start("TPMS alert thresholds");

    snprintf(buf, sizeof(buf), "%.0f", p_warn);
    c.input("number", "Pressure warning", "tpms_p_warn", buf, "Default: 240",
        "<p>Warn when a tyre drops to this pressure.</p>", "min=\"0\" step=\"1\"", "kPa");
    snprintf(buf, sizeof(buf), "%.0f", p_alert);
    c.input("number", "Pressure alert", "tpms_p_alert", buf, "Default: 220",
        "<p>Alert when a tyre drops to this pressure (at or below the warning level).</p>", "min=\"0\" step=\"1\"", "kPa");
    snprintf(buf, sizeof(buf), "%.0f", t_warn);
    c.input("number", "Temperature warning", "tpms_t_warn", buf, "Default: 90",
        "<p>Warn when a tyre reaches this temperature.</p>", "step=\"1\"", "&deg;C");
    snprintf(buf, sizeof(buf), "%.0f", t_alert);
    c.input("number", "Temperature alert", "tpms_t_alert", buf, "Default: 100",
        "<p>Alert when a tyre reaches this temperature (at or above the warning level).</p>", "step=\"1\"", "&deg;C");

    c.fieldset_end();

    c.print("<hr>");
    c.input_button("default", "Save");
    c.form_end();
    c.panel_end();
    c.done();
}

// WebDispChgMetrics: live charging dashboard. Values auto-update via the
// framework's data-metric mechanism (script.js polls /api/status).
void OvmsVehicleToyotaETNGA::WebDispChgMetrics(PageEntry_t& p, PageContext_t& c)
{
    c.head(200);
    PAGE_HOOK("body.pre");

    c.print(
        "<style>\n"
        "h6.metric-head { margin-bottom: 0; color: #676767; font-size: 15px; }\n"
        ".night h6.metric-head { color: unset; }\n"
        "</style>\n"
        "<div class=\"panel panel-primary\">"
          "<div class=\"panel-heading\">Toyota e-TNGA charging metrics</div>"
          "<div class=\"panel-body\">"
            "<div class=\"receiver\">"

              "<div class=\"clearfix\">"
                "<div class=\"metric progress\" data-metric=\"v.b.soc\" data-prec=\"1\">"
                  "<div class=\"progress-bar value-low text-left\" role=\"progressbar\" aria-valuenow=\"0\" aria-valuemin=\"0\" aria-valuemax=\"100\" style=\"width:0%\">"
                    "<div><span class=\"label\">SoC</span><span class=\"value\">?</span><span class=\"unit\">%</span></div>"
                  "</div>"
                "</div>"
              "</div>"

              "<div class=\"clearfix\">"
                "<h6 class=\"metric-head\">Battery</h6>"
                "<div class=\"metric number\" data-metric=\"v.b.power\" data-prec=\"3\"><span class=\"label\">Power</span><span class=\"value\">?</span><span class=\"unit\">kW</span></div>"
                "<div class=\"metric number\" data-metric=\"v.b.voltage\" data-prec=\"1\"><span class=\"label\">Voltage</span><span class=\"value\">?</span><span class=\"unit\">V</span></div>"
                "<div class=\"metric number\" data-metric=\"v.b.current\" data-prec=\"1\"><span class=\"label\">Current</span><span class=\"value\">?</span><span class=\"unit\">A</span></div>"
                "<div class=\"metric number\" data-metric=\"v.b.temp\" data-prec=\"1\"><span class=\"label\">Temp</span><span class=\"value\">?</span><span class=\"unit\">&deg;C</span></div>"
              "</div>"

              "<div class=\"clearfix\">"
                "<h6 class=\"metric-head\">Charger</h6>"
                "<div class=\"metric text\" data-metric=\"v.c.state\"><span class=\"label\">State</span><span class=\"value\">?</span></div>"
                "<div class=\"metric text\" data-metric=\"v.c.type\"><span class=\"label\">Type</span><span class=\"value\">?</span></div>"
                "<div class=\"metric number\" data-metric=\"v.c.power\" data-prec=\"3\"><span class=\"label\">Power</span><span class=\"value\">?</span><span class=\"unit\">kW</span></div>"
                "<div class=\"metric number\" data-metric=\"v.c.voltage\" data-prec=\"1\"><span class=\"label\">Voltage</span><span class=\"value\">?</span><span class=\"unit\">V</span></div>"
                "<div class=\"metric number\" data-metric=\"v.c.current\" data-prec=\"1\"><span class=\"label\">Current</span><span class=\"value\">?</span><span class=\"unit\">A</span></div>"
              "</div>"

              "<div class=\"clearfix\">"
                "<h6 class=\"metric-head\">Session energy</h6>"
                "<div class=\"metric number\" data-metric=\"v.c.kwh\" data-prec=\"2\"><span class=\"label\">Charged</span><span class=\"value\">?</span><span class=\"unit\">kWh</span></div>"
                "<div class=\"metric number\" data-metric=\"v.c.kwh.grid\" data-prec=\"2\"><span class=\"label\">From grid</span><span class=\"value\">?</span><span class=\"unit\">kWh</span></div>"
                "<div class=\"metric number\" data-metric=\"xte.v.e.hvac.kwh\" data-prec=\"3\"><span class=\"label\">Cabin (My Room)</span><span class=\"value\">?</span><span class=\"unit\">kWh</span></div>"
              "</div>"

            "</div>"
          "</div>"
        "</div>");

    PAGE_HOOK("body.post");
    c.done();
}

#endif // CONFIG_OVMS_COMP_WEBSERVER
