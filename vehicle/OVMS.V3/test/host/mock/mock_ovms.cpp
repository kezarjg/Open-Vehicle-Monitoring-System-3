// mock_ovms.cpp — single definition of the mock framework globals + free helpers.
#include "mock_ovms.hpp"

MetricStore         g_metrics;
StandardMetricsType StandardMetrics;
OvmsMetricsManager  MyMetrics;
OvmsConfig          MyConfig;

uint32_t esp_log_timestamp() { return 0; }

std::string hexencode(const std::string& data)
{
    static const char* hex = "0123456789ABCDEF";
    std::string out;
    out.reserve(data.size() * 2);
    for (unsigned char c : data) { out += hex[c >> 4]; out += hex[c & 0xF]; }
    return out;
}
