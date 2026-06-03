// mock_ovms.hpp — Stubs for OVMS framework types enabling native (host) builds.
//
// Force-included (g++ -include) so it loads before any vehicle module header,
// letting the real vehicle .cpp compile and run on a laptop with no ESP-IDF.
//
// Shared harness: this file is intended to cover the framework surface used by
// *any* vehicle module's decode/dispatch path. Extend it (add metrics, base
// methods, constants) as new vehicles are brought under host test — do not fork
// a per-vehicle copy. Originated from the vehicle_vwegolf/tests mock (A. Caps).

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cinttypes>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <functional>
#include <memory>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <utility>

// ---------------------------------------------------------------------------
// Logging — LOGD/LOGV are no-ops (verbose); LOGE/LOGW/LOGI print.
// ---------------------------------------------------------------------------
#define ESP_LOGE(tag, fmt, ...) fprintf(stderr, "[E][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) fprintf(stderr, "[W][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) fprintf(stdout, "[I][%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) do {} while (0)
#define ESP_LOGV(tag, fmt, ...) do {} while (0)

uint32_t esp_log_timestamp();          // monotonic ms; mock returns 0
std::string hexencode(const std::string& data);

// ---------------------------------------------------------------------------
// FreeRTOS odds and ends
// ---------------------------------------------------------------------------
typedef uint32_t TickType_t;
inline void vTaskDelay(TickType_t) {}
inline TickType_t pdMS_TO_TICKS(uint32_t ms) { return ms; }

// ---------------------------------------------------------------------------
// CAN types
// ---------------------------------------------------------------------------
typedef enum { CAN_frame_std, CAN_frame_ext } CAN_frame_format_t;
typedef enum { CAN_MODE_LISTEN, CAN_MODE_ACTIVE, CAN_MODE_OFF } CAN_mode_t;
typedef enum { CAN_SPEED_100KBPS = 100, CAN_SPEED_500KBPS = 500 } CAN_speed_t;
typedef int esp_err_t;
static constexpr esp_err_t ESP_OK = 0;

struct canbus {
    int m_busnumber = 0;
    esp_err_t WriteStandard(uint32_t, uint8_t, uint8_t*, int = 0) { return ESP_OK; }
    esp_err_t WriteExtended(uint32_t, uint8_t, uint8_t*, int = 0) { return ESP_OK; }
    esp_err_t Reset() { return ESP_OK; }
};

typedef union {
    struct { CAN_frame_format_t FF; uint8_t DLC; } B;
} CAN_FIR_t;

struct CAN_frame_t {
    canbus*    origin   = nullptr;
    void*      callback = nullptr;
    CAN_FIR_t  FIR      = {};
    uint32_t   MsgID    = 0;
    union { uint8_t u8[8]; uint32_t u32[2]; uint64_t u64; } data = {};
};

// ---------------------------------------------------------------------------
// Metric units / autostale tags (values arbitrary — only identity matters here)
// ---------------------------------------------------------------------------
enum metric_unit_t {
    Other = 0, Percentage, Celcius, Kilometers, kW, Amps, Volts, Minutes
};
enum { SM_STALE_NONE = 0, SM_STALE_MIN = 15, SM_STALE_MID = 120, SM_STALE_MAX = 600 };

// ---------------------------------------------------------------------------
// Metric store — values live in a global keyed store so a single
// `g_metrics = MetricStore{}` resets all metric state between tests.
// Metric objects are thin name-keyed accessors over this store.
// ---------------------------------------------------------------------------
struct MetricStore {
    std::map<std::string, double>             numbers;
    std::map<std::string, std::string>        strings;
    std::map<std::string, std::vector<float>> vectors;
    std::set<std::string>                     fresh;   // SetValue since reset => not stale
};
extern MetricStore g_metrics;

inline double      msNum(const std::string& n) { auto it = g_metrics.numbers.find(n); return it == g_metrics.numbers.end() ? 0.0 : it->second; }
inline std::string msStr(const std::string& n) { auto it = g_metrics.strings.find(n); return it == g_metrics.strings.end() ? std::string() : it->second; }
inline bool        msFresh(const std::string& n) { return g_metrics.fresh.find(n) != g_metrics.fresh.end(); }

template <typename T>
class OvmsMetric {
    std::string m_name;
public:
    explicit OvmsMetric(const char* n) : m_name(n) {}
    void  SetValue(T v)        { g_metrics.numbers[m_name] = static_cast<double>(v); g_metrics.fresh.insert(m_name); }
    void  SetValue(T v, int)   { SetValue(v); }
    T     AsValue() const      { return static_cast<T>(msNum(m_name)); }
    float AsFloat() const      { return static_cast<float>(msNum(m_name)); }
    int   AsInt()   const      { return static_cast<int>(msNum(m_name)); }
    bool  AsBool()  const      { return msNum(m_name) != 0.0; }
    bool  IsStale() const      { return !msFresh(m_name); }
    void  Clear()              { g_metrics.numbers.erase(m_name); g_metrics.fresh.erase(m_name); }
    void  SetStale(bool s = true) { if (s) g_metrics.fresh.erase(m_name); else g_metrics.fresh.insert(m_name); }
    void  SetAutoStale(int)    {}
};

template <>
class OvmsMetric<std::string> {
    std::string m_name;
public:
    explicit OvmsMetric(const char* n) : m_name(n) {}
    void        SetValue(const std::string& v) { g_metrics.strings[m_name] = v; g_metrics.fresh.insert(m_name); }
    void        SetValue(std::string&& v)       { g_metrics.strings[m_name] = std::move(v); g_metrics.fresh.insert(m_name); }
    void        SetValue(const char* v)         { g_metrics.strings[m_name] = v ? v : ""; g_metrics.fresh.insert(m_name); }
    std::string AsValue()  const { return msStr(m_name); }
    std::string AsString() const { return msStr(m_name); }
    bool        IsStale()  const { return !msFresh(m_name); }
    void        Clear()          { g_metrics.strings.erase(m_name); g_metrics.fresh.erase(m_name); }
    void        SetStale(bool s = true) { if (s) g_metrics.fresh.erase(m_name); else g_metrics.fresh.insert(m_name); }
    void        SetAutoStale(int) {}
};

class OvmsMetricVector {
    std::string m_name;
public:
    explicit OvmsMetricVector(const char* n) : m_name(n) {}
    void SetValue(const std::vector<float>& v) { g_metrics.vectors[m_name] = v; g_metrics.fresh.insert(m_name); }
    void SetValue(const std::vector<short>& v) { g_metrics.vectors[m_name] = std::vector<float>(v.begin(), v.end()); g_metrics.fresh.insert(m_name); }
    std::vector<float> AsVector() const { auto it = g_metrics.vectors.find(m_name); return it == g_metrics.vectors.end() ? std::vector<float>() : it->second; }
    bool IsStale() const { return !msFresh(m_name); }
    void Clear()         { g_metrics.vectors.erase(m_name); g_metrics.fresh.erase(m_name); }
    void SetStale(bool s = true) { if (s) g_metrics.fresh.erase(m_name); else g_metrics.fresh.insert(m_name); }
    void SetAutoStale(int) {}
};

using OvmsMetricInt    = OvmsMetric<int>;
using OvmsMetricFloat  = OvmsMetric<float>;
using OvmsMetricBool   = OvmsMetric<bool>;
using OvmsMetricString = OvmsMetric<std::string>;

// StandardMetrics — pointers, to match how the real code accesses them.
struct StandardMetricsType {
    // Battery
    OvmsMetricFloat*  ms_v_bat_soc            = new OvmsMetricFloat("ms_v_bat_soc");
    OvmsMetricFloat*  ms_v_bat_voltage        = new OvmsMetricFloat("ms_v_bat_voltage");
    OvmsMetricFloat*  ms_v_bat_current        = new OvmsMetricFloat("ms_v_bat_current");
    OvmsMetricFloat*  ms_v_bat_power          = new OvmsMetricFloat("ms_v_bat_power");
    OvmsMetricFloat*  ms_v_bat_temp           = new OvmsMetricFloat("ms_v_bat_temp");
    OvmsMetricFloat*  ms_v_bat_energy_used     = new OvmsMetricFloat("ms_v_bat_energy_used");
    OvmsMetricFloat*  ms_v_bat_energy_recd     = new OvmsMetricFloat("ms_v_bat_energy_recd");
    OvmsMetricFloat*  ms_v_bat_pack_tavg       = new OvmsMetricFloat("ms_v_bat_pack_tavg");
    OvmsMetricFloat*  ms_v_bat_pack_tmax       = new OvmsMetricFloat("ms_v_bat_pack_tmax");
    OvmsMetricFloat*  ms_v_bat_pack_tmin       = new OvmsMetricFloat("ms_v_bat_pack_tmin");
    OvmsMetricFloat*  ms_v_bat_pack_tstddev    = new OvmsMetricFloat("ms_v_bat_pack_tstddev");
    OvmsMetricFloat*  ms_v_bat_pack_vavg       = new OvmsMetricFloat("ms_v_bat_pack_vavg");
    OvmsMetricFloat*  ms_v_bat_pack_vmax       = new OvmsMetricFloat("ms_v_bat_pack_vmax");
    OvmsMetricFloat*  ms_v_bat_pack_vmin       = new OvmsMetricFloat("ms_v_bat_pack_vmin");
    OvmsMetricFloat*  ms_v_bat_pack_vstddev    = new OvmsMetricFloat("ms_v_bat_pack_vstddev");
    OvmsMetricFloat*  ms_v_bat_12v_voltage     = new OvmsMetricFloat("ms_v_bat_12v_voltage");
    OvmsMetricFloat*  ms_v_bat_12v_voltage_ref = new OvmsMetricFloat("ms_v_bat_12v_voltage_ref");
    OvmsMetricVector* ms_v_bat_cell_voltage    = new OvmsMetricVector("ms_v_bat_cell_voltage");
    OvmsMetricVector* ms_v_bat_cell_temp       = new OvmsMetricVector("ms_v_bat_cell_temp");
    // Position
    OvmsMetricFloat*  ms_v_pos_speed          = new OvmsMetricFloat("ms_v_pos_speed");
    OvmsMetricFloat*  ms_v_pos_odometer       = new OvmsMetricFloat("ms_v_pos_odometer");
    OvmsMetricFloat*  ms_v_pos_trip           = new OvmsMetricFloat("ms_v_pos_trip");
    // Environment
    OvmsMetricFloat*  ms_v_env_temp           = new OvmsMetricFloat("ms_v_env_temp");
    OvmsMetricFloat*  ms_v_env_cabintemp      = new OvmsMetricFloat("ms_v_env_cabintemp");
    OvmsMetricFloat*  ms_v_env_cabinsetpoint  = new OvmsMetricFloat("ms_v_env_cabinsetpoint");
    OvmsMetricInt*    ms_v_env_gear           = new OvmsMetricInt("ms_v_env_gear");
    OvmsMetricBool*   ms_v_env_awake          = new OvmsMetricBool("ms_v_env_awake");
    OvmsMetricBool*   ms_v_env_on             = new OvmsMetricBool("ms_v_env_on");
    // Doors
    OvmsMetricBool*   ms_v_door_chargeport    = new OvmsMetricBool("ms_v_door_chargeport");
    // Charge
    OvmsMetricBool*   ms_v_charge_inprogress  = new OvmsMetricBool("ms_v_charge_inprogress");
    OvmsMetricBool*   ms_v_charge_pilot       = new OvmsMetricBool("ms_v_charge_pilot");
    OvmsMetricString* ms_v_charge_mode        = new OvmsMetricString("ms_v_charge_mode");
    OvmsMetricString* ms_v_charge_type        = new OvmsMetricString("ms_v_charge_type");
    OvmsMetricString* ms_v_charge_state       = new OvmsMetricString("ms_v_charge_state");
    OvmsMetricFloat*  ms_v_charge_power       = new OvmsMetricFloat("ms_v_charge_power");
    OvmsMetricFloat*  ms_v_charge_voltage     = new OvmsMetricFloat("ms_v_charge_voltage");
    OvmsMetricFloat*  ms_v_charge_current     = new OvmsMetricFloat("ms_v_charge_current");
    OvmsMetricFloat*  ms_v_charge_kwh         = new OvmsMetricFloat("ms_v_charge_kwh");
    OvmsMetricFloat*  ms_v_charge_kwh_grid    = new OvmsMetricFloat("ms_v_charge_kwh_grid");
    // TPMS (vector)
    OvmsMetricVector* ms_v_tpms_pressure      = new OvmsMetricVector("ms_v_tpms_pressure");
    OvmsMetricVector* ms_v_tpms_temp          = new OvmsMetricVector("ms_v_tpms_temp");
    OvmsMetricVector* ms_v_tpms_alert         = new OvmsMetricVector("ms_v_tpms_alert");
    // Identity / time
    OvmsMetricString* ms_v_vin                = new OvmsMetricString("ms_v_vin");
    OvmsMetricInt*    ms_m_monotonic          = new OvmsMetricInt("ms_m_monotonic");
};
extern StandardMetricsType StandardMetrics;
#define StdMetrics StandardMetrics

// ---------------------------------------------------------------------------
// Metrics manager — Init* return owning metric pointers (leaked; fine for tests)
// ---------------------------------------------------------------------------
struct OvmsMetricsManager {
    OvmsMetricInt*    InitInt(const char* n, uint16_t = 0, int = 0, metric_unit_t = Other, bool = false)         { return new OvmsMetricInt(n); }
    OvmsMetricFloat*  InitFloat(const char* n, uint16_t = 0, float = 0, metric_unit_t = Other, bool = false)     { return new OvmsMetricFloat(n); }
    OvmsMetricBool*   InitBool(const char* n, uint16_t = 0, bool = false, metric_unit_t = Other, bool = false)   { return new OvmsMetricBool(n); }
    OvmsMetricString* InitString(const char* n, uint16_t = 0, const char* = nullptr, metric_unit_t = Other)      { return new OvmsMetricString(n); }
};
extern OvmsMetricsManager MyMetrics;

// ---------------------------------------------------------------------------
// Config — returns the supplied default for everything.
// ---------------------------------------------------------------------------
struct OvmsConfig {
    void        RegisterParam(const char*, const char*, bool = true, bool = true) {}
    std::string GetParamValue(const char*, const char*, const char* def = "") const { return def; }
    int         GetParamValueInt(const char*, const char*, int def = 0) const       { return def; }
    bool        GetParamValueBool(const char*, const char*, bool def = false) const  { return def; }
    float       GetParamValueFloat(const char*, const char*, float def = 0.0f) const { return def; }
    void        SetParamValue(const char*, const char*, const char*) {}
};
extern OvmsConfig MyConfig;

// ---------------------------------------------------------------------------
// Poller framework
// ---------------------------------------------------------------------------
#define VEHICLE_POLL_NSTATES 16
enum { VEHICLE_POLL_TYPE_OBDIISESSION = 0x10, VEHICLE_POLL_TYPE_READDATA = 0x22, VEHICLE_POLL_TYPE_TESTERPRESENT = 0x3E };
enum { ISOTP_STD = 0, ISOTP_EXTADR = 1, ISOTP_EXTFRAME = 2, VWTP_20 = 3 };
enum { POLLSINGLE_OK = 0, POLLSINGLE_TIMEOUT = -1, POLLSINGLE_SENDERR = -2 };

struct OvmsPoller {
    typedef struct {
        uint32_t txmoduleid;
        uint32_t rxmoduleid;
        uint16_t type;
        uint16_t pid;
        uint16_t polltime[VEHICLE_POLL_NSTATES];
        uint8_t  pollbus;
        uint8_t  protocol;
    } poll_pid_t;

    typedef struct {
        canbus*  bus;
        uint8_t  bus_no;
        uint32_t protocol;
        uint16_t type;
        uint16_t pid;
        uint32_t moduleid_sent;
        uint32_t moduleid_low;
        uint32_t moduleid_high;
        uint32_t moduleid_rec;
        uint16_t mlframe;
        uint16_t mloffset;
        uint16_t mlremain;
        uint32_t ticker;
    } poll_job_t;

    struct OnceOffPoll {
        template <typename... A> OnceOffPoll(A&&...) {}
    };
};

#define POLL_LIST_END { 0, 0, 0x00, 0x00, { 0, 0, 0 }, 0, 0 }

// ---------------------------------------------------------------------------
// OvmsVehicle base — only the surface vehicle modules actually call.
// ---------------------------------------------------------------------------
enum vehicle_command_t { Success, Fail, NotImplemented };

struct OvmsVehicle {
    using vehicle_command_t = ::vehicle_command_t;

    int     m_poll_state = 0;
    canbus* m_can1 = nullptr;
    canbus* m_can2 = nullptr;
    canbus* m_can3 = nullptr;

    OvmsVehicle() { static canbus dummy; m_can1 = m_can2 = m_can3 = &dummy; }
    virtual ~OvmsVehicle() = default;

    void RegisterCanBus(int, CAN_mode_t, CAN_speed_t, void* = nullptr, bool = true) {}

    void PollSetState(int s)  { m_poll_state = s; }
    void PollSetThrottling(int) {}
    void PollSetPidList(canbus*, const OvmsPoller::poll_pid_t*) {}
    template <typename... A> void PollRequest(A&&...) {}
    template <typename... A> int  PollSingleRequest(A&&...) { return POLLSINGLE_TIMEOUT; }

    void BmsSetCellArrangementVoltage(int, int) {}
    void BmsSetCellArrangementTemperature(int, int) {}
    void BmsSetCellVoltage(int, float) {}
    void BmsSetCellTemperature(int, float) {}
    void BmsRestartCellVoltages() {}
    void BmsRestartCellTemperatures() {}
    int  BmsGetCellArangementVoltage(int* = nullptr, int* = nullptr)     { return 0; }
    int  BmsGetCellArangementTemperature(int* = nullptr, int* = nullptr) { return 0; }

    virtual void IncomingPollReply(const OvmsPoller::poll_job_t&, uint8_t*, uint8_t) {}
    virtual void IncomingFrameCan1(CAN_frame_t*) {}
    virtual void IncomingFrameCan2(CAN_frame_t*) {}
    virtual void IncomingFrameCan3(CAN_frame_t*) {}
    virtual void Ticker1(uint32_t) {}
};
