/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Toyota e-TNGA platform
   Date:          4th June 2023

   (C) 2023       Jerry Kezar <solterra@kezarnet.com>

   Licensed under the MIT License. See the LICENSE file for details.
*/

#ifndef __VEHICLE_TOYOTA_ETNGA_H__
#define __VEHICLE_TOYOTA_ETNGA_H__

#include <string>
#include <vector>
#include <utility>
#include <iosfwd>
#include <time.h>
#include "vehicle.h"
#ifdef CONFIG_OVMS_COMP_WEBSERVER
#include "ovms_webserver.h"
#endif

// Control states
enum ControlState {
    CS_NONE = 0,
    CS_DRIVING = 1,
    CS_CHARGING = 3
};

// Poll states
enum PollState : int
{
    SLEEP            = 0,
    AWAKE            = 1,
    DRIVING          = 2,
    CHARGE_HANDSHAKE = 3,  // was CHARGING; cable-negotiation fast-poll window
    CHARGE_WAIT      = 4,  // plugged in, not (yet/any longer) charging — sparse
    CHARGE_AC        = 5,
    CHARGE_DC        = 6,
};

class OvmsVehicleToyotaETNGA : public OvmsVehicle
{
public:
    OvmsVehicleToyotaETNGA();
    ~OvmsVehicleToyotaETNGA();

    void Ticker1(uint32_t ticker) override;

    void IncomingPollReply(const OvmsPoller::poll_job_t &job, uint8_t* data, uint8_t length) override;
    void IncomingPollError(const OvmsPoller::poll_job_t &job, int32_t code) override;

    void IncomingFrameCan2(CAN_frame_t* p_frame) override;

#ifdef CONFIG_OVMS_COMP_WEBSERVER
    // Webserver subsystem (implementation: etnga_web.cpp)
    void WebInit();
    void WebDeInit();
    static void WebCfgFeatures(PageEntry_t& p, PageContext_t& c);
    static void WebDispChgMetrics(PageEntry_t& p, PageContext_t& c);
    static void WebChgRenderAc(PageContext_t& c, OvmsVehicleToyotaETNGA* v);   // AC charging panels
    static void WebChgRenderDc(PageContext_t& c, OvmsVehicleToyotaETNGA* v);   // DC charging panels
    static void WebChgChartJs(PageContext_t& c, OvmsVehicleToyotaETNGA* v, bool dc);          // live chart
    static void WebChgStateHistoryJs(PageContext_t& c, OvmsVehicleToyotaETNGA* v, bool dc);   // live state history
    static void WebChargeReports(PageEntry_t& p, PageContext_t& c);   // index of saved charge reports
    static void WebChargeReport(PageEntry_t& p, PageContext_t& c);    // stream one report (raw HTML)
#endif // CONFIG_OVMS_COMP_WEBSERVER

protected:
    std::string m_rxbuf;

    bool m_allow_wake = true;  // Used to implement a cooldown timer if the vehicle is put into sleep
    int m_sleep_entry_time = 0;  // Used to track the time that cooldown timer started
    int m_sleep_cooldown_secs = 10;  // Cooldown window (s) for the current sleep; default must equal SLEEP_COOLDOWN_SECS[0]
    int m_sleep_backoff_idx = 0;     // Index into SLEEP_COOLDOWN_SECS; escalates on consecutive no-activity sleeps
    bool m_12v_was_high = false;     // 12V-above-threshold latch: the SLEEP 12V wake fires on the rising edge only
                                     // (level-triggering oscillated; seeded on each sleep entry in TransitionToSleepState)

    bool m_armed_for_charge = false;   // charge lid seen open since entering AWAKE
    int  m_cable_watch_start = 0;      // monotonic s when armed (15-min cable watch)
    int  m_charge_state_entry = 0;     // monotonic s of last charge-state entry (handshake 60s timer)

    struct ChargeSessionState {
        bool  in_session = false;
        int   start_monotonic = 0;
        int   start_soc = -1;
        time_t start_utc = 0;
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
        float station_kwh = 0.0f;   // ∫ station_kw dt — energy drawn from the EVSE this session
        int   last_sample_monotonic = 0;   // dt for delivered_ah + CSV row cadence
        // v2: event log (monotonic seconds, static label string)
        std::vector<std::pair<int,const char*>> events;
        int   last_hlc = -1;               // last 0x1666 HLC state logged as an event (change detection)
        int   last_acop = -1;              // last 0x1684 AC-Op state logged as an event (change detection)
        // v2: downsampled chart buffer (per sample: delivered kW, SOC, station-offered + car-permitted kW)
        struct Sample { int t_s; float kw; int soc; float sta_max; float car_perm; float station_kw; float hvac_kw; };
        std::vector<Sample> svg;
        int   svg_interval_s = 20;
        int   last_svg_monotonic = 0;
        // v2: file basename (resolved "<dir>/<timestamp>", no extension) + CSV state
        std::string base;
        bool  csv_started = false;        // header emitted (into csv_buf)
        bool  csv_file_created = false;   // <base>.csv exists on disk (first flush truncates)
        std::string csv_buf;              // rows pending flush (batched to limit flash write cycles)
        int   last_csv_flush = 0;         // monotonic s of last flush
    };
    ChargeSessionState m_charge_session;

    static constexpr int TPMS_SLOT_COUNT = 5;
    int8_t m_tpms_corner[TPMS_SLOT_COUNT] = {0};   // slot->corner cache: 0=unread/none, 1=FL,2=FR,3=RL,4=RR

//    ControlState m_s_controlstate;
    OvmsMetricInt* m_s_controlstate;
    OvmsMetricInt* m_v_charge_pisw_raw;   // 0x1669 raw u8 connector state
    OvmsMetricInt* m_v_charge_ac_op;      // 0x1684 AC op status
    OvmsMetricInt* m_v_charge_hlc;        // 0x1666 DC HLC state
    OvmsMetricFloat* m_v_charge_perm;     // 0x16A1 min permission power (kW, s16 two's-comp, NOT biased-32768) — DC curve
    OvmsMetricFloat* m_v_charge_tgti;     // 0x166D target charging current (A)
    OvmsMetricFloat* m_v_charge_sta_max_p;  // 0x166A station max power (kW)
    OvmsMetricFloat* m_v_charge_sta_max_i;  // 0x1679 station max current (A)
    OvmsMetricFloat* m_v_charge_sta_max_v;  // 0x1681 station max voltage (V)
    OvmsMetricFloat* m_v_charge_ac_tgt_p;  // 0x1619 b1-2 AC target power (kW)
    OvmsMetricInt*   m_v_charge_chgr_op;   // 0x1619 b3 charger op status (enum) — distinct from m_v_charge_ac_op (0x1684)
    OvmsMetricFloat* m_v_charge_ac_ilim;   // 0x1619 b4-5 AC current limit (A)
    OvmsMetricFloat* m_v_charge_out;       // 0x161E b1-2 charger output (kW, unit inferred)
    OvmsMetricFloat* m_v_charge_out_tgt;   // 0x161E b3-4 target-from-charger (kW, unit inferred)
    OvmsMetricFloat* m_v_charge_ac_usable; // 0x1665 useable power (kW, unit inferred)
    OvmsMetricBool*  m_v_charge_myroom;   // 0x1692 byte 2 (idx 1) bit 0 = My Room active
    OvmsMetricFloat* m_v_charge_grid_power;  // 0x161D AC charger/grid input power (kW) — live, for CSV/efficiency
    OvmsMetricFloat* m_v_env_hvac_power;  // 0x106E HVAC/cabin power draw (kW): OBC view (0x745) while charging, hybrid-control view (0x7D2) while driving
    OvmsMetricFloat* m_v_env_hvac_kwh;    // My-Room cabin energy (kWh): time-integral of m_v_env_hvac_power over the My-Room-active interval
    OvmsMetricFloat* m_v_env_hvac_kwh_drive;  // Driving cabin/HVAC energy (kWh): per-trip time-integral of m_v_env_hvac_power while DRIVING (reset in NotifyVehicleOn)
    OvmsMetricInt*   m_v_charge_outcome;  // 0x1688 charging history / outcome enum
    OvmsMetricInt*   m_v_charge_stopreq;  // 0x1667 charge seq stop request (enum, partial)
    OvmsMetricVector<float>* m_v_bat_cap_full;  // 0x1D3E 8x u16 x0.01 Ah — per-module full-charge capacity (data collection)
    OvmsMetricVector<float>* m_v_bat_cap_alt;   // 0x1D3F 8x u16 x0.01 Ah — parallel array, function unconfirmed (data collection)
    OvmsMetricBool* m_v_bat_heater_status;
    OvmsMetricFloat* m_v_bat_soc_bms;
    OvmsMetricFloat* m_v_bat_temp_coolant;
    OvmsMetricFloat* m_v_bat_temp_heater;
    // Internal bookkeeping (not exposed as metrics):
    int   m_awake_entered = 0;        // ms_m_monotonic seconds when AWAKE was entered (awake-timeout watchdog)
    float m_trip_start_odo = 0.0f;    // odometer baseline at trip start
    bool  m_trip_start_valid = false; // false until the baseline is seeded; reset on transition to DRIVING
    OvmsMetricInt* m_v_e_awd;   // 0x1087 b2 AWD / X-MODE status (custom; no standard OVMS metric)
    
    void NotifyVehicleOn() override;
    // NotifyChargeStart deliberately not overridden — see vehicle_toyota_etnga.cpp;
    // session counters reset at session open in TransitionToChargeHandshakeState.

private:
    static constexpr const char* TAG = "v-toyota-etnga";
    static constexpr const char* CHARGING_TAG = "v-toyota-etnga-charging";
    // Energy-integrator timestamps (esp_log_timestamp ms; 0 = interval not started).
    // Must be zero-initialized: the first poll reply can arrive before NotifyVehicleOn /
    // NotifyChargeStart reset them, and a garbage dt would corrupt the persistent *_total metrics.
    uint32_t lastBatteryEnergyLogTime = 0;
    uint32_t lastChargerEnergyLogTime = 0;
    float m_charge_obc_kw = 0.0f;   // diagnostic: raw 0x10D4 OBC "battery charging power" (under-reads on DC, issue #109)
    uint32_t lastGridEnergyLogTime = 0;
    uint32_t lastHvacEnergyLogTime = 0;
    uint32_t lastHvacDriveEnergyLogTime = 0;

    // XOR-folded signature of the last poll error logged at WARN (module/pid/code — see
    // IncomingPollError): repeats of the same error (e.g. one TX failure per poll while
    // the bus is wedged) drop to debug level until the signature changes.
    uint32_t m_last_poll_error = 0;

    void InitializeMetrics();  // Initializes the metrics specific to this vehicle module
    void ResetStaleMetrics();  // Checks if state transition metrics are stale (and resets them)

    // Charge session report (etnga_charge_report.cpp)
    void UpdateChargeSessionStats();   // live aggregation while charging (peak power, temp range, type)
    void RenderPowerSvg(std::ostream& out);  // stream the inline SVG power(+SOC)-vs-time chart from m_charge_session.svg
    void GenerateChargeReport();       // write the session-end HTML report to /store/charge-reports/
    void LogChargeEvent(const char* label);            // append a timestamped event
    void AppendChargeCsvRow();                          // buffer one CSV row (header on first call)
    void FlushChargeCsv();                              // write buffered CSV rows to <base>.csv
    std::string ChargeReportDir();                      // "/sd/charge-reports" if SD mounted else "/store/..."
    static const char* ChargeOutcomeLabel(int code);    // 0x1688 enum -> human text
    static const char* HlcStateLabel(int code);         // 0x1666 DC HLC state enum -> human text ("" if unknown)
    static const char* AcOpStatusLabel(int code);       // 0x1684 AC-Op state enum -> human text ("" for Stop/unknown)
    std::string LookupLocationName(float lat, float lon); // matching OVMS named-location (geofence), or ""

    // Incoming message handling functions
    void IncomingAirConditionerSystem(uint16_t pid);
    void IncomingHPCMHybridPtCtr(uint16_t pid);
    void IncomingHybridBatterySystem(uint16_t pid);
    void IncomingHybridControlSystem(uint16_t pid);
    void IncomingPlugInControlSystem(uint16_t pid);
    void IncomingBrakeEpb(uint16_t pid);
    void IncomingTPMS(uint16_t pid);
    void UpdateTPMSAlert();
    bool TPMSCornerMapValid();

    // Data calculation functions
    float CalculateAmbientTemperature(const std::string& data);
    float CalculateAmbientTemperatureEV(const std::string& data);
    std::vector<float> CalculateBatteryCellVoltages(const std::string& data);
    std::vector<float> CalculateBatteryCapacityArray(const std::string& data);
    float CalculateBatteryChargingPower(const std::string& data);
    float CalculateBatteryCurrent(const std::string& data);
    float CalculateBatteryPower(float voltage, float current);
    float CalculateBatterySOC(const std::string& data);
    float CalculateBatterySOCBMS(const std::string& data);
    std::vector<float> CalculateBatteryTemperatures(const std::string& data);
    float CalculateBatteryVoltage(const std::string& data);
    float CalculateCabinTemperature(const std::string& data);
    int CalculateAcOpStatus(const std::string& data);
    int CalculateChargeType(const std::string& data);
    int CalculateHlcState(const std::string& data);
    int CalculatePISWRaw(const std::string& data);
    float CalculatePermissionPower(const std::string& data);
    float CalculateTargetCurrent(const std::string& data);
    float CalculateStationVoltage(const std::string& data);
    float CalculateStationCurrent(const std::string& data);
    float CalculateStationMaxPower(const std::string& data);
    float CalculateStationMaxCurrent(const std::string& data);
    float CalculateStationMaxVoltage(const std::string& data);
    float CalculateChargerInputPower(const std::string& data);
    float CalculateAcTargetPower(const std::string& data);
    int   CalculateChargerOpStatus(const std::string& data);
    float CalculateAcCurrentLimit(const std::string& data);
    float CalculateChargerOutput(const std::string& data);
    float CalculateChargerOutputTarget(const std::string& data);
    float CalculateAcUsable(const std::string& data);
    bool  CalculateMyRoom(const std::string& data);
    float CalculateAcConsumption(const std::string& data);
    int   CalculateChargeOutcome(const std::string& data);
    int   CalculateChargeStopReq(const std::string& data);
    bool CalculateChargingDoorStatus(const std::string& data);
    int CalculateControlMode(const std::string& data);
    float CalculateHVACSetpoint(const std::string& data);
    float CalculateOdometer(const std::string& data);
    bool CalculatePISWStatus(const std::string& data);
    bool CalculateReadyStatus(const std::string& data);
    int CalculateShiftPosition(const std::string& data);
    float CalculateVehicleSpeed(const std::string& data);
    float CalculateThrottle(const std::string& data);
    int   CalculateDriveMode(const std::string& data);
    int   CalculateAwdMode(const std::string& data);
    float CalculateFootBrake(const std::string& data);
    bool  CalculateParkBrake(const std::string& data);

    // Metric setter functions
    void SetAcOpStatus(int v);
    void SetAmbientTemperature(float temperature);
    void SetAwake(bool awake);
    void SetBatteryChargingPower(float power);
    void SetBatteryCurrent(float current);
    void SetBatteryPower(float power);
    void SetBatterySOC(float soc);
    void SetBatterySOCBMS(float soc);
    void SetBatteryCellVoltages(const std::vector<float>& voltages);
    void SetBatteryCapacityFull(const std::vector<float>& caps);
    void SetBatteryCapacityAlt(const std::vector<float>& caps);
    void SetBatteryCellVoltageStatistics(const std::vector<float>& voltages);
    void SetBatteryTemperatures(const std::vector<float>& temperatures);
    void SetBatteryTemperatureStatistics(const std::vector<float>& temperatures);
    void SetBatteryVoltage(float voltage);
    void SetCabinTemperature(float temperature);
    void SetChargeType(int chargeType);
    void SetChargeState(PollState state);
    void SetChargerInputPower(float power);
    void SetChargingStatus(bool status);
    void SetChargingDoorStatus(bool status);
    void SetControlMode(int controlMode);
    void SetHlcState(int v);
    void SetHVACSetpoint(float temperature);
    void SetOdometer(float odometer);
    void SetPISWRaw(int v);
    void SetPISWStatus(bool status);
    void SetPollState(int state);
    void SetReadyStatus(bool status);
    void SetShiftPosition(int position);
    void SetVehicleSpeed(float speed);
    void SetVehicleVIN(std::string vin);
    void SetStationVoltage(float volts);
    void SetStationCurrent(float amps);
    void SetPermissionPower(float kw);
    void SetTargetCurrent(float amps);
    void SetStationMaxPower(float kw);
    void SetStationMaxCurrent(float amps);
    void SetStationMaxVoltage(float volts);
    void SetAcTargetPower(float kw);
    void SetChargerOpStatus(int v);
    void SetAcCurrentLimit(float v);
    void SetChargerOutput(float v);
    void SetChargerOutputTarget(float v);
    void SetAcUsable(float v);
    void SetMyRoom(bool active);
    void SetHvacPower(float kw);
    void SetChargeOutcome(int v);
    void SetChargeStopReq(int v);
    void SetThrottle(float pct);
    void SetDriveMode(int mode);
    void SetAwdMode(int mode);
    void SetFootBrake(float pct);
    void SetParkBrake(bool applied);

    // const char* throughout: these run on every poll reply, and std::string
    // parameters meant heap allocations per call even with logging filtered out.
    void LogMetricChange(OvmsMetricBool* metric, bool newValue, const char* label, const char* valueLabel);
    void LogMetricChange(OvmsMetricFloat* metric, float newValue, const char* label, const char* units);
    void LogMetricChange(OvmsMetricInt* metric, int newValue, const char* label, const char* valueLabel);
    void LogMetricChange(OvmsMetricString* metric, const std::string& newValue, const char* label);

    // Hours since the last sample on an energy-integrator channel; updates the channel
    // timestamp. Implementation: etnga_metrics.cpp.
    float EnergyIntervalHours(uint32_t& lastSampleTime);

    // State transition functions
    void HandleSleepState();
    void HandleAwakeState();
    void HandleDrivingState();
    void HandleChargeHandshakeState();
    void HandleChargeWaitState();
    void HandleChargeAcState();
    void HandleChargeDcState();
    void ResetSleepBackoff();   // reset cooldown escalation to the base step (real activity seen)
    void TransitionToSleepState();
    void TransitionToAwakeState();
    void TransitionToDrivingState();
    void TransitionToChargeHandshakeState();
    void TransitionToChargeWaitState();
    void TransitionToChargeAcState();
    void TransitionToChargeDcState();

    void RequestVIN();
    void IncomingVINSuccess(uint16_t type, uint32_t module_sent, uint32_t module_rec, uint16_t pid, CAN_frame_format_t format, const std::string &data);
    void IncomingVINFail(uint16_t type, uint32_t module_sent, uint32_t module_rec, uint16_t pid, int errorcode);

};

// CAN bus addresses
enum CANAddress
{
    AIR_CONDITIONER_TX = 0x7C4,
    AIR_CONDITIONER_RX = 0x7CC,
    HYBRID_BATTERY_SYSTEM_TX = 0x747,
    HYBRID_BATTERY_SYSTEM_RX = 0x74F,
    HYBRID_CONTROL_SYSTEM_TX = 0x7D2,
    HYBRID_CONTROL_SYSTEM_RX = 0x7DA,
    PLUG_IN_CONTROL_SYSTEM_TX = 0x745,
    PLUG_IN_CONTROL_SYSTEM_RX = 0x74D,
    HPCM_HYBRIDPTCTR_RX = 0x7EA,
    BRAKE_EPB_TX = 0x7B0,    // Brake/EPB ECU (ABS/VSC/TRC + Electric Parking Brake) — direct-poll, standard ISO-TP
    BRAKE_EPB_RX = 0x7B8,
    TPMS_GW_TX = 0x7502A,    // (0x750 << 8) | 0x2A  -> MsgID 0x750, sub 0x2A (ISOTP_EXTADR mixed addressing)
    TPMS_GW_RX = 0x7582A,    // (0x758 << 8) | 0x2A  -> MsgID 0x758, sub 0x2A
};

// CAN PIDs
enum CANPID
{
    PID_ACTIVE_DIAGNOSTIC_SESSION = 0xF186,
    PID_AC_INPUT_CURRENT = 0x1654,
    PID_AMBIENT_TEMPERATURE = 0x1002,
    PID_AMBIENT_TEMPERATURE_EV = 0x1F46,
    PID_BATTERY_CAPACITY = 0x1D3E,        // 8x u16 BE x0.01 Ah — per-module full-charge capacity (data-collection only)
    PID_BATTERY_CAPACITY_ALT = 0x1D3F,    // 8x u16 BE x0.01 Ah — parallel array ~4.5% lower, function unconfirmed (data-collection only)
    PID_BATTERY_CELL_VOLTAGES = 0x182E,
    PID_BATTERY_CHARGING_POWER = 0x10D4,
    PID_BATTERY_COOLANT_TEMPERATURE = 0x1848,
    PID_BATTERY_HEATER_STATUS = 0x2806,
    PID_BATTERY_HEATER_TEMPERATURE = 0x1824,
    PID_BATTERY_TEMPERATURES = 0x1814,
    PID_BATTERY_SOC = 0x1738,
    PID_BATTERY_SOC_BMS = 0x1F5B,
    PID_BATTERY_WATER_PUMP_SPEED = 0x110E,
    PID_BATTERY_VOLTAGE_AND_CURRENT = 0x1F9A,
    PID_CABIN_TEMPERATURE = 0x1001,
    PID_CHARGER_INPUT_POWER = 0x161D,
    PID_CONTROL_SYSTEM_MODE = 0x10D1,
    PID_CHARGING_CONTROL_INFORMATION = 0x1689,
    PID_CHARGING_LID = 0x1625,
    PID_CHARGING_VOLTAGE_TYPE = 0x161C,
    PID_HVAC_SETPOINT = 0x1036,
    PID_HEATER_POWER = 0x1086,            // HV electric heater power (W, 2-byte cluster, split TBD); >0 => v.e.heating
    PID_BLOWER_LEVEL = 0x2801,            // Blower level (u8 1-7) => v.e.cabinfan %
    PID_ODOMETER = 0x1FA6,
    PID_PISW_STATUS = 0x1669,
    PID_READY_SIGNAL = 0x1076,
    PID_SHIFT_POSITION = 0x1061,
    PID_VEHICLE_SPEED = 0x1F0D,
    PID_VIN = 0xF190,
    
    PID_DC_CHARGER_PRESENT_CURRENT = 0x166C,  // u16 BE x1 A/LSB; DC station present current (idle 0 when no station)

    PID_DC_CHARGER_PRESENT_VOLTAGE = 0x166B,  // u16 BE x1 V/LSB; DC station present voltage (idle 0 when no station)

    PID_AC_CHARGING_OP_STATUS = 0x1684,  // 0=Stop,1=Startup,2=Running,3=Finishing
    PID_HLC_STATE = 0x1666,              // DC HLC: 0xFF=Unconnected, 0x0A-0x12 active

    PID_MIN_PERMISSION_POWER = 0x16A1,   // s16 BE x0.01 kW; 0x8000 sentinel = inactive — THE DC taper curve
    PID_TARGET_CHARGING_CURRENT = 0x166D, // u16 BE x1 A; live current request

    PID_DC_CHARGER_MAX_POWER = 0x166A,    // u16 BE x0.01 kW; station advertised max power
    PID_DC_CHARGER_MAX_CURRENT = 0x1679,  // u16 BE x1 A; station advertised max current (CCS)
    PID_DC_CHARGER_MAX_VOLTAGE = 0x1681,  // u16 BE x1 V; station advertised max voltage (CCS)

    PID_CHARGER_STATE_CLUSTER = 0x1619,   // AC-only: b1-2 target power (biased-32768 x0.01 kW), b3 op status enum, b4-5 current limit (biased-32768 x0.01 A)
    PID_CHARGER_OUTPUT_POWER = 0x161E,    // AC-only: b1-2 output (x5/1000 kW, unit inferred), b3-4 target-from-charger (x5/1000 kW, unit inferred)
    PID_AC_USABLE_POWER = 0x1665,         // AC-only: u8 x0.01 kW (unit inferred)

    PID_AC_CONSUMPTION = 0x106E,    // A/C consumption power: u8 x0.05 kW/LSB (50 W/LSB) — cabin draw, OBC view
    PID_CHARGE_STOP_REQ = 0x1667,   // Charge Seq Stop Request from CCM: u8 enum (0x00 Normal / 0x06 HLC-error; partial)
    PID_CHARGE_HISTORY = 0x1688,    // Charging History (outcome/stop-reason): u8 26-state enum
    PID_MYROOM = 0x1692,            // byte 2 bit 0 = My Room active flag (live)
    PID_TPMS_PRESSURES = 0x1005,  // gateway 0x2A: 5x u16 [status][raw]; psi = raw*0.25 - 7.35
    PID_TPMS_TEMPS = 0x1004,      // gateway 0x2A: 5x u8;  C = raw - 40
    PID_TPMS_CORNERS = 0x2021,    // gateway 0x2A: 5x u8 corner enum (0 none/1 FL/2 FR/3 RL/4 RR)

    // 2026-06-06 pins — EV ECU (0x7D2) driver-input signals
    PID_THROTTLE = 0x1060,          // b1: accelerator position, u8 x0.5 %/LSB (0x00-0xC8 -> 0-100%)
    PID_DRIVE_MODE_SELECT = 0x1004, // b1: Eco/Normal/Power enum. NOTE: same numeric value as PID_TPMS_TEMPS,
                                    //     but dispatched on a different ECU (0x7DA vs TPMS gateway) so no conflict.
    PID_AWD_MODE = 0x1087,          // b2: X-MODE/AWD status enum

    // 2026-06-06 pins — Brake/EPB ECU (0x7B0) direct-poll
    PID_BRAKE_PEDAL_STROKE = 0x104C, // b1: brake pedal stroke, u8 ~1 mm/LSB (0 rest .. ~67 full)
    PID_EPB_STATUS = 0x1045,         // b1 = RH actuator status enum; handbrake applied = 0x00 (Park Applied)

};

// RX buffer access functions

inline uint8_t GetRxBByte(const std::string& rxbuf, size_t index)
{
    // Bounds-checked: a short/garbled UDS reply must not read past the buffer.
    // Handlers should still length-check multi-byte payloads before decoding
    // (a zero-fill is memory-safe but not valid data).
    return (index < rxbuf.size()) ? static_cast<uint8_t>(rxbuf[index]) : 0;
}

inline uint16_t GetRxBUint16(const std::string& rxbuf, size_t index)
{
    return (static_cast<uint16_t>(GetRxBByte(rxbuf, index)) << 8) | GetRxBByte(rxbuf, index + 1);
}

inline uint32_t GetRxBUint24(const std::string& rxbuf, size_t index)
{
    return (static_cast<uint32_t>(GetRxBByte(rxbuf, index)) << 16) |
        (static_cast<uint32_t>(GetRxBByte(rxbuf, index + 1)) << 8) |
        GetRxBByte(rxbuf, index + 2);
}

inline uint32_t GetRxBUint32(const std::string& rxbuf, size_t index)
{
    return (static_cast<uint32_t>(GetRxBByte(rxbuf, index)) << 24) |
        (static_cast<uint32_t>(GetRxBByte(rxbuf, index + 1)) << 16) |
        (static_cast<uint32_t>(GetRxBByte(rxbuf, index + 2)) << 8) |
        GetRxBByte(rxbuf, index + 3);
}

inline int8_t GetRxBInt8(const std::string& rxbuf, size_t index)
{
    return static_cast<int8_t>(GetRxBByte(rxbuf, index));
}

inline int16_t GetRxBInt16(const std::string& rxbuf, size_t index)
{
    return static_cast<int16_t>(GetRxBUint16(rxbuf, index));
}

inline int32_t GetRxBInt32(const std::string& rxbuf, size_t index)
{
    return static_cast<int32_t>(GetRxBUint32(rxbuf, index));
}

inline bool GetRxBBit(const std::string& rxbuf, size_t byteIndex, size_t bitIndex)
{
    uint8_t byte = GetRxBByte(rxbuf, byteIndex);
    return (byte & (1 << bitIndex)) != 0;
}

inline const char* ConvertPollStateToString(int state) {
    // Indexed by PollState (SLEEP..CHARGE_DC).
    static const char* const names[] = {
        "SLEEP", "AWAKE", "DRIVING", "CHARGE_HANDSHAKE", "CHARGE_WAIT", "CHARGE_AC", "CHARGE_DC"
    };
    if (state < 0 || state >= (int)(sizeof(names) / sizeof(names[0])))
        return "UNKNOWN";
    return names[state];
}

#endif // __VEHICLE_TOYOTA_ETNGA_H__
