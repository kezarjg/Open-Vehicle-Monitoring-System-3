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
#include "vehicle.h"

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

    void Ticker1(uint32_t ticker);

    void IncomingPollReply(const OvmsPoller::poll_job_t &job, uint8_t* data, uint8_t length) override;

    void IncomingFrameCan2(CAN_frame_t* p_frame) override;

protected:
    std::string m_rxbuf;

    bool m_allow_wake = true;  // Used to implement a cooldown timer if the vehicle is put into sleep
    int m_sleep_entry_time = 0;  // Used to track the time that cooldown timer started

    bool m_armed_for_charge = false;   // charge lid seen open since entering AWAKE
    int  m_cable_watch_start = 0;      // monotonic s when armed (15-min cable watch)
    int  m_charge_state_entry = 0;     // monotonic s of last charge-state entry (handshake 60s timer)

    struct ChargeSessionState {
        bool in_session = false;
        int  start_monotonic = 0;
        int  start_soc = -1;
    };
    ChargeSessionState m_charge_session;

    int8_t m_tpms_corner[5] = {0};   // slot->corner cache (0 = invalid/unread)

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
    OvmsMetricInt*   m_v_charge_ac_ilim;   // 0x1619 b4-5 current upper limit (raw, scale deferred)
    OvmsMetricInt*   m_v_charge_out;       // 0x161E b1-2 charger output (raw, scale deferred)
    OvmsMetricInt*   m_v_charge_out_tgt;   // 0x161E b3-4 target-from-charger (raw, scale deferred)
    OvmsMetricInt*   m_v_charge_ac_usable; // 0x1665 useable power (raw, scale deferred)
    OvmsMetricBool*  m_v_charge_myroom;   // 0x1692 byte 2 (idx 1) bit 0 = My Room active
    OvmsMetricFloat* m_v_charge_acpwr;    // 0x106E A/C consumption power (kW)
    OvmsMetricInt*   m_v_charge_outcome;  // 0x1688 charging history / outcome enum
    OvmsMetricInt*   m_v_charge_stopreq;  // 0x1667 charge seq stop request (enum, partial)
    OvmsMetricBool* m_v_bat_heater_status;
    OvmsMetricFloat* m_v_bat_soc_bms;
    OvmsMetricFloat* m_v_bat_speed_water_pump;
    OvmsMetricFloat* m_v_bat_temp_coolant;
    OvmsMetricFloat* m_v_bat_temp_heater;
    OvmsMetricInt* m_v_env_awaketime;
    OvmsMetricFloat* m_v_pos_trip_start;
    
    void NotifyVehicleOn();
    void NotifyChargeStart();

private:
    static constexpr const char* TAG = "v-toyota-etnga";
    static constexpr const char* CHARGING_TAG = "v-toyota-etnga-charging";
    uint32_t lastBatteryEnergyLogTime;
    uint32_t lastChargerEnergyLogTime;
    uint32_t lastGridEnergyLogTime;

    void InitializeMetrics();  // Initializes the metrics specific to this vehicle module
    void ResetStaleMetrics();  // Checks if state transition metrics are stale (and resets them)

    // Incoming message handling functions
    void IncomingAirConditionerSystem(uint16_t pid);
    void IncomingHPCMHybridPtCtr(uint16_t pid);
    void IncomingHybridBatterySystem(uint16_t pid);
    void IncomingHybridControlSystem(uint16_t pid);
    void IncomingPlugInControlSystem(uint16_t pid);
    void IncomingTPMS(uint16_t pid);

    // Data calculation functions
    float CalculateAmbientTemperature(const std::string& data);
    float CalculateAmbientTemperatureEV(const std::string& data);
    std::vector<float> CalculateBatteryCellVoltages(const std::string& data);
    float CalculateBatteryChargingPower(const std::string& data);
    float CalculateBatteryCurrent(const std::string& data);
    float CalculateBatteryPower(float voltage, float current);
    float CalculateBatterySOC(const std::string& data);
    float CalculateBatterySOCBMS(const std::string& data);
    std::vector<float> CalculateBatteryTemperatures(const std::string& data);
    float CalculateBatteryVoltage(const std::string& data);
    float CalculateCabinTemperature(const std::string& data);
    int CalculateAcOpStatus(const std::string& data);
    int CalculateChargeMode(const std::string& data);
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
    int   CalculateAcCurrentLimitRaw(const std::string& data);
    int   CalculateChargerOutputRaw(const std::string& data);
    int   CalculateChargerOutputTargetRaw(const std::string& data);
    int   CalculateAcUsableRaw(const std::string& data);
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
    void SetBatteryCellVoltageStatistics(const std::vector<float>& voltages);
    void SetBatteryTemperatures(const std::vector<float>& temperatures);
    void SetBatteryTemperatureStatistics(const std::vector<float>& temperatures);
    void SetBatteryVoltage(float voltage);
    void SetCabinTemperature(float temperature);
    void SetChargeMode(int chargeMode);
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
    void SetAcCurrentLimitRaw(int v);
    void SetChargerOutputRaw(int v);
    void SetChargerOutputTargetRaw(int v);
    void SetAcUsableRaw(int v);
    void SetMyRoom(bool active);
    void SetAcConsumption(float kw);
    void SetChargeOutcome(int v);
    void SetChargeStopReq(int v);

    void LogMetricChange(OvmsMetricBool* metric, bool newValue, const std::string& label, const std::string& valueLabel);
    void LogMetricChange(OvmsMetricFloat* metric, float newValue, const std::string& label,const std::string& units);
    void LogMetricChange(OvmsMetricInt* metric, int newValue, const std::string& label, const std::string& valueLabel);
    void LogMetricChange(OvmsMetricString* metric, const std::string& newValue, const std::string& label);
    
    // State transition functions
    void HandleSleepState();
    void HandleAwakeState();
    void HandleDrivingState();
    void HandleChargeHandshakeState();
    void HandleChargeWaitState();
    void HandleChargeAcState();
    void HandleChargeDcState();
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
    void RequestChargeMode();
    void RequestChargeType();
    void DiagnosticSession();
    
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
    TPMS_GW_TX = 0x75002A,   // gateway 0x750, sub-target 0x2A (ISOTP_EXTADR mixed addressing)
    TPMS_GW_RX = 0x75802A,   // gateway response 0x758, sub-target 0x2A
};

// CAN PIDs
enum CANPID
{
    PID_ACTIVE_DIAGNOSTIC_SESSION = 0xF186,
    PID_AC_INPUT_CURRENT = 0x1654,
    PID_AMBIENT_TEMPERATURE = 0x1002,
    PID_AMBIENT_TEMPERATURE_EV = 0x1F46,
    PID_BATTERY_CAPACITY = 0x1D3E,
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
    PID_CHARGING_CONTROL_STATUS = 0x1668,
    PID_CHARGING_LID = 0x1625,
    PID_CHARGING_VOLTAGE_TYPE = 0x161C,
    PID_HVAC_SETPOINT = 0x1036,
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

    PID_CHARGER_STATE_CLUSTER = 0x1619,   // AC-only: b1-2 target power (biased-32768 x0.01kW), b3 op status, b4-5 current limit (raw)
    PID_CHARGER_OUTPUT_POWER = 0x161E,    // AC-only: b1-2 output (raw), b3-4 target-from-charger (raw)
    PID_AC_USABLE_POWER = 0x1665,         // AC-only: u8 useable power (raw)

    PID_AC_CONSUMPTION = 0x106E,    // A/C consumption power: u8 x0.05 kW/LSB (50 W/LSB) — cabin draw, OBC view
    PID_CHARGE_STOP_REQ = 0x1667,   // Charge Seq Stop Request from CCM: u8 enum (0x00 Normal / 0x06 HLC-error; partial)
    PID_CHARGE_HISTORY = 0x1688,    // Charging History (outcome/stop-reason): u8 26-state enum
    PID_MYROOM = 0x1692,            // byte 2 bit 0 = My Room active flag (live)
    PID_TPMS_PRESSURES = 0x1005,  // gateway 0x2A: 5x u16 [status][raw]; psi = raw*0.25 - 7.35
    PID_TPMS_TEMPS = 0x1004,      // gateway 0x2A: 5x u8;  C = raw - 40
    PID_TPMS_CORNERS = 0x2021,    // gateway 0x2A: 5x u8 corner enum (0 none/1 FL/2 FR/3 RL/4 RR)

};

// RX buffer access functions

inline uint8_t GetRxBByte(const std::string& rxbuf, size_t index)
{
    return static_cast<uint8_t>(rxbuf[index]);
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
    const char* pollStateText;

    switch (state) {
        case (PollState::SLEEP):
            pollStateText = "SLEEP";
            break;
        case (PollState::AWAKE):
            pollStateText = "AWAKE";
            break;
        case (PollState::DRIVING):
            pollStateText = "DRIVING";
            break;
        case (PollState::CHARGE_HANDSHAKE):
            pollStateText = "CHARGE_HANDSHAKE";
            break;
        case (PollState::CHARGE_WAIT):
            pollStateText = "CHARGE_WAIT";
            break;
        case (PollState::CHARGE_AC):
            pollStateText = "CHARGE_AC";
            break;
        case (PollState::CHARGE_DC):
            pollStateText = "CHARGE_DC";
            break;
        default:
            pollStateText = "UNKNOWN";
            break;
    }

    return pollStateText;
}

#endif // __VEHICLE_TOYOTA_ETNGA_H__
