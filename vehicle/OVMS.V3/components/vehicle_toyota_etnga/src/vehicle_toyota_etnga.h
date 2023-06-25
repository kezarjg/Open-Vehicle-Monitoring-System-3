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

class OvmsVehicleToyotaETNGA : public OvmsVehicle
{
public:
    OvmsVehicleToyotaETNGA();
    ~OvmsVehicleToyotaETNGA();

    void Ticker1(uint32_t ticker);
    void Ticker60(uint32_t ticker);

    void IncomingPollReply(canbus* bus, uint16_t type, uint16_t pid, uint8_t* data, uint8_t length, uint16_t mlremain);

    void IncomingFrameCan2(CAN_frame_t* p_frame);

protected:
    std::string m_rxbuf;

    OvmsMetricInt* m_s_pollstate;
    OvmsMetricBool* m_v_bat_heater_status;
    OvmsMetricFloat* m_v_bat_soc_bms;
    OvmsMetricFloat* m_v_bat_speed_water_pump;
    OvmsMetricFloat* m_v_bat_temp_coolant;
    OvmsMetricFloat* m_v_bat_temp_heater;
    OvmsMetricFloat* m_v_pos_trip_start;
    
    void NotifyVehicleOn();
    void NotifyChargeStart();

private:
    static constexpr const char* TAG = "v-toyota-etnga";
    bool isNewChargeSession = false;
    uint32_t lastBatteryEnergyLogTime;
    uint32_t lastChargerEnergyLogTime;
    uint32_t lastGridEnergyLogTime;

    void InitializeMetrics();  // Initializes the metrics specific to this vehicle module
    void ResetStaleMetrics();  // Checks if state transition metrics are stale (and resets them)

    // Incoming message handling functions
    void IncomingHPCMHybridPtCtr(uint16_t pid);
    void IncomingHybridBatterySystem(uint16_t pid);
    void IncomingHybridControlSystem(uint16_t pid);
    void IncomingPlugInControlSystem(uint16_t pid);

    // Data calculation functions
    float CalculateAmbientTemperature(const std::string& data);
    float CalculateBatteryChargingPower(const std::string& data);
    float CalculateBatteryCurrent(const std::string& data);
    float CalculateBatteryPower(float voltage, float current);
    float CalculateBatterySOC(const std::string& data);
    std::vector<float> CalculateBatteryTemperatures(const std::string& data);
    float CalculateBatteryVoltage(const std::string& data);
    float CalculateChargerInputPower(const std::string& data);
    bool CalculateChargingDoorStatus(const std::string& data);
    bool CalculateChargingStatus(const std::string& data);
    float CalculateOdometer(const std::string& data);
    bool CalculatePISWStatus(const std::string& data);
    bool CalculateReadyStatus(const std::string& data);
    int CalculateShiftPosition(const std::string& data);
    float CalculateVehicleSpeed(const std::string& data);

    // Metric setter functions
    void SetAmbientTemperature(float temperature);
    void SetAwake(bool awake);
    void SetBatteryChargingPower(float power);
    void SetBatteryCurrent(float current);
    void SetBatteryPower(float power);
    void SetBatterySOC(float soc);
    void SetBatteryTemperatures(const std::vector<float>& temperatures);
    void SetBatteryTemperatureStatistics(const std::vector<float>& temperatures);
    void SetBatteryVoltage(float voltage);
    void SetChargeMode(int chargeMode);
    void SetChargeType(int chargeType);
    void SetChargeState(std::string chargeState);
    void SetChargerInputPower(float power);
    void SetChargingDoorStatus(bool status);
    void SetChargingStatus(bool status);
    void SetOdometer(float odometer);
    void SetPISWStatus(bool status);
    void SetPollState(int state);
    void SetReadyStatus(bool status);
    void SetShiftPosition(int position);
    void SetVehicleSpeed(float speed);
    void SetVehicleVIN(std::string vin);

    // State transition functions
    void HandleSleepState();
    void HandleAwakeState();
    void HandleReadyState();
    void HandleChargingState();
    void TransitionToSleepState();
    void TransitionToAwakeState();
    void TransitionToReadyState();
    void TransitionToChargingState();

    void RequestVIN();
    void RequestChargeMode();
    void RequestChargeType();
    
};

// Poll states
enum PollState
{
    SLEEP,
    AWAKE,
    READY,
    CHARGING
};

// CAN bus addresses
enum CANAddress
{
    HYBRID_BATTERY_SYSTEM_TX = 0x747,
    HYBRID_BATTERY_SYSTEM_RX = 0x74F,
    HYBRID_CONTROL_SYSTEM_TX = 0x7D2,
    HYBRID_CONTROL_SYSTEM_RX = 0x7DA,
    PLUG_IN_CONTROL_SYSTEM_TX = 0x745,
    PLUG_IN_CONTROL_SYSTEM_RX = 0x74D,
    HPCM_HYBRIDPTCTR_RX = 0x7EA
};

// CAN PIDs
enum CANPID
{
    PID_AMBIENT_TEMPERATURE = 0x1F46,
    PID_BATTERY_CHARGING_POWER = 0x10D4,
    PID_BATTERY_COOLANT_TEMPERATURE = 0x1848,
    PID_BATTERY_HEATER_STATUS = 0x2806,
    PID_BATTERY_HEATER_TEMPERATURE = 0x1824,
    PID_BATTERY_TEMPERATURES = 0x1814,
    PID_BATTERY_SOC = 0x1738,
    PID_BATTERY_SOC_BMS = 0x1F5B,
    PID_BATTERY_WATER_PUMP_SPEED = 0x110E,
    PID_BATTERY_VOLTAGE_AND_CURRENT = 0x1F9A,
    PID_CHARGING = 0x10D1,
    PID_CHARGING_CONTROL_STATUS = 0x1668,
    PID_CHARGING_LID = 0x1625,
    PID_CHARGING_VOLTAGE_TYPE = 0x161C,
    PID_ODOMETER = 0x1FA6,
    PID_PISW_STATUS = 0x1669,
    PID_READY_SIGNAL = 0x1076,
    PID_SHIFT_POSITION = 0x1061,
    PID_VEHICLE_SPEED = 0x1F0D,
    PID_VIN = 0xF190,

    PID_CHARGER_INPUT_POWER = 0x161D,
    PID_AC_INPUT_CURRENT = 0x1654,

    
    PID_DC_CHARGER_PRESENT_CURRENT = 0x166C,

    PID_DC_CHARGER_PRESENT_VOLTAGE = 0x166B

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
        case (PollState::READY):
            pollStateText = "READY";
            break;
        case (PollState::CHARGING):
            pollStateText = "CHARGING";
            break;
        default:
            pollStateText = "UNKNOWN";
            break;
    }

    return pollStateText;
}

#endif // __VEHICLE_TOYOTA_ETNGA_H__
