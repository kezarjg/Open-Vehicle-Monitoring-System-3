/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Toyota e-TNGA platform
   Date:          4th June 2023

   (C) 2023       Jerry Kezar <solterra@kezarnet.com>

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include "vehicle_toyota_etnga.h"
#include <algorithm>
#include <cmath>

void OvmsVehicleToyotaETNGA::InitializeMetrics()
{
    m_s_controlstate = MyMetrics.InitInt("xte.s.controlstate", SM_STALE_MIN);  // This variable stores the control state variable
    m_v_charge_pisw_raw = MyMetrics.InitInt("xte.v.c.pisw", SM_STALE_MID);
    m_v_charge_ac_op    = MyMetrics.InitInt("xte.v.c.acop", SM_STALE_MID);
    m_v_charge_hlc      = MyMetrics.InitInt("xte.v.c.hlc",  SM_STALE_MID);
    m_v_charge_perm = MyMetrics.InitFloat("xte.v.c.perm", SM_STALE_MID, 0.0f, kW);
    m_v_charge_tgti = MyMetrics.InitFloat("xte.v.c.tgti", SM_STALE_MID, 0.0f, Amps);
    m_v_bat_heater_status = MyMetrics.InitBool("xte.v.b.heater", SM_STALE_MID);  // This variable stores the status of the battery coolant heater relay
    m_v_bat_soc_bms = MyMetrics.InitFloat("xte.v.b.soc.bms", SM_STALE_MID, 0.0f, Percentage, true);  // This variable stores the SOC as reported by the BMS
    m_v_bat_speed_water_pump = MyMetrics.InitFloat("xte.v.b.speed.waterpump", SM_STALE_MID, 0.0f, Other);  // This variable stores the RPM of the battery water pump
    m_v_bat_temp_coolant = MyMetrics.InitFloat("xte.v.b.temp.coolant", SM_STALE_MID, 0.0f, Celcius);  // This variable stores the temperature of the battery coolant
    m_v_bat_temp_heater = MyMetrics.InitFloat("xte.v.b.temp.heater", SM_STALE_MID, 0.0f, Celcius);  // This variable stores the temperature of the battery coolant
    m_v_pos_trip_start = MyMetrics.InitFloat("xte.v.p.trip.start", SM_STALE_NONE, 0.0f, Kilometers, false);  // This variable stores the odometer reading at the beginning of a trip
    m_v_env_awaketime = MyMetrics.InitInt("xte.v.e.awaketime", SM_STALE_NONE);  // Time awake state was entered for awake timeout (monotonic seconds)

    // Set initial values for metrics
    SetAwake(false);
    SetReadyStatus(false);
    SetChargingDoorStatus(false);

    // Set poll state transition variables to shorter autostale than default
    // in case their ECUs go to sleep before the 'false' poll
    StandardMetrics.ms_v_door_chargeport->SetAutoStale(10);
}

void OvmsVehicleToyotaETNGA::ResetStaleMetrics() // Reset stale variables
{
    // Check to make sure the 'Control Mode' signal has been updated recently
    if (m_s_controlstate->IsStale() && m_s_controlstate->AsInt() != 0) {
        ESP_LOGD(TAG, "Control Mode is stale. Manually setting to off");
        SetControlMode(0);
    }

    // Check to make sure the 'awake' signal has been updated recently
    if (StandardMetrics.ms_v_env_awake->IsStale() && StandardMetrics.ms_v_env_awake->AsBool()) {
        ESP_LOGD(TAG, "Awake is stale. Manually setting to off");
        SetAwake(false);
    }

    // Check to make sure the 'charging door' signal has been updated recently
    if (StandardMetrics.ms_v_door_chargeport->IsStale() && StandardMetrics.ms_v_door_chargeport->AsBool()) {
        ESP_LOGD(TAG, "Charging Door is stale. Manually setting to off");
        SetChargingDoorStatus(false);
    }

    // Check to make sure the 'pilot' signal has been updated recently
    if (StandardMetrics.ms_v_charge_pilot->IsStale() && StandardMetrics.ms_v_charge_pilot->AsBool()) {
        ESP_LOGD(TAG, "Pilot Status is stale. Manually setting to off");
        SetPISWStatus(false);
    }

    // Check to make sure the 'power' has been updated recently
    if (StandardMetrics.ms_v_bat_power->IsStale() && StandardMetrics.ms_v_bat_power->AsFloat() != 0) {
        ESP_LOGD(TAG, "Power is stale. Manually setting to zero");
        SetBatteryCurrent(0);
        SetBatteryPower(0);
    }
}

// Data calculation functions
float OvmsVehicleToyotaETNGA::CalculateAmbientTemperature(const std::string& data)
{
    return static_cast<float>(GetRxBInt16(data, 0)) / 100.0f;
}

float OvmsVehicleToyotaETNGA::CalculateAmbientTemperatureEV(const std::string& data)
{
    return static_cast<float>(GetRxBInt8(data, 0)) - 40.0f;
}

float OvmsVehicleToyotaETNGA::CalculateBatteryChargingPower(const std::string& data)
{
    return static_cast<float>(GetRxBUint16(data, 0)- 32768) / 100.0f ;
}

float OvmsVehicleToyotaETNGA::CalculateBatteryCurrent(const std::string& data)
{
    return static_cast<float>(GetRxBInt16(data, 4)) / 10.0f;
}

float OvmsVehicleToyotaETNGA::CalculateBatteryPower(float voltage, float current)
{
    return voltage * current / 1000.0f;
}

float OvmsVehicleToyotaETNGA::CalculateBatterySOC(const std::string& data)
{
    return static_cast<float>(GetRxBByte(data, 0));
}

float OvmsVehicleToyotaETNGA::CalculateBatterySOCBMS(const std::string& data)
{
    return GetRxBByte(data, 0) * 100.0f / 255.0f;
}

std::vector<float> OvmsVehicleToyotaETNGA::CalculateBatteryCellVoltages(const std::string& data)
{
    // 0x182E payload: 96 cells × uint16 BE; each LSB = 5/65535 V (~76 µV).
    std::vector<float> voltages;
    voltages.reserve(96);

    for (size_t i = 0; i < 192; i += 2) {
        uint16_t raw = GetRxBUint16(data, i);
        voltages.push_back(static_cast<float>(raw) * 5.0f / 65535.0f);
    }

    return voltages;
}

std::vector<float> OvmsVehicleToyotaETNGA::CalculateBatteryTemperatures(const std::string& data)
{
    std::vector<float> temperatures;

    for (size_t i = 0; i < 48; i += 2) {
        int16_t temperatureRaw = GetRxBInt16(data, i);
        float temperature = static_cast<float>(temperatureRaw) / 256.0f - 50.0f;
        temperatures.push_back(temperature);
    }

    return temperatures;
}

float OvmsVehicleToyotaETNGA::CalculateBatteryVoltage(const std::string& data)
{
    return static_cast<float>(GetRxBUint16(data, 2)) / 64.0f;
}

float OvmsVehicleToyotaETNGA::CalculateCabinTemperature(const std::string& data)
{
    return static_cast<float>(GetRxBInt16(data, 0)) / 100.0f;
}

float OvmsVehicleToyotaETNGA::CalculateChargerInputPower(const std::string& data)
{
    return static_cast<float>(GetRxBUint16(data, 0)) * 5.0f / 1000.0f ;
}

bool OvmsVehicleToyotaETNGA::CalculateChargingDoorStatus(const std::string& data)
{
    return GetRxBBit(data, 1, 1);
}

int OvmsVehicleToyotaETNGA::CalculateChargeMode(const std::string& data)
{
    return GetRxBInt8(data, 0);
}

int OvmsVehicleToyotaETNGA::CalculateAcOpStatus(const std::string& data) { return GetRxBInt8(data, 0); }
void OvmsVehicleToyotaETNGA::SetAcOpStatus(int v) { m_v_charge_ac_op->SetValue(v); }
int OvmsVehicleToyotaETNGA::CalculateHlcState(const std::string& data) { return GetRxBByte(data, 0); }
void OvmsVehicleToyotaETNGA::SetHlcState(int v) { m_v_charge_hlc->SetValue(v); }
int OvmsVehicleToyotaETNGA::CalculatePISWRaw(const std::string& data) { return GetRxBInt8(data, 0); }
void OvmsVehicleToyotaETNGA::SetPISWRaw(int v) { m_v_charge_pisw_raw->SetValue(v); }

float OvmsVehicleToyotaETNGA::CalculatePermissionPower(const std::string& data)
{
    // 0x16A1: two's-complement s16, x0.01 kW. Sentinel 0x8000 handled by caller (skip).
    return static_cast<float>(GetRxBInt16(data, 0)) / 100.0f;
}
void OvmsVehicleToyotaETNGA::SetPermissionPower(float kw) { m_v_charge_perm->SetValue(kw); }

float OvmsVehicleToyotaETNGA::CalculateTargetCurrent(const std::string& data)
{
    return static_cast<float>(GetRxBUint16(data, 0));  // x1 A
}
void OvmsVehicleToyotaETNGA::SetTargetCurrent(float amps) { m_v_charge_tgti->SetValue(amps); }

int OvmsVehicleToyotaETNGA::CalculateControlMode(const std::string& data)
{
    // 0x00 = None
    // 0x01 = Driving
    // 0x03 = Charging / Discharging Mode
    return GetRxBInt8(data, 0);
}

int OvmsVehicleToyotaETNGA::CalculateChargeType(const std::string& data)
{
    return GetRxBInt8(data, 0);
}

float OvmsVehicleToyotaETNGA::CalculateHVACSetpoint(const std::string& data)
{
    int rawValue = GetRxBInt8(data, 0);
    
    // Calculate the HVAC setpoint based on the piecewise function
    if (rawValue == 0) {
        return 0.0f;
    } else if (rawValue < 28) {
        return (rawValue / 2.0f) + 15.5f;
    } else if (rawValue < 55) {
        return (rawValue - 1) * 5.0f / 9.0f;
    } else if (rawValue == 55) {
        return 100.0f;
    } else if (rawValue < 100) {
        return (rawValue / 2.0f) - 34.0f;
    } else {
        return (rawValue - 74) * 5.0f / 9.0f;
    }
}

float OvmsVehicleToyotaETNGA::CalculateOdometer(const std::string& data)
{
    return static_cast<float>(GetRxBUint32(data, 0)) / 10.0f;
}

bool OvmsVehicleToyotaETNGA::CalculatePISWStatus(const std::string& data)
{
    // 0x00 = None
    // 0x03 = Charging connector connected
    return GetRxBInt8(data, 0);
}

bool OvmsVehicleToyotaETNGA::CalculateReadyStatus(const std::string& data)
{
    return GetRxBBit(data, 1, 0);
}

int OvmsVehicleToyotaETNGA::CalculateShiftPosition(const std::string& data)
{
    return static_cast<int>(GetRxBInt8(data, 0));
}

float OvmsVehicleToyotaETNGA::CalculateVehicleSpeed(const std::string& data)
{
    return static_cast<float>(GetRxBInt8(data, 0));
}

// Metric setter functions
void OvmsVehicleToyotaETNGA::SetAmbientTemperature(float temperature)
{
    if (temperature == -40.0f)
    {
        // Ignore -40 temperature
        ESP_LOGI(TAG, "Ignoring invalid temperature value: %f", temperature);
    }
    else
    {
        LogMetricChange(StandardMetrics.ms_v_env_temp, temperature, "Ambient Temperature", "°C");
        StandardMetrics.ms_v_env_temp->SetValue(temperature);
    }
}

void OvmsVehicleToyotaETNGA::SetAwake(bool awake)
{
    StandardMetrics.ms_v_env_awake->SetValue(awake);
}

void OvmsVehicleToyotaETNGA::SetBatteryChargingPower(float power)
{
    ESP_LOGD(TAG, "Battery Charging Power: %f", power);

    float hoursSinceLastUpdate = 1.0f / 60.0f / 60.0f; // Default value of 1 second

    if (lastChargerEnergyLogTime != 0)
    {
        hoursSinceLastUpdate = static_cast<float>((esp_log_timestamp() - lastChargerEnergyLogTime) / (1000.0f * 60.0f * 60.0f));
    }

    const float energy = power * hoursSinceLastUpdate;

    ESP_LOGD(TAG, "Time update: %f hours", hoursSinceLastUpdate);
    ESP_LOGD(TAG, "Battery Charging Energy: %f kWh", energy);

    // Add the energy to the integrater
    StandardMetrics.ms_v_charge_kwh->SetValue(StandardMetrics.ms_v_charge_kwh->AsFloat() + energy);

    // Update the update time for the next energy period
    lastChargerEnergyLogTime = esp_log_timestamp();

}

void OvmsVehicleToyotaETNGA::SetBatteryCurrent(float current)
{
    LogMetricChange(StandardMetrics.ms_v_bat_current, current, "Battery Current", "A");
    StandardMetrics.ms_v_bat_current->SetValue(current);
}

void OvmsVehicleToyotaETNGA::SetBatteryPower(float power)
{
    LogMetricChange(StandardMetrics.ms_v_bat_power, power, "Battery Power", "kW");
    StandardMetrics.ms_v_bat_power->SetValue(power);

    float hoursSinceLastUpdate = 1.0f / 60.0f / 60.0f; // Default value of 1 second
    int now = esp_log_timestamp();

    if (lastBatteryEnergyLogTime != 0)
    {
        hoursSinceLastUpdate = static_cast<float>((now - lastBatteryEnergyLogTime) / (1000.0f * 60.0f * 60.0f));
    }

    const float energy = power * hoursSinceLastUpdate;

    ESP_LOGD(TAG, "Battery Energy: %f kWh", energy);

    // Only log energy use/recovery while driving
    if (static_cast<PollState>(m_poll_state) == PollState::DRIVING)
    {
        if (power > 0)
        {
            StandardMetrics.ms_v_bat_energy_used->SetValue(
                StandardMetrics.ms_v_bat_energy_used->AsFloat() + energy);
        }
        else if (power < 0)
        {
            StandardMetrics.ms_v_bat_energy_recd->SetValue(
                StandardMetrics.ms_v_bat_energy_recd->AsFloat() - energy);
        }
    }

    // Update the update time for the next energy period
    lastBatteryEnergyLogTime = now;
}

void OvmsVehicleToyotaETNGA::SetBatterySOC(float soc)
{
    if (soc == 0.0 && StandardMetrics.ms_v_bat_soc->AsFloat() > 1.0)
    {
        // Ignore zero SOC if previousSOC was greater than 1
        ESP_LOGI(TAG, "Ignoring invalid SOC value: %f", soc);
    }
    else
    {
        LogMetricChange(StandardMetrics.ms_v_bat_soc, soc, "SOC", "%");
        StandardMetrics.ms_v_bat_soc->SetValue(soc);
    }
}

void OvmsVehicleToyotaETNGA::SetBatterySOCBMS(float soc)
{
    if (soc == 0.0 && m_v_bat_soc_bms->AsFloat() > 1.0)
    {
        // Ignore zero SOC if previousSOC was greater than 1
        ESP_LOGI(TAG, "Ignoring invalid SOC value: %f", soc);
    }
    else
    {
        LogMetricChange(m_v_bat_soc_bms, soc, "BMS SOC", "%");
        m_v_bat_soc_bms->SetValue(soc);
    }
}

void OvmsVehicleToyotaETNGA::SetBatteryCellVoltages(const std::vector<float>& voltages)
{
    // If the subclass declared a BMS voltage arrangement, route per-cell through the
    // BMS API so it can maintain per-cell history, deviation flags, and pack stats.
    // Otherwise fall back to a direct vector set.
    if (BmsGetCellArangementVoltage() > 0) {
        BmsRestartCellVoltages();
        for (size_t i = 0; i < voltages.size(); ++i) {
            BmsSetCellVoltage(static_cast<int>(i), voltages[i]);
        }
    } else {
        StandardMetrics.ms_v_bat_cell_voltage->SetValue(voltages);
    }
}

void OvmsVehicleToyotaETNGA::SetBatteryCellVoltageStatistics(const std::vector<float>& voltages)
{
    if (BmsGetCellArangementVoltage() > 0) {
        // BmsSetCellVoltage already populated pack vmin/vmax/vavg/vstddev.
        return;
    }

    // No BMS arrangement: compute and publish pack stats manually.
    float minVoltage = *std::min_element(voltages.begin(), voltages.end());
    float maxVoltage = *std::max_element(voltages.begin(), voltages.end());
    float sum = std::accumulate(voltages.begin(), voltages.end(), 0.0f);
    float averageVoltage = sum / voltages.size();
    float variance = 0.0f;
    for (float v : voltages) {
        variance += pow(v - averageVoltage, 2);
    }
    float standardDeviation = sqrt(variance / voltages.size());

    StandardMetrics.ms_v_bat_pack_vmin->SetValue(minVoltage);
    StandardMetrics.ms_v_bat_pack_vmax->SetValue(maxVoltage);
    StandardMetrics.ms_v_bat_pack_vavg->SetValue(averageVoltage);
    StandardMetrics.ms_v_bat_pack_vstddev->SetValue(standardDeviation);
}

void OvmsVehicleToyotaETNGA::SetBatteryTemperatures(const std::vector<float>& temperatures)
{
    // If the subclass declared a BMS temperature arrangement, route per-sensor through
    // the BMS API so it can maintain per-cell history, deviation flags, and pack stats.
    // Otherwise fall back to a direct vector set (legacy / un-arranged subclasses).
    if (BmsGetCellArangementTemperature() > 0) {
        BmsRestartCellTemperatures();
        for (size_t i = 0; i < temperatures.size(); ++i) {
            BmsSetCellTemperature(static_cast<int>(i), temperatures[i]);
        }
    } else {
        StandardMetrics.ms_v_bat_cell_temp->SetValue(temperatures);
    }
}

void OvmsVehicleToyotaETNGA::SetBatteryTemperatureStatistics(const std::vector<float>& temperatures)
{
    // Average is needed for ms_v_bat_temp regardless of code path.
    float sum = std::accumulate(temperatures.begin(), temperatures.end(), 0.0f);
    float averageTemperature = sum / temperatures.size();

    if (BmsGetCellArangementTemperature() == 0) {
        // No BMS arrangement: compute pack stats manually and publish them.
        float minTemperature = *std::min_element(temperatures.begin(), temperatures.end());
        float maxTemperature = *std::max_element(temperatures.begin(), temperatures.end());
        float variance = 0.0f;
        for (float temperature : temperatures) {
            variance += pow(temperature - averageTemperature, 2);
        }
        float standardDeviation = sqrt(variance / temperatures.size());

        StandardMetrics.ms_v_bat_pack_tmin->SetValue(minTemperature);
        StandardMetrics.ms_v_bat_pack_tmax->SetValue(maxTemperature);
        StandardMetrics.ms_v_bat_pack_tavg->SetValue(averageTemperature);
        StandardMetrics.ms_v_bat_pack_tstddev->SetValue(standardDeviation);
    }
    // When arrangement IS set, BmsSetCellTemperature already populated tmin/tmax/tavg/tstddev.

    // ms_v_bat_temp (the single representative pack temperature) isn't auto-set by either
    // path, so publish the average here.
    StandardMetrics.ms_v_bat_temp->SetValue(averageTemperature);
}

void OvmsVehicleToyotaETNGA::SetBatteryVoltage(float voltage)
{
    LogMetricChange(StandardMetrics.ms_v_bat_voltage, voltage, "Voltage", "V");
    StandardMetrics.ms_v_bat_voltage->SetValue(voltage);
}

void OvmsVehicleToyotaETNGA::SetChargeMode(int chargeMode)
{
    const std::string chargeModeValue = (chargeMode == 0x00) ? "Standard" : "Performance";
    LogMetricChange(StandardMetrics.ms_v_charge_mode, chargeModeValue, "Charge Mode");
    StandardMetrics.ms_v_charge_mode->SetValue(chargeModeValue);
}

void OvmsVehicleToyotaETNGA::SetChargingStatus(bool status)
{
    const std::string valueLabel = status ? "Charging" : "Not Charging";
    LogMetricChange(StandardMetrics.ms_v_charge_inprogress, status, "Charging Status", valueLabel);
    StandardMetrics.ms_v_charge_inprogress->SetValue(status);
}

void OvmsVehicleToyotaETNGA::SetChargeState(PollState state)
{
    std::string s;
    switch (state) {
        case PollState::CHARGE_HANDSHAKE: s = "prepared"; break;
        case PollState::CHARGE_WAIT:      s = "stopped";  break;
        case PollState::CHARGE_AC:
        case PollState::CHARGE_DC:        s = "charging"; break;
        default:                          s = ""; break;
    }
    LogMetricChange(StandardMetrics.ms_v_charge_state, s, "Charge State");
    StandardMetrics.ms_v_charge_state->SetValue(s);
}

void OvmsVehicleToyotaETNGA::SetCabinTemperature(float temperature)
{
    if (temperature == -40.0f)
    {
        // Ignore -40 temperature
        ESP_LOGI(TAG, "Ignoring invalid temperature value: %f", temperature);
    }
    else
    {
        LogMetricChange(StandardMetrics.ms_v_env_cabintemp, temperature, "Cabin Temperature", "°C");
        StandardMetrics.ms_v_env_cabintemp->SetValue(temperature);
    }
}

void OvmsVehicleToyotaETNGA::SetChargeType(int chargeType)
{
    std::string chargeTypeValue;

    // 0x161C "Charger Power Supply Voltage Type" enum (confirmed 2026-05-10):
    // 0 = None/unconnected, 1 = 100V Series, 2 = 200V Series. raw 0 must clear the
    // metric (no charge connected), NOT report "ccs" — that was the prior bug.
    // DCFC value is still TBD pending a real DC charge.
    switch (chargeType)
    {
        case 0x00:
            chargeTypeValue = "";  // None / unconnected
            break;
        case 0x01:
            chargeTypeValue = "type1";  // 100V Series
            break;
        case 0x02:
            chargeTypeValue = "type2";  // 200V Series
            break;
        default:
            chargeTypeValue = "unknown";
            break;
    }
    
    LogMetricChange(StandardMetrics.ms_v_charge_type, chargeTypeValue, "Charge Type");
    StandardMetrics.ms_v_charge_type->SetValue(chargeTypeValue);
}

void OvmsVehicleToyotaETNGA::SetChargerInputPower(float power)
{
    ESP_LOGD(TAG, "Charger Input Power: %f", power);
    StandardMetrics.ms_v_charge_power->SetValue(power);

    float hoursSinceLastUpdate = 1.0f / 60.0f / 60.0f; // Default value of 1 second
    int now = esp_log_timestamp();

    if (lastGridEnergyLogTime != 0)
        {
        hoursSinceLastUpdate = static_cast<float>((now - lastGridEnergyLogTime) / (1000.0f * 60.0f * 60.0f));
        }

    const float energy = power * hoursSinceLastUpdate;

    ESP_LOGD(TAG, "Grid Energy: %f kWh", energy);

    StandardMetrics.ms_v_charge_kwh_grid->SetValue(StandardMetrics.ms_v_charge_kwh_grid->AsFloat() + energy);

    // Update the update time for the next energy period
    lastGridEnergyLogTime = now;
}

void OvmsVehicleToyotaETNGA::SetChargingDoorStatus(bool status)
{
    int newValue = status ? 1 : 0;
    const std::string valueLabel = status ? "Open" : "Closed";    
    
    LogMetricChange(StandardMetrics.ms_v_door_chargeport, newValue, "Charging Door Status", valueLabel);

    StandardMetrics.ms_v_door_chargeport->SetValue(status);
}

void OvmsVehicleToyotaETNGA::SetControlMode(int controlMode)
{
    std::string controlModeLabel;

    switch (controlMode)
    {
        case 0x00:
            controlModeLabel = "None";
            break;
        case 0x01:
            controlModeLabel = "Driving";
            break;
        case 0x03:
            controlModeLabel = "Charging";
            break;
        default:
            controlModeLabel = "unknown";
            break;
    }

    LogMetricChange(m_s_controlstate, controlMode, "Control Mode", controlModeLabel);
    m_s_controlstate->SetValue(controlMode);
}

void OvmsVehicleToyotaETNGA::SetHVACSetpoint(float temperature)
{
    LogMetricChange(StandardMetrics.ms_v_env_cabinsetpoint, temperature, "HVAC Setpoint", "°C");
    StandardMetrics.ms_v_env_cabinsetpoint->SetValue(temperature);
}

void OvmsVehicleToyotaETNGA::SetOdometer(float odometer)
{
    LogMetricChange(StandardMetrics.ms_v_pos_odometer, odometer, "Odometer", "km");
    StandardMetrics.ms_v_pos_odometer->SetValue(odometer);  // Set the odometer metric

    if (m_v_pos_trip_start->IsStale())
    {
        // Update the trip start metric if it is stale
        // It becomes stale when first transitioning to the READY state
        m_v_pos_trip_start->SetValue(odometer);
    }

    // Update the trip odometer
    float tripOdometer = odometer - m_v_pos_trip_start->AsFloat();
    StandardMetrics.ms_v_pos_trip->SetValue(tripOdometer);
}

void OvmsVehicleToyotaETNGA::SetPISWStatus(bool status)
{
    const std::string valueLabel = status ? "Connected" : "Not Connected";
    LogMetricChange(StandardMetrics.ms_v_charge_pilot, status, "Pilot Status", valueLabel);
    StandardMetrics.ms_v_charge_pilot->SetValue(status);
}

void OvmsVehicleToyotaETNGA::SetPollState(int state)
{
    const char* CurrentPollStateText = ConvertPollStateToString(m_poll_state);
    const char* NextPollStateText = ConvertPollStateToString(state);

    ESP_LOGI(TAG, "Transitioning from the %s to the %s state", CurrentPollStateText, NextPollStateText);

    PollSetState(state);
}

void OvmsVehicleToyotaETNGA::SetReadyStatus(bool status)
{
    const std::string valueLabel = status ? "Ready" : "Not Ready";
    LogMetricChange(StandardMetrics.ms_v_env_on, status, "Ready Status", valueLabel);
    StandardMetrics.ms_v_env_on->SetValue(status);
}

void OvmsVehicleToyotaETNGA::SetShiftPosition(int position)
{
    const char* shiftPositionText;
    int gear;

    switch (position) {
        case 2:
            shiftPositionText = "Reverse";
            gear = -1;
            break;
        case 4:
            shiftPositionText = "Neutral";
            gear = 0;
            break;
        case 0:
            shiftPositionText = "Park";
            gear = 0;
            break;
        case 6:
            shiftPositionText = "Drive";
            gear = 1;
            break;
        // No case 8 ("B"): the Solterra/bZ4X has no B range — Toyota's RM Electronic
        // Shift Lever data list states "This vehicle does not have the B (S) range",
        // and gear-byte0 has only ever been 0/2/4/6 across all captures. Regen is via
        // paddles (Solterra) / boost button (bZ4X), not a selector position. Any
        // unexpected value correctly falls through to Unknown below.
        default:
            shiftPositionText = "Unknown";
            gear = -999;
            break;
    }

    LogMetricChange(StandardMetrics.ms_v_env_gear, gear, "Gear", shiftPositionText);
    StandardMetrics.ms_v_env_gear->SetValue(gear);
}

void OvmsVehicleToyotaETNGA::SetVehicleSpeed(float speed)
{
    LogMetricChange(StandardMetrics.ms_v_pos_speed, speed, "Speed", "kph");
    StandardMetrics.ms_v_pos_speed->SetValue(speed);
}

void OvmsVehicleToyotaETNGA::SetVehicleVIN(std::string vin)
{
    ESP_LOGD(TAG, "VIN: %s", vin.c_str());
    StandardMetrics.ms_v_vin->SetValue(std::move(vin));
}

void OvmsVehicleToyotaETNGA::LogMetricChange(OvmsMetricBool* metric, bool newValue, const std::string& label, const std::string& valueLabel)
{
    if (metric->AsBool() != newValue)
    {
        ESP_LOGD(TAG, "%s changed: %s (%d)", label.c_str(), valueLabel.c_str(), newValue ? 1 : 0);
    }
    else
    {
        ESP_LOGV(TAG, "%s unchanged: %s (%d)", label.c_str(), valueLabel.c_str(), newValue ? 1 : 0);
    }
}

void OvmsVehicleToyotaETNGA::LogMetricChange(OvmsMetricFloat* metric, float newValue, const std::string& label, const std::string& units = "")
{
    if (metric->AsFloat() != newValue)
    {
        ESP_LOGD(TAG, "%s changed: %.2f%s", label.c_str(), newValue, units.empty() ? "" : (" " + units).c_str());
    }
    else
    {
        ESP_LOGV(TAG, "%s unchanged: %.2f%s", label.c_str(), newValue, units.empty() ? "" : (" " + units).c_str());
    }
}

void OvmsVehicleToyotaETNGA::LogMetricChange(OvmsMetricInt* metric, int newValue, const std::string& label, const std::string& valueLabel)
{
    if (metric->AsInt() != newValue)
    {
        ESP_LOGD(TAG, "%s changed: %s (%d)", label.c_str(), valueLabel.c_str(), newValue);
    }
    else
    {
        ESP_LOGV(TAG, "%s unchanged: %s (%d)", label.c_str(), valueLabel.c_str(), newValue);
    }
}

void OvmsVehicleToyotaETNGA::LogMetricChange(OvmsMetricString* metric, const std::string& newValue, const std::string& label)
{
    if (metric->AsString() != newValue)
    {
        ESP_LOGD(TAG, "%s changed: %s", label.c_str(), newValue.c_str());
    }
    else
    {
        ESP_LOGV(TAG, "%s unchanged: %s", label.c_str(), newValue.c_str());
    }
}