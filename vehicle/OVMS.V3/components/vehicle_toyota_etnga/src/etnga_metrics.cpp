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
    m_s_pollstate = MyMetrics.InitInt("xte.s.pollstate", SM_STALE_NONE);  // This variable stores the pollstate
    m_v_bat_heater_status = MyMetrics.InitBool("xte.v.b.heater", SM_STALE_MID);  // This variable stores the status of the battery coolant heater relay
    m_v_bat_soc_bms = MyMetrics.InitFloat("xte.v.b.soc.bms", SM_STALE_MID, 0.0f, Percentage, true);  // This variable stores the SOC as reported by the BMS
    m_v_bat_speed_water_pump = MyMetrics.InitFloat("xte.v.b.speed.waterpump", SM_STALE_MID, 0.0f, Other);  // This variable stores the RPM of the battery water pump
    m_v_bat_temp_coolant = MyMetrics.InitFloat("xte.v.b.temp.coolant", SM_STALE_MID, 0.0f, Celcius);  // This variable stores the temperature of the battery coolant
    m_v_bat_temp_heater = MyMetrics.InitFloat("xte.v.b.temp.heater", SM_STALE_MID, 0.0f, Celcius);  // This variable stores the temperature of the battery coolant
    m_v_pos_trip_start = MyMetrics.InitFloat("xte.v.p.trip.start", SM_STALE_NONE, 0.0f, Kilometers, false);  // This variable stores the odometer reading at the beginning of a trip

    // Set initial values for metrics
    SetAwake(false);
    SetReadyStatus(false);
    SetChargingDoorStatus(false);

    // Set poll state transition variables to shorter autostale than default
    // in case their ECUs go to sleep before the 'false' poll
    StandardMetrics.ms_v_door_chargeport->SetAutoStale(10);
}

void OvmsVehicleToyotaETNGA::ResetStaleMetrics() // Reset stale state transition variables
{
    // Check to make sure the 'ready' signal has been updated recently
    if (StandardMetrics.ms_v_env_awake->IsStale() && StandardMetrics.ms_v_env_awake->AsBool()) {
        ESP_LOGD(TAG, "Awake is stale. Manually setting to off");
        SetAwake(false);
    }

    // Check to make sure the 'ready' signal has been updated recently
    if (StandardMetrics.ms_v_env_on->IsStale() && StandardMetrics.ms_v_env_on->AsBool()) {
        ESP_LOGD(TAG, "Ready is stale. Manually setting to off");
        SetReadyStatus(false);
    }

    // Check to make sure the 'charging door' signal has been updated recently
    if (StandardMetrics.ms_v_door_chargeport->IsStale() && StandardMetrics.ms_v_door_chargeport->AsBool()) {
        ESP_LOGD(TAG, "Charging Door is stale. Manually setting to off");
        SetChargingDoorStatus(false);
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
    return static_cast<float>(GetRxBInt8(data, 0)) - 40.0f;
}

float OvmsVehicleToyotaETNGA::CalculateBatteryChargingPower(const std::string& data)
{
    return static_cast<float>(GetRxBUint16(data,0)- 32768) / 100.0f ;
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
    return static_cast<float>(GetRxBInt8(data, 0));
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

float OvmsVehicleToyotaETNGA::CalculateChargerInputPower(const std::string& data)
{
    return static_cast<float>(GetRxBUint16(data,0)) * 5.0f / 1000.0f ;
}

bool OvmsVehicleToyotaETNGA::CalculateChargingDoorStatus(const std::string& data)
{
    return GetRxBBit(data, 1, 1);
}

bool OvmsVehicleToyotaETNGA::CalculateChargingStatus(const std::string& data)
{
    // 0x00 = None
    // 0x03 = Charging / Discharging Mode
    return GetRxBInt8(data, 0);
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
        ESP_LOGD(TAG, "Ignoring invalid temperature value: %f", temperature);
    }
    else
    {
        ESP_LOGD(TAG, "Ambient Temperature: %f", temperature);
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

    if (!lastChargerEnergyLogTime == 0)
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
    ESP_LOGD(TAG, "Current: %f", current);
    StandardMetrics.ms_v_bat_current->SetValue(current);
}

void OvmsVehicleToyotaETNGA::SetBatteryPower(float power)
{
    ESP_LOGD(TAG, "Battery Power: %f", power);
    StandardMetrics.ms_v_bat_power->SetValue(power);

    float hoursSinceLastUpdate = 1.0f / 60.0f / 60.0f; // Default value of 1 second

    if (!lastBatteryEnergyLogTime == 0)
    {
        hoursSinceLastUpdate = static_cast<float>((esp_log_timestamp() - lastBatteryEnergyLogTime) / (1000.0f * 60.0f * 60.0f));
    }

    const float energy = power * hoursSinceLastUpdate;

    ESP_LOGD(TAG, "Battery Energy: %f kWh", energy);

    if (power > 0)
    {
        StandardMetrics.ms_v_bat_energy_used->SetValue(StandardMetrics.ms_v_bat_energy_used->AsFloat() + energy);
    }
    else if (power < 0)
    {
        StandardMetrics.ms_v_bat_energy_recd->SetValue(StandardMetrics.ms_v_bat_energy_recd->AsFloat() - energy);
    }

    // Update the update time for the next energy period
    lastBatteryEnergyLogTime = esp_log_timestamp();
}

void OvmsVehicleToyotaETNGA::SetBatterySOC(float soc)
{
    if (soc == 0.0 && StandardMetrics.ms_v_bat_soc->AsFloat() > 1.0)
    {
        // Ignore zero SOC if previousSOC was greater than 1
        ESP_LOGD(TAG, "Ignoring invalid SOC value: %f", soc);
    }
    else
    {
        ESP_LOGD(TAG, "SOC: %f", soc);
        StandardMetrics.ms_v_bat_soc->SetValue(soc);
    }
}

void OvmsVehicleToyotaETNGA::SetBatteryTemperatures(const std::vector<float>& temperatures)
{
    StandardMetrics.ms_v_bat_cell_temp->SetValue(temperatures);
}

void OvmsVehicleToyotaETNGA::SetBatteryTemperatureStatistics(const std::vector<float>& temperatures)
{
    // Calculate the minimum temperature
    float minTemperature = *std::min_element(temperatures.begin(), temperatures.end());

    // Calculate the maximum temperature
    float maxTemperature = *std::max_element(temperatures.begin(), temperatures.end());

    // Calculate the average temperature
    float sum = std::accumulate(temperatures.begin(), temperatures.end(), 0.0f);
    float averageTemperature = sum / temperatures.size();

    // Calculate the standard deviation
    float variance = 0.0f;
    for (float temperature : temperatures) {
        variance += pow(temperature - averageTemperature, 2);
    }
    float standardDeviation = sqrt(variance / temperatures.size());

    // Store the results
    StandardMetrics.ms_v_bat_pack_tmin->SetValue(minTemperature);
    StandardMetrics.ms_v_bat_pack_tmax->SetValue(maxTemperature);
    StandardMetrics.ms_v_bat_pack_tavg->SetValue(averageTemperature);
    StandardMetrics.ms_v_bat_pack_tstddev->SetValue(standardDeviation);

    // There is no single battery temperature value, so I'm using the min Temperature
    StandardMetrics.ms_v_bat_temp->SetValue(minTemperature);
}

void OvmsVehicleToyotaETNGA::SetBatteryVoltage(float voltage)
{
    ESP_LOGD(TAG, "Voltage: %f", voltage);
    StandardMetrics.ms_v_bat_voltage->SetValue(voltage);
}

void OvmsVehicleToyotaETNGA::SetChargeMode(int chargeMode)
{

    std::string chargeModeValue = (chargeMode == 0x00) ? "Standard" : "Performance";

    ESP_LOGD(TAG, "Charge Mode: %s", chargeModeValue.c_str());
    StandardMetrics.ms_v_charge_mode->SetValue(chargeModeValue);
}

void OvmsVehicleToyotaETNGA::SetChargeType(int chargeType)
{
    std::string chargeTypeValue;

    switch (chargeType)
    {
        case 0x00:
            chargeTypeValue = "ccs";
            break;
        case 0x01:
            chargeTypeValue = "type1";
            break;
        case 0x02:
            chargeTypeValue = "type2";
            break;
        default:
            chargeTypeValue = "unknown";
            break;
    }
    
    ESP_LOGD(TAG, "Charge Type: %s", chargeTypeValue.c_str());
    StandardMetrics.ms_v_charge_type->SetValue(chargeTypeValue);
}

void OvmsVehicleToyotaETNGA::SetChargeState(std::string chargeState)
{
    ESP_LOGI(TAG, "Charge State: %s", chargeState.c_str());
    StandardMetrics.ms_v_charge_state->SetValue(chargeState.c_str());
}

void OvmsVehicleToyotaETNGA::SetChargerInputPower(float power)
{
    ESP_LOGD(TAG, "Changer Input Power: %f", power);
    StandardMetrics.ms_v_charge_power->SetValue(power);

    float hoursSinceLastUpdate = 1.0f / 60.0f / 60.0f; // Default value of 1 second

    if (!lastGridEnergyLogTime == 0)
        {
        hoursSinceLastUpdate = static_cast<float>((esp_log_timestamp() - lastGridEnergyLogTime) / (1000.0f * 60.0f * 60.0f));
        }

    const float energy = power * hoursSinceLastUpdate;

    ESP_LOGD(TAG, "Grid Energy: %f kWh", energy);

    StandardMetrics.ms_v_charge_kwh_grid->SetValue(StandardMetrics.ms_v_charge_kwh_grid->AsFloat() + energy);

    // Update the update time for the next energy period
    lastGridEnergyLogTime = esp_log_timestamp();
}

void OvmsVehicleToyotaETNGA::SetChargingDoorStatus(bool status)
{
    ESP_LOGD(TAG, "Charging Door Status: %s", status ? "Open" : "Closed");
    StandardMetrics.ms_v_door_chargeport->SetValue(status);
}

void OvmsVehicleToyotaETNGA::SetChargingStatus(bool status)
{
    ESP_LOGD(TAG, "Charging Status: %s", status ? "Charging" : "Not Charging");
    StandardMetrics.ms_v_charge_inprogress->SetValue(status);
}

void OvmsVehicleToyotaETNGA::SetOdometer(float odometer)
{
    ESP_LOGD(TAG, "Odometer: %f", odometer);
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
    ESP_LOGD(TAG, "Pilot Status: %s", status ? "Connected" : "Not Connected");
    StandardMetrics.ms_v_charge_pilot->SetValue(status);
}

void OvmsVehicleToyotaETNGA::SetPollState(int state)
{
    const char* CurrentPollStateText = ConvertPollStateToString(m_s_pollstate->AsInt());
    const char* NextPollStateText = ConvertPollStateToString(state);
    
    ESP_LOGI(TAG, "Transitioning from the %s to the %s state", CurrentPollStateText, NextPollStateText);

    PollSetState(state);
    m_s_pollstate->SetValue(state);
}

void OvmsVehicleToyotaETNGA::SetReadyStatus(bool status)
{
    ESP_LOGD(TAG, "Ready Status: %s", status ? "Ready" : "Not Ready");
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
        default:
            shiftPositionText = "Unknown";
            gear = -999;
            break;
    }

    ESP_LOGD(TAG, "Gear: %s", shiftPositionText);
    StandardMetrics.ms_v_env_gear->SetValue(gear);
}

void OvmsVehicleToyotaETNGA::SetVehicleSpeed(float speed)
{
    ESP_LOGD(TAG, "Speed: %f", speed);
    StandardMetrics.ms_v_pos_speed->SetValue(speed);
}

void OvmsVehicleToyotaETNGA::SetVehicleVIN(std::string vin)
{
    ESP_LOGD(TAG, "VIN: %s", vin.c_str());
    StandardMetrics.ms_v_vin->SetValue(std::move(vin));
}
