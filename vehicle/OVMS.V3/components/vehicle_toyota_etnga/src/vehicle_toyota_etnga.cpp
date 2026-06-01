/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Toyota e-TNGA platform
   Date:          4th June 2023

   (C) 2023       Jerry Kezar <solterra@kezarnet.com>

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include "ovms_log.h"
#include "vehicle_toyota_etnga.h"

// Poll state descriptions:
//    SLEEP (0)             : Vehicle is sleeping; no activity on the CAN bus. We are listening only.
//    AWAKE (1)             : Vehicle is alive; vehicle has been switched on by driver
//    DRIVING (2)           : Vehicle is "Ready" to drive or being driven
//    CHARGING (3)          : Vehicle is charging

static const OvmsPoller::poll_pid_t obdii_polls[] = {
    // State variables polls
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CONTROL_SYSTEM_MODE, { 0, 1, 1, 1}, 0, ISOTP_STD }, // { 0, 1, 1, 1} (Checked)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGING_LID, { 0, 10, 10, 0}, 0, ISOTP_STD }, // { 0, 10, 10, 0} (Checked)

    // Combined polls
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_SOC, { 0, 0, 1, 1}, 0, ISOTP_STD }, // { 0, 0, 1, 1} (Checked)
  { HYBRID_BATTERY_SYSTEM_TX, HYBRID_BATTERY_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_SOC_BMS, {0, 0, 1, 1}, 0, ISOTP_STD }, // {0, 0, 1, 1} (Checked)
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_READY_SIGNAL, { 0, 0, 1, 1}, 0, ISOTP_STD }, // { 0, 0, 1, 1} (Checked)
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_VOLTAGE_AND_CURRENT, { 0, 0, 1, 1}, 0, ISOTP_STD }, // { 0, 0, 1, 1} (Checked)
  { HYBRID_BATTERY_SYSTEM_TX, HYBRID_BATTERY_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_TEMPERATURES, {0, 0, 10, 10}, 0, ISOTP_STD }, // {0, 0, 10, 10} (Checked)
  { HYBRID_BATTERY_SYSTEM_TX, HYBRID_BATTERY_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_CELL_VOLTAGES, {0, 0, 5, 5}, 0, ISOTP_STD }, // 0x182E: 192-byte panel (96 × uint16 BE × 5/65535 V)

    // Driving polls
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_VEHICLE_SPEED, { 0, 0, 1, 0}, 0, ISOTP_STD }, // { 0, 0, 1, 0} (Checked)
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_SHIFT_POSITION, { 0, 0, 1, 0}, 0, ISOTP_STD }, // { 0, 0, 1, 0} (Checked)
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_ODOMETER, { 0, 0, 1, 0}, 0, ISOTP_STD }, // { 0, 0, 1, 0} (Checked)
  { AIR_CONDITIONER_TX, AIR_CONDITIONER_RX, VEHICLE_POLL_TYPE_READDATA, PID_AMBIENT_TEMPERATURE, { 0, 0, 10, 0}, 0, ISOTP_STD }, // {0, 0, 10, 0} (Checked)
  { AIR_CONDITIONER_TX, AIR_CONDITIONER_RX, VEHICLE_POLL_TYPE_READDATA, PID_CABIN_TEMPERATURE, { 0, 0, 10, 0}, 0, ISOTP_STD }, // {0, 0, 10, 0} (Checked)
  { AIR_CONDITIONER_TX, AIR_CONDITIONER_RX, VEHICLE_POLL_TYPE_READDATA, PID_HVAC_SETPOINT, {0, 0, 10, 0}, 0, ISOTP_STD }, // {0, 0, 10, 0} (Checked)

    // Charging polls
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AMBIENT_TEMPERATURE_EV, { 0, 0, 0, 10}, 0, ISOTP_STD }, // { 0, 0, 0, 10} (Checked)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGING_CONTROL_STATUS, { 0, 0, 0, 1}, 0, ISOTP_STD }, // { 0, 0, 0, 1} (Checked)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_PISW_STATUS, { 0, 0, 0, 10}, 0, ISOTP_STD }, // { 0, 0, 0, 10} (Checked)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGING_VOLTAGE_TYPE, { 0, 0, 0, 10}, 0, ISOTP_STD }, // { 0, 0, 0, 1} (Checking)

//  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGING_CONTROL_INFORMATION, { 0, 0, 0, 1}, 0, ISOTP_STD }, // { 0, 1, 1, 1}


  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_CHARGING_POWER, { 0, 0, 0, 1}, 0, ISOTP_STD }, // 0x10D4 battery-side charging power (1s in CHARGING); handler gated on charge-in-progress
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGER_INPUT_POWER, { 0, 0, 0, 1}, 0, ISOTP_STD }, // 0x161D grid-side input power (1s in CHARGING); handler gated on AC charge

//  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_DC_CHARGER_PRESENT_CURRENT, {0, 0, 0, 1}, 0, ISOTP_STD },
//  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_DC_CHARGER_PRESENT_VOLTAGE, {0, 0, 0, 1}, 0, ISOTP_STD },

    // Teter Present
  //{ PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_TESTERPRESENT, 0, { 0, 2, 2, 2}, 0, ISOTP_STD },
  //{ HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_ACTIVE_DIAGNOSTIC_SESSION, { 0, 2, 2, 2}, 0, ISOTP_STD },
  //{ HYBRID_BATTERY_SYSTEM_TX, HYBRID_BATTERY_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_ACTIVE_DIAGNOSTIC_SESSION, { 0, 2, 2, 2}, 0, ISOTP_STD },
  //{ PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_ACTIVE_DIAGNOSTIC_SESSION, { 0, 2, 2, 2}, 0, ISOTP_STD },
    POLL_LIST_END
};

OvmsVehicleToyotaETNGA::OvmsVehicleToyotaETNGA()
{
    ESP_LOGI(TAG, "Toyota eTNGA platform module");

    // Init metrics
    InitializeMetrics();

    // Init CAN
    RegisterCanBus(2, CAN_MODE_ACTIVE, CAN_SPEED_500KBPS);

    // Set polling state
    TransitionToSleepState();

    // Set polling PID list
    PollSetPidList(m_can2, obdii_polls);
    PollSetThrottling(0);
}

OvmsVehicleToyotaETNGA::~OvmsVehicleToyotaETNGA()
{
    ESP_LOGI(TAG, "Shutdown Toyota eTNGA platform module");
}

void OvmsVehicleToyotaETNGA::NotifyVehicleOn()
{
    ESP_LOGV(TAG, "Notification of vehicle on - Reset energy metrics for trip reporting");
    // Vehicle started. Reset the trip statistics
    StandardMetrics.ms_v_bat_energy_used->SetValue(0);
    StandardMetrics.ms_v_bat_energy_recd->SetValue(0);
    lastBatteryEnergyLogTime = 0;
}

void OvmsVehicleToyotaETNGA::NotifyChargeStart()
{
    ESP_LOGV(TAG, "Notification of charge start - Reset energy metrics for trip reporting");
    // Vehicle started. Reset the trip statistics
    StandardMetrics.ms_v_bat_energy_used->SetValue(0);
    StandardMetrics.ms_v_bat_energy_recd->SetValue(0);
    lastBatteryEnergyLogTime = 0;
    
    StandardMetrics.ms_v_charge_kwh->SetValue(0);
    lastChargerEnergyLogTime = 0;

    StandardMetrics.ms_v_charge_kwh_grid->SetValue(0);
    lastGridEnergyLogTime = 0;
    
}

void OvmsVehicleToyotaETNGA::Ticker1(uint32_t ticker)
{
  
    if (StandardMetrics.ms_v_charge_inprogress->AsBool()) {
        ESP_LOGV(CHARGING_TAG, "%.0f, %.2f, %.4f, %.0f, %.2f, %.2f, %.2f, %.2f, %.0f, %.2f, %.4f",
                StandardMetrics.ms_v_bat_voltage->AsFloat(),
                StandardMetrics.ms_v_bat_current->AsFloat(),
                StandardMetrics.ms_v_bat_power->AsFloat(),
                StandardMetrics.ms_v_env_temp->AsFloat(),
                StandardMetrics.ms_v_bat_pack_tavg->AsFloat(),
                StandardMetrics.ms_v_bat_pack_tmax->AsFloat(),
                StandardMetrics.ms_v_bat_pack_tmin->AsFloat(),
                StandardMetrics.ms_v_bat_pack_tstddev->AsFloat(),
                StandardMetrics.ms_v_bat_soc->AsFloat(),
                m_v_bat_soc_bms->AsFloat(),
                StandardMetrics.ms_v_charge_power->AsFloat());
    }

    //ESP_LOGI(TAG, "Entering Ticker1: %d", ticker);
    ResetStaleMetrics();

    switch (static_cast<PollState>(m_poll_state)) {
        case PollState::SLEEP:
            HandleSleepState();
            break;

        case PollState::AWAKE:
            HandleAwakeState();
            break;

        case PollState::DRIVING:
            HandleDrivingState();
            break;

        case PollState::CHARGING:
            HandleChargingState();
            break;

        default:
            ESP_LOGE(TAG, "Invalid poll state: %d", m_poll_state);
            break;
    }
}

