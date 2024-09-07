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
//    READY (2)             : Vehicle is "Ready" to drive or being driven
//    CHARGING (3)          : Vehicle is charging

static const OvmsVehicle::poll_pid_t obdii_polls[] = {
    // State variables polls
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_READY_SIGNAL, { 0, 1, 1, 0}, 0, ISOTP_STD },
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGING_LID, { 0, 1, 0, 1}, 0, ISOTP_STD },

    // Driving polls
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_VOLTAGE_AND_CURRENT, { 0, 1, 1, 1}, 0, ISOTP_STD },
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_ODOMETER, { 0, 0, 1, 0}, 0, ISOTP_STD },
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_VEHICLE_SPEED, { 0, 0, 1, 0}, 0, ISOTP_STD },
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AMBIENT_TEMPERATURE, {0, 60, 60, 60}, 0, ISOTP_STD },
  { HYBRID_BATTERY_SYSTEM_TX, HYBRID_BATTERY_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_TEMPERATURES, {0, 60, 60, 60}, 0, ISOTP_STD }, // {0, 60, 60, 60}
  { HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_SHIFT_POSITION, { 0, 0, 15, 0}, 0, ISOTP_STD }, 

    // Combined polls
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_SOC, { 0, 1, 1, 1}, 0, ISOTP_STD },

    // Charging polls
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_PISW_STATUS, { 0, 0, 0, 1}, 0, ISOTP_STD },
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGING, { 0, 0, 0, 1}, 0, ISOTP_STD },
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_CHARGING_POWER, { 0, 0, 0, 1}, 0, ISOTP_STD },
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGER_INPUT_POWER, { 0, 0, 0, 1}, 0, ISOTP_STD },
  { HYBRID_BATTERY_SYSTEM_TX, HYBRID_BATTERY_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_SOC_BMS, {0, 1, 1, 1}, 0, ISOTP_STD },
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
    PollSetState(PollState::SLEEP);

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
        ESP_LOGI(TAG, "%.0f, %.2f, %.4f, %.0f, %.2f, %.2f, %.2f, %.2f, %.0f, %.2f, %.4f",
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

        case PollState::READY:
            HandleReadyState();
            break;

        case PollState::CHARGING:
            HandleChargingState();
            break;

        default:
            ESP_LOGE(TAG, "Invalid poll state: %d", m_poll_state);
            break;
    }
}

void OvmsVehicleToyotaETNGA::Ticker60(uint32_t ticker)
{
    // Request VIN if not already set
if (StandardMetrics.ms_v_vin->AsString().empty() && (m_poll_state == PollState::READY)) {
        RequestVIN();
    }
}
