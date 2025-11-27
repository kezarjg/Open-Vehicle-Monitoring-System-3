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

void OvmsVehicleToyotaETNGA::HandleSleepState()
{
    int monotonic = StandardMetrics.ms_m_monotonic->AsInt();
    
    if (!m_allow_wake)
    {
        if ((monotonic - m_sleep_entry_time) > 10)
        {
            ESP_LOGI(TAG, "Cooling off period ended, allowing wake");
            m_allow_wake = true;
        }
    }

    if (StandardMetrics.ms_v_env_awake->AsBool()) {
        // There is life.
        TransitionToAwakeState();
    } else if (StandardMetrics.ms_v_bat_12v_voltage->AsFloat() > (StandardMetrics.ms_v_bat_12v_voltage_ref->AsFloat()+0.2f)) {
        // Voltage is high. Maybe awake as well...
        ESP_LOGI(TAG, "Aux 12V has exceeded the threshold");
        // Send a CAN reset.
        esp_err_t result = m_can2->Reset();
        if (result == ESP_OK) {
            ESP_LOGI(TAG, "CAN bus reset successfully");
        } else {
            ESP_LOGE(TAG, "CAN bus reset failed, error code: %d", result);
        }
        TransitionToAwakeState();
    }
}

void OvmsVehicleToyotaETNGA::HandleAwakeState()
{
    int monotonic = StandardMetrics.ms_m_monotonic->AsInt();
    
    if (!StandardMetrics.ms_v_env_awake->AsBool()) {
        // No CAN communication for 120s - stop polling
        TransitionToSleepState();
    }
    else if (m_s_controlstate->AsInt() == ControlState::CS_DRIVING) {
        // If the vehicle is switched on
        TransitionToDrivingState();
    }
    else if (m_s_controlstate->AsInt() == CS_CHARGING) {
        // If the vehicle starts charging
        TransitionToChargingState();
    }
    else if (monotonic - m_v_env_awaketime->AsInt() > 300) {
        // If the vehicle has been awake for 5 minutes without a clear state
        ESP_LOGD(TAG, "Vehicle awake for over 300s with no activity — forcing sleep state");
        m_sleep_entry_time = monotonic;
        m_allow_wake = false;
        TransitionToSleepState();
    }
}

void OvmsVehicleToyotaETNGA::HandleDrivingState()
{
    if (m_s_controlstate->AsInt() != ControlState::CS_DRIVING) {
        TransitionToAwakeState();
        SetReadyStatus(false);
        SetVehicleSpeed(0);
        SetShiftPosition(0);
        StandardMetrics.ms_v_env_temp->Clear();
        StandardMetrics.ms_v_env_cabintemp->Clear();
    }
}

void OvmsVehicleToyotaETNGA::HandleChargingState()
{
    if (m_s_controlstate->AsInt() != ControlState::CS_CHARGING) {
        TransitionToAwakeState();
        StandardMetrics.ms_v_env_temp->Clear();
        SetChargingStatus(false);
    }
}

void OvmsVehicleToyotaETNGA::TransitionToSleepState()
{
    // Perform actions needed for transitioning to the SLEEP state
    SetPollState(PollState::SLEEP); // Update the state
    SetAwake(false);
}

void OvmsVehicleToyotaETNGA::TransitionToAwakeState()
{
    // Perform actions needed for transitioning to the ACTIVE state
    SetPollState(PollState::AWAKE);
    m_v_env_awaketime->SetValue(StandardMetrics.ms_m_monotonic->AsInt());   // Store the time we entered to use as a timeout
}

void OvmsVehicleToyotaETNGA::TransitionToDrivingState()
{
    // Perform actions needed for transitioning to the READY state
    SetPollState(PollState::DRIVING); // Update the state
    m_v_pos_trip_start->SetStale(true);  // Set the start trip metric as stale so it resets next odometer reading
}

void OvmsVehicleToyotaETNGA::TransitionToChargingState()
{
    // Perform actions needed for transitioning to the CHARGING state

    // Get the one-time metrics for charging
    SetPollState(PollState::CHARGING); // Update the state
    SetChargingStatus(true);
}
