/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Toyota e-TNGA platform
   Date:          4th June 2023

   (C) 2023       Jerry Kezar <solterra@kezarnet.com>

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include "ovms_log.h"
#include "vehicle_toyota_etnga.h"

void OvmsVehicleToyotaETNGA::IncomingPollReply(const OvmsPoller::poll_job_t &job, uint8_t* data, uint8_t length)
{
    // Check if this is the first frame of the multi-frame response
    if (job.mlframe == 0) {
        m_rxbuf.clear();
        m_rxbuf.reserve(length + job.mlremain);
    }

    // Append the data to the receive buffer
    m_rxbuf.append(reinterpret_cast<char*>(data), length);

    // Check if response is complete
    if (job.mlremain != 0)
        return;

    // Log the received response
    ESP_LOGV(TAG, "IncomingPollReply: PID %02X: len=%d %s", job.pid, m_rxbuf.size(), hexencode(m_rxbuf).c_str());

    // Process based on m_poll_moduleid_low
    switch (job.moduleid_rec) {
        case AIR_CONDITIONER_RX:
            IncomingAirConditionerSystem(job.pid);
            break;

        case HYBRID_BATTERY_SYSTEM_RX:
            IncomingHybridBatterySystem(job.pid);
            break;

        case HYBRID_CONTROL_SYSTEM_RX:
            IncomingHybridControlSystem(job.pid);
            break;

        case PLUG_IN_CONTROL_SYSTEM_RX:
            IncomingPlugInControlSystem(job.pid);
            break;

        case HPCM_HYBRIDPTCTR_RX:
            IncomingHPCMHybridPtCtr(job.pid);
            break;

        case TPMS_GW_RX:
            IncomingTPMS(job.pid);
            break;

        default:
            ESP_LOGW(TAG, "Unknown module: %03" PRIx32, job.moduleid_rec);
            return;
    }
}

void OvmsVehicleToyotaETNGA::IncomingAirConditionerSystem(uint16_t pid)
{
    switch (pid) {
        case PID_AMBIENT_TEMPERATURE: {
            float temperature = CalculateAmbientTemperature(m_rxbuf);
            SetAmbientTemperature(temperature);
            break;
        }

        case PID_CABIN_TEMPERATURE: {
            float temperature = CalculateCabinTemperature(m_rxbuf);
            SetCabinTemperature(temperature);
            break;
        }

        case PID_HVAC_SETPOINT: {
            float temperature = CalculateHVACSetpoint(m_rxbuf);
            SetHVACSetpoint(temperature);
            break;
        }

        // Add more cases for other PIDs if needed

        default:
            // Handle unsupported PID
            ESP_LOGW(TAG, "Unsupported PID: %04X", pid);
            break;
    }
}

void OvmsVehicleToyotaETNGA::IncomingHybridControlSystem(uint16_t pid)
{
    switch (pid) {
        case PID_ACTIVE_DIAGNOSTIC_SESSION: {
            break;
        }

        case PID_AMBIENT_TEMPERATURE_EV: {
            float temperature = CalculateAmbientTemperatureEV(m_rxbuf);
            SetAmbientTemperature(temperature);
            break;
        }

        case PID_BATTERY_VOLTAGE_AND_CURRENT: {
            float batVoltage = CalculateBatteryVoltage(m_rxbuf);
            float batCurrent = CalculateBatteryCurrent(m_rxbuf);
            float batPower = CalculateBatteryPower(batVoltage, batCurrent);

            SetBatteryVoltage(batVoltage);
            SetBatteryCurrent(batCurrent);
            SetBatteryPower(batPower);

            break;
        }

        case PID_READY_SIGNAL: {
            bool readyStatus = CalculateReadyStatus(m_rxbuf);
            SetReadyStatus(readyStatus);
            break;
        }

        case PID_SHIFT_POSITION: {
            int shiftPosition = CalculateShiftPosition(m_rxbuf);
            SetShiftPosition(shiftPosition);
            break;
        }

        case PID_ODOMETER: {
            float odometer = CalculateOdometer(m_rxbuf);
            SetOdometer(odometer);
            break;
        }

        case PID_VEHICLE_SPEED: {
            float speed = CalculateVehicleSpeed(m_rxbuf);
            SetVehicleSpeed(speed);
            break;
        }

        // Add more cases for other PIDs if needed

        default:
            // Handle unsupported PID
            ESP_LOGW(TAG, "Unsupported PID: %04X", pid);
            break;
    }
}

void OvmsVehicleToyotaETNGA::IncomingPlugInControlSystem(uint16_t pid)
{
    switch (pid) {
        case PID_ACTIVE_DIAGNOSTIC_SESSION: {
            break;
        }

        case PID_BATTERY_SOC: {
            float SOC = CalculateBatterySOC(m_rxbuf);
            SetBatterySOC(SOC);
            break;
        }

        case PID_CHARGING_LID: {
            bool chargingDoorStatus = CalculateChargingDoorStatus(m_rxbuf);
            SetChargingDoorStatus(chargingDoorStatus);
            break;
        }

        case PID_CHARGING_CONTROL_STATUS: {
            int chargeMode = CalculateChargeMode(m_rxbuf);
            SetChargeMode(chargeMode);
            break;
        }

        case PID_CONTROL_SYSTEM_MODE: {
            int controlMode = CalculateControlMode(m_rxbuf);
            SetControlMode(controlMode);
            break;
        }

        case PID_PISW_STATUS: {
            bool PISWStatus = CalculatePISWStatus(m_rxbuf);
            SetPISWStatus(PISWStatus);
            SetPISWRaw(CalculatePISWRaw(m_rxbuf));
            break;
        }

        case PID_AC_CHARGING_OP_STATUS: {
            SetAcOpStatus(CalculateAcOpStatus(m_rxbuf));
            break;
        }

        case PID_HLC_STATE: {
            SetHlcState(CalculateHlcState(m_rxbuf));
            break;
        }

        case PID_BATTERY_CHARGING_POWER: {
            // Only valid during AC or DC charging
            if (StandardMetrics.ms_v_charge_inprogress->AsBool()) {
                float batteryChargingPower = CalculateBatteryChargingPower(m_rxbuf);
                SetBatteryChargingPower(batteryChargingPower);
            }
            break;
        }

        case PID_CHARGER_INPUT_POWER: {
            // Only valid during AC charging
            if (StandardMetrics.ms_v_charge_inprogress->AsBool() && std::string(StandardMetrics.ms_v_charge_mode->AsString()) == "Standard") {
                float chargerInputPower = CalculateChargerInputPower(m_rxbuf);
                SetChargerInputPower(chargerInputPower);
            }
            break;
        }

        case PID_CHARGING_VOLTAGE_TYPE: {
            int chargeType = CalculateChargeType(m_rxbuf);
            SetChargeType(chargeType);
            break;
        }

        case PID_DC_CHARGER_PRESENT_CURRENT:
            SetStationCurrent(CalculateStationCurrent(m_rxbuf));
            break;

        case PID_DC_CHARGER_PRESENT_VOLTAGE:
            SetStationVoltage(CalculateStationVoltage(m_rxbuf));
            break;

        case PID_MIN_PERMISSION_POWER: {
            if (GetRxBUint16(m_rxbuf, 0) != 0x8000)  // 0x8000 = feature inactive; preserve forward-fill
                SetPermissionPower(CalculatePermissionPower(m_rxbuf));
            break;
        }

        case PID_TARGET_CHARGING_CURRENT: {
            SetTargetCurrent(CalculateTargetCurrent(m_rxbuf));
            break;
        }

        case PID_DC_CHARGER_MAX_POWER: {
            SetStationMaxPower(CalculateStationMaxPower(m_rxbuf));
            break;
        }
        case PID_DC_CHARGER_MAX_CURRENT: {
            SetStationMaxCurrent(CalculateStationMaxCurrent(m_rxbuf));
            break;
        }
        case PID_DC_CHARGER_MAX_VOLTAGE: {
            SetStationMaxVoltage(CalculateStationMaxVoltage(m_rxbuf));
            break;
        }

        case PID_CHARGER_STATE_CLUSTER: {
            SetAcTargetPower(CalculateAcTargetPower(m_rxbuf));
            SetChargerOpStatus(CalculateChargerOpStatus(m_rxbuf));
            SetAcCurrentLimitRaw(CalculateAcCurrentLimitRaw(m_rxbuf));
            break;
        }
        case PID_CHARGER_OUTPUT_POWER: {
            SetChargerOutputRaw(CalculateChargerOutputRaw(m_rxbuf));
            SetChargerOutputTargetRaw(CalculateChargerOutputTargetRaw(m_rxbuf));
            break;
        }
        case PID_AC_USABLE_POWER: {
            SetAcUsableRaw(CalculateAcUsableRaw(m_rxbuf));
            break;
        }

        case PID_MYROOM: {
            SetMyRoom(CalculateMyRoom(m_rxbuf));
            break;
        }
        case PID_AC_CONSUMPTION: {
            SetAcConsumption(CalculateAcConsumption(m_rxbuf));
            break;
        }
        case PID_CHARGE_HISTORY: {
            SetChargeOutcome(CalculateChargeOutcome(m_rxbuf));
            break;
        }
        case PID_CHARGE_STOP_REQ: {
            SetChargeStopReq(CalculateChargeStopReq(m_rxbuf));
            break;
        }

        // Add more cases for other PIDs if needed

        default:
            // Handle unsupported PID
            ESP_LOGW(TAG, "Unsupported PID: %04X", pid);
            break;
    }
}

void OvmsVehicleToyotaETNGA::IncomingHybridBatterySystem(uint16_t pid)
{
    switch (pid) {
        case PID_ACTIVE_DIAGNOSTIC_SESSION: {
            break;
        }

        case PID_BATTERY_SOC_BMS: {
            float SOC = CalculateBatterySOCBMS(m_rxbuf);
            SetBatterySOCBMS(SOC);
            break;
        }

        case PID_BATTERY_TEMPERATURES: {
            std::vector<float> temperatures = CalculateBatteryTemperatures(m_rxbuf);
            SetBatteryTemperatures(temperatures);
            SetBatteryTemperatureStatistics(temperatures);
            break;
        }

        case PID_BATTERY_CELL_VOLTAGES: {
            std::vector<float> voltages = CalculateBatteryCellVoltages(m_rxbuf);
            SetBatteryCellVoltages(voltages);
            SetBatteryCellVoltageStatistics(voltages);
            break;
        }

        case PID_BATTERY_CAPACITY: {
            std::vector<float> caps = CalculateBatteryCapacityArray(m_rxbuf);
            SetBatteryCapacityFull(caps);
            if (caps.size() >= 8)
                ESP_LOGD(TAG, "Battery capacity 0x1D3E (Ah): %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                         caps[0], caps[1], caps[2], caps[3], caps[4], caps[5], caps[6], caps[7]);
            break;
        }

        case PID_BATTERY_CAPACITY_ALT: {
            std::vector<float> caps = CalculateBatteryCapacityArray(m_rxbuf);
            SetBatteryCapacityAlt(caps);
            if (caps.size() >= 8)
                ESP_LOGD(TAG, "Battery capacity 0x1D3F (Ah): %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                         caps[0], caps[1], caps[2], caps[3], caps[4], caps[5], caps[6], caps[7]);
            break;
        }

        // Add more cases for other PIDs if needed

        default:
            // Handle unsupported PID
            ESP_LOGW(TAG, "Unsupported PID: %04X", pid);
            break;
    }
}

void OvmsVehicleToyotaETNGA::IncomingHPCMHybridPtCtr(uint16_t pid)
{
    switch (pid) {

        // Add more cases for other PIDs if needed

        default:
            // Handle unsupported PID
            ESP_LOGD(TAG, "Unsupported PID: %04X", pid);
            break;
    }
}

void OvmsVehicleToyotaETNGA::RequestVIN()
{
    if (!StandardMetrics.ms_v_vin->AsString().empty())
        return;

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;
    using std::placeholders::_5;
    using std::placeholders::_6;

    auto entry = std::shared_ptr<OvmsPoller::OnceOffPoll>(
        new OvmsPoller::OnceOffPoll(
            std::bind(&OvmsVehicleToyotaETNGA::IncomingVINSuccess, this, _1, _2, _3, _4, _5, _6),
            std::bind(&OvmsVehicleToyotaETNGA::IncomingVINFail,    this, _1, _2, _3, _4, _5),
            HYBRID_CONTROL_SYSTEM_TX, HYBRID_CONTROL_SYSTEM_RX,
            VEHICLE_POLL_TYPE_READDATA, PID_VIN,
            ISOTP_STD, 0, /*retry_fail=*/3));
    PollRequest(m_can2, "!xte.vin", entry);
}

void OvmsVehicleToyotaETNGA::IncomingVINSuccess(uint16_t type, uint32_t module_sent, uint32_t module_rec, uint16_t pid, CAN_frame_format_t format, const std::string &data)
{
    SetVehicleVIN(data);
}

void OvmsVehicleToyotaETNGA::IncomingVINFail(uint16_t type, uint32_t module_sent, uint32_t module_rec, uint16_t pid, int errorcode)
{
    ESP_LOGW(TAG, "RequestVIN: Failed with error code %d", errorcode);
}

void OvmsVehicleToyotaETNGA::DiagnosticSession()
{
    std::string response;
    int res = PollSingleRequest(
        m_can2,
        PLUG_IN_CONTROL_SYSTEM_TX,
        PLUG_IN_CONTROL_SYSTEM_RX,
        VEHICLE_POLL_TYPE_OBDIISESSION,
        0x03,
        response,
        1000,
        ISOTP_STD
    );

    if (res == POLLSINGLE_OK)
    {
        ESP_LOGW(TAG, "DiagnosticSession: Worked");
    }
    else
    {
        ESP_LOGW(TAG, "DiagnosticSession: Failed with error code %d", res);
    }
}

void OvmsVehicleToyotaETNGA::RequestChargeMode()
{
    std::string response;
    int chargeMode;
    int maxRetries = 5;
    int retryCount = 0;
    int res = POLLSINGLE_TIMEOUT;  // non-OK sentinel so the retry loop runs at least once

    while (retryCount < maxRetries && res != POLLSINGLE_OK)
    {
        res = PollSingleRequest(
            m_can2,
            PLUG_IN_CONTROL_SYSTEM_TX,
            PLUG_IN_CONTROL_SYSTEM_RX,
            VEHICLE_POLL_TYPE_READDATA,
            PID_CHARGING_CONTROL_STATUS,
            response,
            1000,
            ISOTP_STD
        );

        if (res == POLLSINGLE_OK)
        {
            // Request successful
            chargeMode = response[0] & 0xFF;
            SetChargeMode(chargeMode);
            break;
        }
        else
        {
            retryCount++;
            ESP_LOGW(TAG, "RequestChargeMode: Request failed with error code %d. Retrying (%d/%d)", res, retryCount, maxRetries);
        }
    }

    if (res != POLLSINGLE_OK)
    {
        ESP_LOGE(TAG, "RequestChargeMode: Maximum retries reached. Request failed with error code %d", res);
    }
}

void OvmsVehicleToyotaETNGA::RequestChargeType()
{
    std::string response;
    int chargeType;
    int maxRetries = 5;
    int retryCount = 0;
    int res = POLLSINGLE_TIMEOUT;  // non-OK sentinel so the retry loop runs at least once

    while (retryCount < maxRetries && res != POLLSINGLE_OK)
    {
        res = PollSingleRequest(
            m_can2,
            PLUG_IN_CONTROL_SYSTEM_TX,
            PLUG_IN_CONTROL_SYSTEM_RX,
            VEHICLE_POLL_TYPE_READDATA,
            PID_CHARGING_VOLTAGE_TYPE,
            response,
            1000,
            ISOTP_STD
        );

        if (res == POLLSINGLE_OK)
        {
            // Request successful
            chargeType = response[0] & 0xFF;
            SetChargeType(chargeType);
            break;
        }
        else
        {
            retryCount++;
            ESP_LOGW(TAG, "RequestChargeType: Request failed with error code %d. Retrying (%d/%d)", res, retryCount, maxRetries);
        }
    }

    if (res != POLLSINGLE_OK)
    {
        ESP_LOGE(TAG, "RequestChargeType: Maximum retries reached. Request failed with error code %d", res);
    }
}
