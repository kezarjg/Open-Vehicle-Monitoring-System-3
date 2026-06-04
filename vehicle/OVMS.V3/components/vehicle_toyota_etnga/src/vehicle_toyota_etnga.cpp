/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Toyota e-TNGA platform
   Date:          4th June 2023

   (C) 2023       Jerry Kezar <solterra@kezarnet.com>

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include "ovms_log.h"
#include "ovms_config.h"
#include "vehicle_toyota_etnga.h"

// Poll state descriptions:
//    SLEEP (0)             : Vehicle is sleeping; no activity on the CAN bus. We are listening only.
//    AWAKE (1)             : Vehicle is alive; vehicle has been switched on by driver
//    DRIVING (2)           : Vehicle is "Ready" to drive or being driven
//    CHARGE_HANDSHAKE (3)  : Cable-negotiation fast-poll window (was CHARGING)
//    CHARGE_WAIT (4)       : Plugged in, not (yet/any longer) charging — sparse poll
//    CHARGE_AC (5)         : AC charging in progress
//    CHARGE_DC (6)         : DC fast charging in progress

static const OvmsPoller::poll_pid_t obdii_polls[] = {
    // Column layout: { SLEEP, AWAKE, DRIVING, CHARGE_HANDSHAKE, CHARGE_WAIT, CHARGE_AC, CHARGE_DC }
    // Charge columns (indices 3-6) tuned from sim POLLS dict in bin/ovms-sim.py.

    // State variables polls
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CONTROL_SYSTEM_MODE,       { 0,  1, 1,  1,  1,  1,  1}, 0, ISOTP_STD }, // 0x10D1 ctrl-mode: all active states @1s (sim: all states)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGING_LID,              { 0, 10, 10, 0,  0,  0,  0}, 0, ISOTP_STD }, // 0x1625 charge lid: AWAKE+DRIVING only (sim: not in charge states)

    // Combined polls
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_SOC,               { 0,  0, 1,  0,  0,  1,  1}, 0, ISOTP_STD }, // 0x1738 SOC: DRIVING+AC+DC @1s (sim: absent in HS/WAIT)
  { HYBRID_BATTERY_SYSTEM_TX,  HYBRID_BATTERY_SYSTEM_RX,  VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_SOC_BMS,           { 0,  0, 1,  0,  0,  1,  1}, 0, ISOTP_STD }, // 0x1F5B BMS SOC: mirrors PID_BATTERY_SOC pattern (not in sim but module-local)
  { HYBRID_CONTROL_SYSTEM_TX,  HYBRID_CONTROL_SYSTEM_RX,  VEHICLE_POLL_TYPE_READDATA, PID_READY_SIGNAL,              { 0,  0, 1,  0,  0,  0,  0}, 0, ISOTP_STD }, // 0x1076 ready signal: DRIVING only (sim: not in charge states)
  { HYBRID_CONTROL_SYSTEM_TX,  HYBRID_CONTROL_SYSTEM_RX,  VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_VOLTAGE_AND_CURRENT,{ 0, 0, 1,  0,  0,  1,  1}, 0, ISOTP_STD }, // 0x1F9A pack I: DRIVING+AC+DC @1s (sim: DID_PACK_I AC=1,DC=1)
  { HYBRID_BATTERY_SYSTEM_TX,  HYBRID_BATTERY_SYSTEM_RX,  VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_TEMPERATURES,      { 0,  0, 10, 0,  0,  0, 20}, 0, ISOTP_STD }, // 0x1814 cell-temp array: DRIVING@10s, DC@20s (sim: DC=20 only)
  { HYBRID_BATTERY_SYSTEM_TX,  HYBRID_BATTERY_SYSTEM_RX,  VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_CELL_VOLTAGES,     { 0,  0, 5,  0,  0,  0, 30}, 0, ISOTP_STD }, // 0x182E cell voltages: DRIVING@5s, DC@30s (sim: DC=30 only)

    // Capacity arrays (data-collection only) — near-static, recalibrate over charge cycles; sampled at rest, driving, and both charge phases to watch drift
  { HYBRID_BATTERY_SYSTEM_TX,  HYBRID_BATTERY_SYSTEM_RX,  VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_CAPACITY,          { 0, 60, 120, 0,  0, 60, 60}, 0, ISOTP_STD }, // 0x1D3E 8x per-module full-charge capacity (Ah)
  { HYBRID_BATTERY_SYSTEM_TX,  HYBRID_BATTERY_SYSTEM_RX,  VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_CAPACITY_ALT,      { 0, 60, 120, 0,  0, 60, 60}, 0, ISOTP_STD }, // 0x1D3F 8x parallel capacity array, function unconfirmed

    // Driving polls
  { HYBRID_CONTROL_SYSTEM_TX,  HYBRID_CONTROL_SYSTEM_RX,  VEHICLE_POLL_TYPE_READDATA, PID_VEHICLE_SPEED,             { 0,  0, 1,  0,  0,  0,  0}, 0, ISOTP_STD }, // 0x1F0D speed: DRIVING only
  { HYBRID_CONTROL_SYSTEM_TX,  HYBRID_CONTROL_SYSTEM_RX,  VEHICLE_POLL_TYPE_READDATA, PID_SHIFT_POSITION,            { 0,  0, 1,  0,  0,  0,  0}, 0, ISOTP_STD }, // 0x1061 gear: DRIVING only
  { HYBRID_CONTROL_SYSTEM_TX,  HYBRID_CONTROL_SYSTEM_RX,  VEHICLE_POLL_TYPE_READDATA, PID_ODOMETER,                  { 0,  0, 1,  0,  0,  0,  0}, 0, ISOTP_STD }, // 0x1FA6 odometer: DRIVING only
  { AIR_CONDITIONER_TX,        AIR_CONDITIONER_RX,        VEHICLE_POLL_TYPE_READDATA, PID_AMBIENT_TEMPERATURE,       { 0,  0, 10, 0,  0,  0,  0}, 0, ISOTP_STD }, // 0x1002 ambient temp: DRIVING only
  { AIR_CONDITIONER_TX,        AIR_CONDITIONER_RX,        VEHICLE_POLL_TYPE_READDATA, PID_CABIN_TEMPERATURE,         { 0,  0, 10, 0,  0,  0,  0}, 0, ISOTP_STD }, // 0x1001 cabin temp: DRIVING only
  { AIR_CONDITIONER_TX,        AIR_CONDITIONER_RX,        VEHICLE_POLL_TYPE_READDATA, PID_HVAC_SETPOINT,             { 0,  0, 10, 0,  0,  0,  0}, 0, ISOTP_STD }, // 0x1036 HVAC setpoint: DRIVING only

    // Charging polls — state-machine DIDs
  { HYBRID_CONTROL_SYSTEM_TX,  HYBRID_CONTROL_SYSTEM_RX,  VEHICLE_POLL_TYPE_READDATA, PID_AMBIENT_TEMPERATURE_EV,   { 0,  0, 0,  0,  0,  0,  0}, 0, ISOTP_STD }, // 0x1F46: not polled by sim in any charge state; 0 until confirmed useful
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGING_CONTROL_STATUS,  { 0,  0, 0,  0,  0,  0,  0}, 0, ISOTP_STD }, // 0x1668: sim uses 0x1684 (PID_AC_CHARGING_OP_STATUS); 0x1668 not in sim charge lists
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_PISW_STATUS,              { 0,  5, 0,  1, 30, 10, 10}, 0, ISOTP_STD }, // 0x1669 PISW: AWAKE=5 (Task 5); HS=1,WAIT=30,AC=10,DC=10 (sim)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGING_VOLTAGE_TYPE,    { 0,  0, 0,  5,  0,  0,  0}, 0, ISOTP_STD }, // 0x161C VTYPE: HS=5s only (sim: HANDSHAKE=5, absent elsewhere)

//  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGING_CONTROL_INFORMATION, { 0, 0, 0, 1, 1, 1, 1}, 0, ISOTP_STD }, // 0x1689 deferred

  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AC_CHARGING_OP_STATUS,    { 0,  0, 0,  1, 30,  1,  1}, 0, ISOTP_STD }, // 0x1684 AC op: HS=1,WAIT=30,AC=1,DC=1 (sim)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_HLC_STATE,                { 0,  0, 0,  1, 30,  0,  1}, 0, ISOTP_STD }, // 0x1666 HLC: HS=1,WAIT=30,AC=0 (not needed),DC=1 (sim)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_MIN_PERMISSION_POWER,     { 0,  0, 0,  0,  0,  1,  1}, 0, ISOTP_STD }, // 0x16A1 perm power (curve): AC+DC=1s (sim DID_PERM_PWR)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_TARGET_CHARGING_CURRENT,  { 0,  0, 0,  0,  0,  1,  1}, 0, ISOTP_STD }, // 0x166D target current: AC+DC=1s (sim DID_TGT_CUR)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_BATTERY_CHARGING_POWER,   { 0,  0, 0,  0,  0,  1,  1}, 0, ISOTP_STD }, // 0x10D4 batt power: AC+DC=1s (sim); handler gated on charge-in-progress
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGER_INPUT_POWER,      { 0,  0, 0,  0,  0,  5,  5}, 0, ISOTP_STD }, // 0x161D grid power: AC+DC=5s (sim); handler gated on AC charge

  // DC station telemetry — present V/I (DC=1s) drive standard ms_v_charge_voltage/current; max P/I/V (DC=5s) are the station caps
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_DC_CHARGER_PRESENT_CURRENT,{ 0, 0, 0,  0,  0,  0,  1}, 0, ISOTP_STD }, // 0x166C DC station A: DC=1s (sim DID_STA_A)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_DC_CHARGER_PRESENT_VOLTAGE,{ 0, 0, 0,  0,  0,  0,  1}, 0, ISOTP_STD }, // 0x166B DC station V: DC=1s (sim DID_STA_V)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_DC_CHARGER_MAX_POWER,      { 0,  0, 0,  0,  0,  0,  5}, 0, ISOTP_STD }, // 0x166A station max power: DC=5s (sim DID_STA_MAX_P)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_DC_CHARGER_MAX_CURRENT,    { 0,  0, 0,  0,  0,  0,  5}, 0, ISOTP_STD }, // 0x1679 station max current: DC=5s (sim DID_STA_MAX_A)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_DC_CHARGER_MAX_VOLTAGE,    { 0,  0, 0,  0,  0,  0,  5}, 0, ISOTP_STD }, // 0x1681 station max voltage: DC=5s (sim DID_STA_MAX_V)

  // AC charger telemetry — AC-only (scale deferred pending sustained-AC capture)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGER_STATE_CLUSTER,     { 0,  0, 0,  0,  0,  1,  0}, 0, ISOTP_STD }, // 0x1619 charger state cluster: AC=1s (sim DID_CHG_STATE; AC-only)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGER_OUTPUT_POWER,      { 0,  0, 0,  0,  0,  5,  0}, 0, ISOTP_STD }, // 0x161E charger output cluster: AC=5s (sim DID_CHG_OUT_P; AC-only)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AC_USABLE_POWER,           { 0,  0, 0,  0,  0,  5,  0}, 0, ISOTP_STD }, // 0x1665 A/C useable power: AC=5s (sim DID_AC_USABLE; AC-only)

  // Charge-report supporting channels (increment 2) — My Room, A/C power, outcome, stop-request
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_MYROOM,            { 0,  0, 0,  0,  0,  5,  5}, 0, ISOTP_STD }, // 0x1692 My Room flag: AC+DC=5s (sim DID_MYROOM)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_AC_CONSUMPTION,    { 0,  0, 0,  0,  0,  2,  2}, 0, ISOTP_STD }, // 0x106E A/C consumption power: AC+DC=2s (sim DID_AC_CONS_P)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGE_HISTORY,    { 0,  0, 0,  5, 10,  5,  5}, 0, ISOTP_STD }, // 0x1688 outcome/history: HS=5,WAIT=10,AC=5,DC=5 (sim DID_HISTORY)
  { PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX, VEHICLE_POLL_TYPE_READDATA, PID_CHARGE_STOP_REQ,   { 0,  0, 0,  1, 30,  5,  5}, 0, ISOTP_STD }, // 0x1667 stop-request: HS=1,WAIT=30,AC=5,DC=5 (sim DID_STOP_REQ)

  // TPMS — gateway-relayed (0x750 sub-target 0x2A) via ISOTP_EXTADR; DRIVING @ 60s
  // (0x2A only answers while driving/My-Room — asleep parked & during charging, per the 2026-06-03 gateway census — so AWAKE polls just timed out)
  { TPMS_GW_TX, TPMS_GW_RX, VEHICLE_POLL_TYPE_READDATA, PID_TPMS_CORNERS,    { 0,  0, 60, 0, 0, 0, 0}, 0, ISOTP_EXTADR }, // 0x2021 slot->corner map
  { TPMS_GW_TX, TPMS_GW_RX, VEHICLE_POLL_TYPE_READDATA, PID_TPMS_PRESSURES, { 0,  0, 60, 0, 0, 0, 0}, 0, ISOTP_EXTADR }, // 0x1005 pressures (kPa)
  { TPMS_GW_TX, TPMS_GW_RX, VEHICLE_POLL_TYPE_READDATA, PID_TPMS_TEMPS,     { 0,  0, 60, 0, 0, 0, 0}, 0, ISOTP_EXTADR }, // 0x1004 temperatures (C)

    // Tester Present
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
    MyConfig.RegisterParam("xte", "Toyota eTNGA", true, true);

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

        case PollState::CHARGE_HANDSHAKE:
            HandleChargeHandshakeState();
            break;

        case PollState::CHARGE_WAIT:
            HandleChargeWaitState();
            break;

        case PollState::CHARGE_AC:
            HandleChargeAcState();
            break;

        case PollState::CHARGE_DC:
            HandleChargeDcState();
            break;

        default:
            ESP_LOGE(TAG, "Invalid poll state: %d", m_poll_state);
            break;
    }
}

