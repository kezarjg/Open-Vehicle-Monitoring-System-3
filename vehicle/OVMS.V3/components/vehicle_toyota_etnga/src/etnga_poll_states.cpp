/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Toyota e-TNGA platform
   Date:          4th June 2023

   (C) 2023       Jerry Kezar <solterra@kezarnet.com>

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include <time.h>
#include <sys/stat.h>
#include "ovms_log.h"
#include "vehicle_toyota_etnga.h"

// Poll state descriptions:
//    SLEEP (0)             : Vehicle is sleeping; no activity on the CAN bus. We are listening only.
//    AWAKE (1)             : Vehicle is alive; vehicle has been switched on by driver
//    DRIVING (2)           : Vehicle is "Ready" to drive or being driven
//    CHARGE_HANDSHAKE (3)  : Cable-negotiation fast-poll window (was CHARGING)
//    CHARGE_WAIT (4)       : Plugged in, not (yet/any longer) charging — sparse poll
//    CHARGE_AC (5)         : AC charging in progress
//    CHARGE_DC (6)         : DC fast charging in progress

// Escalating sleep cooldown schedule (seconds). Each consecutive no-activity sleep uses the
// next entry; real activity (drive/charge/charge-door/12V wake) resets to [0]. Caps at the
// last entry. See ResetSleepBackoff() and TransitionToSleepState().
static const int SLEEP_COOLDOWN_SECS[] = {10, 30, 60, 120, 300};
static const int SLEEP_COOLDOWN_STEPS  = (int)(sizeof(SLEEP_COOLDOWN_SECS) / sizeof(SLEEP_COOLDOWN_SECS[0]));

void OvmsVehicleToyotaETNGA::ResetSleepBackoff()
{
    m_sleep_backoff_idx = 0;
}

void OvmsVehicleToyotaETNGA::HandleSleepState()
{
    int monotonic = StandardMetrics.ms_m_monotonic->AsInt();
    
    if (!m_allow_wake)
    {
        if ((monotonic - m_sleep_entry_time) > m_sleep_cooldown_secs)
        {
            ESP_LOGI(TAG, "Cooling off period ended (%ds), allowing wake", m_sleep_cooldown_secs);
            m_allow_wake = true;
        }
    }

    if (StandardMetrics.ms_v_env_awake->AsBool()) {
        // There is life.
        TransitionToAwakeState();
    } else if (StandardMetrics.ms_v_bat_12v_voltage->AsFloat() > (StandardMetrics.ms_v_bat_12v_voltage_ref->AsFloat()+0.2f)) {
        // Voltage is high. Maybe awake as well...
        ESP_LOGI(TAG, "Aux 12V has exceeded the threshold");
        // Real power-up — resume responsive cooldowns.
        ResetSleepBackoff();
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

    // Wake-reconcile: if a charge session was open and the cable is now gone (confirmed
    // fresh read — not stale/transient), the cable was removed while we slept.
    // Guard: only act on a non-stale PISW value to avoid the ~30s OBC post-wake transient
    // where PISW may briefly report 0x00 before the OBC fully wakes.
    // REGRESSION DEPENDENCY: this requires PID_PISW_STATUS to be polled in the AWAKE
    // column (obdii_polls[] in vehicle_toyota_etnga.cpp, AWAKE=5s). Without an AWAKE poll
    // the cached PISW value is stale (gap >120s) or the pre-sleep connected value
    // (gap <120s), so a fresh 0x00 never arrives and this reconcile can NEVER fire —
    // leaking in_session and blocking the next session's open-guard. Do not remove the
    // AWAKE PISW poll without replacing this reconcile trigger.
    if (m_charge_session.in_session &&
        !m_v_charge_pisw_raw->IsStale() &&
        m_v_charge_pisw_raw->AsInt() == 0x00) {
        // Session ended while we slept (cable removed during the sleep gap).
        ESP_LOGI(TAG, "Charge session ended during sleep — finalizing");
        StandardMetrics.ms_v_charge_state->SetValue("done");
        m_charge_session = ChargeSessionState{};
        // fall through to normal AWAKE handling
    }

    if (!StandardMetrics.ms_v_env_awake->AsBool()) {
        // No CAN communication - bus is dead, go to sleep
        ESP_LOGI(TAG, "CAN bus idle (env_awake cleared) — sleeping");
        TransitionToSleepState();
        return;
    }
    else if (m_s_controlstate->AsInt() == ControlState::CS_DRIVING) {
        // HV system is up — vehicle is READY/driving
        TransitionToDrivingState();
        return;
    }

    // PISW-based arm logic (mirrors sim S.AWAKE branch)
    int pisw = m_v_charge_pisw_raw->AsInt();
    bool lid_open = StandardMetrics.ms_v_door_chargeport->AsBool();

    // pisw is fresh here mainly after a HANDSHAKE->AWAKE bounce (pisw not polled in
    // plain AWAKE); cold cable-detect relies on the lid-open arm logic below.
    if (pisw >= 0x02) {
        // Cable is seated — enter charge handshake immediately
        TransitionToChargeHandshakeState();
        return;
    }
    if (!m_armed_for_charge) {
        if (lid_open) {
            // Charge door just opened: arm and start the 15-min cable watch
            m_armed_for_charge = true;
            m_cable_watch_start = monotonic;
            ResetSleepBackoff();   // deliberate user action — resume responsive cooldowns
        } else if (monotonic - m_v_env_awaketime->AsInt() > 300) {
            // Door watch expired (5 min in AWAKE, charge door never opened) → sleep
            ESP_LOGI(TAG, "Vehicle awake for over 300s with no activity — forcing sleep state");
            TransitionToSleepState();
            return;
        }
    } else if (monotonic - m_cable_watch_start > 900) {
        // Cable watch expired (15 min armed, no cable plug-in) → sleep
        ESP_LOGI(TAG, "Armed 15min, no cable plug-in — giving up");
        TransitionToSleepState();
        return;
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

void OvmsVehicleToyotaETNGA::HandleChargeHandshakeState()
{
    int pisw = m_v_charge_pisw_raw->AsInt();
    int ac_op = m_v_charge_ac_op->AsInt();
    int hlc = m_v_charge_hlc->AsInt();
    int monotonic = StandardMetrics.ms_m_monotonic->AsInt();

    if (pisw == 0x00) {
        // Premature unplug — re-arm logic handled in TransitionToAwakeState
        TransitionToAwakeState();
        return;
    }
    if (hlc >= 0x0A && hlc <= 0x12) {
        // DC HLC sequence active
        TransitionToChargeDcState();
        return;
    }
    // HANDSHAKE: only ac_op==0x02 (Running), NOT 0x01 (Startup) — avoids premature
    // AC entry during negotiation. (CHARGE_WAIT deliberately accepts 0x01 too.)
    if (ac_op == 0x02) {
        TransitionToChargeAcState();
        return;
    }
    if (monotonic - m_charge_state_entry >= 60 && ac_op == 0x00 && pisw >= 0x02) {
        // Scheduled-wait heuristic: AC Op stuck at Stop for 60s with cable present
        TransitionToChargeWaitState();
        return;
    }
}

void OvmsVehicleToyotaETNGA::HandleChargeWaitState()
{
    int pisw = m_v_charge_pisw_raw->AsInt();
    int ac_op = m_v_charge_ac_op->AsInt();
    int hlc = m_v_charge_hlc->AsInt();

    if (pisw == 0x00) {
        // Cable removed — session ended
        TransitionToAwakeState();
        return;
    }
    if (ac_op == 0x01 || ac_op == 0x02) {
        // EVSE engaged (Startup or Running)
        TransitionToChargeAcState();
        return;
    }
    if (hlc >= 0x0A && hlc <= 0x12) {
        // DC engaged
        TransitionToChargeDcState();
        return;
    }
    if (!StandardMetrics.ms_v_env_awake->AsBool()) {
        // Bus went dead during scheduled wait (OBC slept or gateway isolated OBD)
        TransitionToSleepState();
        return;
    }
}

void OvmsVehicleToyotaETNGA::HandleChargeAcState()
{
    // Lock isolation note: locking the car during AC charging causes sustained OBC poll
    // timeouts (gateway isolates OBD from OBC).  We do NOT tear down the session on
    // timeouts — only an explicit fresh PISW=Unconnected (0x00) read or a clean phase-end
    // (ac_op==0x00) terminates the session.  On unlock the OBC answers again and these
    // handlers resume normally.  Do not add timeout-based session teardown here.
    UpdateChargeSessionStats();   // aggregate peak power / temp range / type for the session report

    int pisw = m_v_charge_pisw_raw->AsInt();
    int ac_op = m_v_charge_ac_op->AsInt();

    if (ac_op == 0x00) {
        // AC Op = Stop: phase ended
        TransitionToChargeWaitState();
        return;
    }
    if (pisw == 0x00) {
        // Cable pulled mid-charge
        TransitionToChargeWaitState();
        return;
    }
}

void OvmsVehicleToyotaETNGA::HandleChargeDcState()
{
    // Lock isolation note: same as HandleChargeAcState — sustained OBC timeouts while
    // in_session do NOT tear down the session.  Only an explicit fresh PISW=Unconnected
    // (0x00) read or hlc==0xFF (HLC Unconnected) terminates the DC phase.  On unlock the
    // OBC answers again and polling resumes.  Do not add timeout-based session teardown here.
    UpdateChargeSessionStats();   // aggregate peak power / temp range / type for the session report

    int pisw = m_v_charge_pisw_raw->AsInt();
    int hlc = m_v_charge_hlc->AsInt();

    if (hlc == 0xFF) {
        // HLC = Unconnected (255): DC phase ended
        TransitionToChargeWaitState();
        return;
    }
    if (pisw == 0x00) {
        // Cable pulled mid-DC
        TransitionToChargeWaitState();
        return;
    }
}

void OvmsVehicleToyotaETNGA::TransitionToSleepState()
{
    int monotonic = StandardMetrics.ms_m_monotonic->AsInt();
    m_armed_for_charge = false;
    // Arm the escalating cooldown: ignore CAN-frame wakes for the current window, then step
    // the index up (clamped) so the next consecutive no-activity sleep waits longer.
    // ResetSleepBackoff() returns the index to 0 on real activity.
    m_sleep_entry_time = monotonic;
    m_sleep_cooldown_secs = SLEEP_COOLDOWN_SECS[m_sleep_backoff_idx];
    m_allow_wake = false;
    if (m_sleep_backoff_idx < SLEEP_COOLDOWN_STEPS - 1)
        m_sleep_backoff_idx++;
    SetPollState(PollState::SLEEP);
    SetAwake(false);
}

void OvmsVehicleToyotaETNGA::TransitionToAwakeState()
{
    int monotonic = StandardMetrics.ms_m_monotonic->AsInt();
    PollState oldState = static_cast<PollState>(m_poll_state);
    // If bouncing back from CHARGE_HANDSHAKE (premature unplug / DCFC retry dance),
    // re-arm and restart the cable watch so the next plug-in gets a fresh 15-min window.
    // Check oldState BEFORE calling SetPollState (sim: armed_for_charge=True on HANDSHAKE→AWAKE).
    if (oldState == PollState::CHARGE_HANDSHAKE) {
        m_armed_for_charge = true;
        m_cable_watch_start = monotonic;
        LogChargeEvent("Unplug bounce — re-armed (DCFC retry)");
        ESP_LOGD(TAG, "Re-armed after HANDSHAKE→AWAKE bounce (DCFC retry)");
    }
    // For all other transitions into AWAKE (SLEEP→AWAKE, DRIVING→AWAKE),
    // arm state is reset by TransitionToSleepState / TransitionToDrivingState.
    SetPollState(PollState::AWAKE);
    m_v_env_awaketime->SetValue(monotonic);
    // Leaving the charge sub-machine: close the session. ms_v_charge_state is "done" only
    // when energy was being delivered (AC/DC — normally these exit via CHARGE_WAIT; the
    // direct path is a safety net); HANDSHAKE/WAIT exits just clear it.
    if (oldState >= PollState::CHARGE_HANDSHAKE) {
        bool delivered = (oldState == PollState::CHARGE_AC || oldState == PollState::CHARGE_DC);
        StandardMetrics.ms_v_charge_state->SetValue(delivered ? "done" : "");
        StandardMetrics.ms_v_charge_mode->SetValue("");   // clear AC/DC indicator on session end
        if (m_charge_session.in_session) {
            ESP_LOGI(TAG, "Charge session closed");
            LogChargeEvent("Unplugged");
            GenerateChargeReport();   // write the session-end HTML report (no-op if no energy delivered)
        }
        m_charge_session = ChargeSessionState{};   // reset (clears in_session)
    }
}

void OvmsVehicleToyotaETNGA::TransitionToDrivingState()
{
    m_armed_for_charge = false;
    ResetSleepBackoff();   // real drive — resume responsive cooldowns
    SetPollState(PollState::DRIVING);
    m_v_pos_trip_start->SetStale(true);
    RequestVIN();
}

void OvmsVehicleToyotaETNGA::TransitionToChargeHandshakeState()
{
    m_armed_for_charge = false;  // arm state consumed on entering handshake
    ResetSleepBackoff();   // charge session beginning — resume responsive cooldowns
    m_charge_state_entry = StandardMetrics.ms_m_monotonic->AsInt();
    SetPollState(PollState::CHARGE_HANDSHAKE);
    SetChargingStatus(false);    // not yet delivering energy (AC/DC states set true)
    SetChargeState(PollState::CHARGE_HANDSHAKE);
    if (!m_charge_session.in_session) {
        m_charge_session.in_session = true;
        m_charge_session.start_monotonic = StandardMetrics.ms_m_monotonic->AsInt();
        m_charge_session.start_utc = StandardMetrics.ms_m_timeutc->AsInt();
        m_charge_session.start_soc = (int) StandardMetrics.ms_v_bat_soc->AsFloat();
        if (StandardMetrics.ms_v_pos_gpslock->AsBool()) {
            m_charge_session.has_loc = true;
            m_charge_session.start_lat = StandardMetrics.ms_v_pos_latitude->AsFloat();
            m_charge_session.start_lon = StandardMetrics.ms_v_pos_longitude->AsFloat();
        }
        // Seed the ambient range only from a fresh reading. env temp is Clear()ed when the
        // car parks, so seeding unconditionally produced a bogus "0 -> 0 C". The in-charge
        // 0x1F46 poll (enabled in the poll list) fills the range as readings arrive.
        if (StandardMetrics.ms_v_env_temp->IsDefined()) {
            float amb = StandardMetrics.ms_v_env_temp->AsFloat();
            m_charge_session.amb_seen = true;
            m_charge_session.amb_min = m_charge_session.amb_max = amb;
        }
        m_charge_session.svg_interval_s = 20;
        m_charge_session.last_sample_monotonic = 0;
        m_charge_session.last_svg_monotonic = 0;
        // basename = "<dir>/<UTC timestamp>" (no extension); files are <base>.html / <base>.csv
        {
            char ts[40];
            int utc = m_charge_session.start_utc;
            if (utc > 1000000000) {
                time_t st = (time_t) utc; struct tm tmv; gmtime_r(&st, &tmv);
                strftime(ts, sizeof(ts), "%Y%m%dT%H%M%SZ", &tmv);
            } else {
                snprintf(ts, sizeof(ts), "charge-%d", m_charge_session.start_monotonic);
            }
            mkdir(ChargeReportDir().c_str(), 0755);
            m_charge_session.base = ChargeReportDir() + "/" + ts;
        }
        LogChargeEvent("Plugged in — handshake");
        ESP_LOGI(TAG, "Charge session opened (SOC %d%%)", m_charge_session.start_soc);
    }
    RequestVIN();
}

void OvmsVehicleToyotaETNGA::TransitionToChargeWaitState()
{
    m_charge_state_entry = StandardMetrics.ms_m_monotonic->AsInt();
    SetPollState(PollState::CHARGE_WAIT);
    SetChargingStatus(false);
    SetChargeState(PollState::CHARGE_WAIT);
    LogChargeEvent("Charging paused / phase ended");
}

void OvmsVehicleToyotaETNGA::TransitionToChargeAcState()
{
    m_charge_state_entry = StandardMetrics.ms_m_monotonic->AsInt();
    SetPollState(PollState::CHARGE_AC);
    SetChargingStatus(true);
    SetChargeState(PollState::CHARGE_AC);
    StandardMetrics.ms_v_charge_mode->SetValue("standard");      // AC charging (OVMS-standard v.c.mode)
    LogChargeEvent("AC charging started");
}

void OvmsVehicleToyotaETNGA::TransitionToChargeDcState()
{
    m_charge_state_entry = StandardMetrics.ms_m_monotonic->AsInt();
    SetPollState(PollState::CHARGE_DC);
    SetChargingStatus(true);
    SetChargeState(PollState::CHARGE_DC);
    StandardMetrics.ms_v_charge_mode->SetValue("performance");   // DC fast charge -> ABRP is_dcfc (v.c.mode == "performance")
    LogChargeEvent("DC charging started");
}
