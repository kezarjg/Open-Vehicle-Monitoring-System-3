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

// CHARGE_WAIT 12V-drain protection: stop polling after a sustained idle wait so the bus
// idles and the 12V recovers (session is preserved; resumes on passive wake). First wait
// gets a responsive window; a wait re-entered after a prior sleep re-sleeps quickly to keep
// the oscillation duty-cycle low. See HandleChargeWaitState().
static const int CHARGE_WAIT_SLEEP_SECS   = 600;  // first wait, no charge -> sleep
static const int CHARGE_WAIT_RESLEEP_SECS = 15;   // re-entered after a wait-sleep -> sleep fast

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
    } else {
        // 12V wake is RISING-EDGE only. Level-triggering here oscillated: the post-drive/
        // post-charge surface charge keeps 12V above ref+0.2 for a long time with a dead
        // bus, so every SLEEP tick reset the backoff, reset the CAN controller and bounced
        // to AWAKE — a ~2s SLEEP/AWAKE loop (one CAN reset per cycle) whose sleep re-entry
        // also restarted the cooldown, so frame-wake never re-enabled until 12V decayed.
        float v12 = StandardMetrics.ms_v_bat_12v_voltage->AsFloat();
        float ref = StandardMetrics.ms_v_bat_12v_voltage_ref->AsFloat();
        bool high = v12 > ref + 0.2f;
        if (high && !m_12v_was_high) {
            // Voltage just jumped: real power-up (DC-DC came on) — wake even during cooldown.
            ESP_LOGI(TAG, "Aux 12V has exceeded the threshold");
            ResetSleepBackoff();
            // Send a CAN reset.
            esp_err_t result = m_can2->Reset();
            if (result == ESP_OK) {
                ESP_LOGI(TAG, "CAN bus reset successfully");
            } else {
                ESP_LOGE(TAG, "CAN bus reset failed, error code: %d", result);
            }
            // Lift the cooldown gate so incoming CAN frames can hold us awake; without this
            // a 12V wake during cooldown bounced straight back to sleep even with the
            // vehicle genuinely on (IncomingFrameCan2 was still discarding frames).
            m_allow_wake = true;
            TransitionToAwakeState();
        }
        // Hysteresis on the latch: set above ref+0.2, clear only below ref+0.1 — a level
        // hovering at the threshold (ADC noise) must not produce repeated edges/CAN resets.
        if (high)
            m_12v_was_high = true;
        else if (v12 < ref + 0.1f)
            m_12v_was_high = false;
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
    // column (obdii_polls_base[] in vehicle_toyota_etnga.cpp, AWAKE=5s). Without an AWAKE poll
    // the cached PISW value is stale (gap >120s) or the pre-sleep connected value
    // (gap <120s), so a fresh 0x00 never arrives and this reconcile can NEVER fire —
    // leaking in_session and blocking the next session's open-guard. Do not remove the
    // AWAKE PISW poll without replacing this reconcile trigger.
    // Debounce the cable-removed read: the OBC can briefly report PISW=0x00 for ~30s after
    // wake before it is fully up. Require two consecutive fresh 0x00 reads before finalizing,
    // so a wake-from-CHARGE_WAIT-sleep transient does not falsely close a still-plugged session.
    if (!m_v_charge_pisw_raw->IsStale()) {
        if (m_v_charge_pisw_raw->AsInt() == 0x00)
            m_pisw_zero_count++;
        else
            m_pisw_zero_count = 0;
    }
    if (m_charge_session.in_session && m_pisw_zero_count >= 2) {
        // Session ended while we slept (cable removed during the sleep gap).
        // Close the session the same way TransitionToAwakeState does: flush + report.
        // Resetting without the report silently discarded the whole session (and the
        // streamed CSV was then deleted as an orphan by PruneChargeReports).
        ESP_LOGI(TAG, "Charge session ended during sleep — finalizing");
        StandardMetrics.ms_v_charge_state->SetValue("done");
        LogChargeEvent("Unplugged (during sleep)");
        GenerateChargeReport();   // flushes the CSV; no-op + stub-CSV cleanup if no energy delivered
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

    // pisw is polled @5s in AWAKE (obdii_polls_base[]; also required by the wake-reconcile
    // above), so a seated cable is detected here directly. The lid-open arm logic below
    // only bounds how long we stay awake waiting for a plug-in.
    if (pisw >= 0x02) {
        // Cable is seated. If a session is already open we are resuming after a CHARGE_WAIT
        // sleep — go straight back to CHARGE_WAIT (the engage-watch + direct AC/DC checks there
        // catch the charge when it starts) and let Tier 2 re-sleep. Only a genuinely new
        // plug-in (no open session) runs the full handshake negotiation.
        if (m_charge_session.in_session) {
            TransitionToChargeWaitState();
        } else {
            TransitionToChargeHandshakeState();
        }
        return;
    }
    if (!m_armed_for_charge) {
        if (lid_open) {
            // Charge door just opened: arm and start the 15-min cable watch
            m_armed_for_charge = true;
            m_cable_watch_start = monotonic;
            ResetSleepBackoff();   // deliberate user action — resume responsive cooldowns
        } else if (monotonic - m_awake_entered > 300) {
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
    // 12V-drain protection: if we have sat in CHARGE_WAIT without charge engaging for the
    // threshold, stop polling and sleep so the bus idles and 12V recovers. The session is
    // preserved across the sleep and resumes on passive wake (see HandleAwakeState). A wait
    // re-entered after a prior sleep uses the short threshold to keep oscillation cheap.
    int monotonic = StandardMetrics.ms_m_monotonic->AsInt();
    int sleep_after = m_charge_wait_slept ? CHARGE_WAIT_RESLEEP_SECS : CHARGE_WAIT_SLEEP_SECS;
    if (monotonic - m_charge_state_entry >= sleep_after) {
        ESP_LOGI(TAG, "CHARGE_WAIT idle %ds — sleeping to protect 12V", sleep_after);
        m_charge_wait_slept = true;
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
    // Seed the 12V edge latch from the current reading: entering sleep with 12V still
    // elevated (post-drive/post-charge surface charge) must not count as a rising edge.
    m_12v_was_high = StandardMetrics.ms_v_bat_12v_voltage->AsFloat()
                     > (StandardMetrics.ms_v_bat_12v_voltage_ref->AsFloat() + 0.2f);
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
    m_awake_entered = monotonic;
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
        m_charge_wait_slept = false;   // leaving the charge sub-machine — clear wait flag
    }
}

void OvmsVehicleToyotaETNGA::TransitionToDrivingState()
{
    m_armed_for_charge = false;
    ResetSleepBackoff();   // real drive — resume responsive cooldowns
    SetPollState(PollState::DRIVING);
    m_trip_start_valid = false;
    RequestVIN();
}

void OvmsVehicleToyotaETNGA::TransitionToChargeHandshakeState()
{
    m_armed_for_charge = false;  // arm state consumed on entering handshake
    m_charge_state_entry = StandardMetrics.ms_m_monotonic->AsInt();
    SetPollState(PollState::CHARGE_HANDSHAKE);
    SetChargingStatus(false);    // not yet delivering energy (AC/DC states set true)
    SetChargeState(PollState::CHARGE_HANDSHAKE);
    if (!m_charge_session.in_session) {
        ResetSleepBackoff();   // brand-new charge session — resume responsive cooldowns
        m_charge_wait_slept = false;   // brand-new session — fresh responsive wait window
        m_pisw_zero_count = 0;   // fresh session — clear cable-removed debounce
        m_charge_session.in_session = true;
        m_charge_session.start_monotonic = StandardMetrics.ms_m_monotonic->AsInt();
        m_charge_session.start_utc = StandardMetrics.ms_m_timeutc->AsInt();
        m_charge_session.start_soc = (int) StandardMetrics.ms_v_bat_soc->AsFloat();
        // Session energy counters reset HERE (session open), NOT in NotifyChargeStart:
        // the framework fires that hook on every ms_v_charge_state change to "charging",
        // including a CHARGE_WAIT pause/resume mid-session, which wiped the energy
        // accumulated in earlier phases (and made the report's kWh phase-local).
        StandardMetrics.ms_v_bat_energy_used->SetValue(0);
        StandardMetrics.ms_v_bat_energy_recd->SetValue(0);
        lastBatteryEnergyLogTime = 0;
        StandardMetrics.ms_v_charge_kwh->SetValue(0);
        lastChargerEnergyLogTime = 0;
        StandardMetrics.ms_v_charge_kwh_grid->SetValue(0);
        lastGridEnergyLogTime = 0;
        m_v_env_hvac_kwh->SetValue(0);
        lastHvacEnergyLogTime = 0;
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
            time_t utc = m_charge_session.start_utc;
            if (utc > 1000000000) {
                time_t st = utc; struct tm tmv; gmtime_r(&st, &tmv);
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
    m_charge_wait_slept = false;   // real charge engaged — end the wait
    SetPollState(PollState::CHARGE_AC);
    SetChargingStatus(true);
    SetChargeState(PollState::CHARGE_AC);
    StandardMetrics.ms_v_charge_mode->SetValue("standard");      // AC charging (OVMS-standard v.c.mode)
    LogChargeEvent("AC charging started");
}

void OvmsVehicleToyotaETNGA::TransitionToChargeDcState()
{
    m_charge_state_entry = StandardMetrics.ms_m_monotonic->AsInt();
    m_charge_wait_slept = false;   // real charge engaged — end the wait
    SetPollState(PollState::CHARGE_DC);
    SetChargingStatus(true);
    SetChargeState(PollState::CHARGE_DC);
    StandardMetrics.ms_v_charge_mode->SetValue("performance");   // DC fast charge -> ABRP is_dcfc (v.c.mode == "performance")
    LogChargeEvent("DC charging started");
}
