/*
   Project:   Open Vehicle Monitor System
   Module:    Vehicle Toyota e-TNGA platform — charge-fault diagnostic DID dump
   Date:      23rd June 2026

   (C) 2026   Jerry Kezar <solterra@kezarnet.com>
   Licensed under the MIT License.

   On an abnormal 0x1688 charge-stop, fire a one-shot OnceOffPoll burst over the OBC's
   diagnostic DIDs (all on 0x745/0x74D) and capture each raw response for the charge report.
   Raw hex only — decode happens offline against the solterra-can RE repo.
*/

#include "vehicle_toyota_etnga.h"

using namespace std;

// All diagnostic DIDs live on the Plug-In Control System / OBC (tx 0x745 / rx 0x74D),
// confirmed by a full gallia scan. Raw responses are captured; no decode here.
static const uint16_t etnga_dump_dids[] = {
    // state-machine
    0x1666, 0x1684, 0x1688, 0x1664, 0x1667, 0x1668, 0x1736,
    // trip-flag
    0x16AA, 0x16A9, 0x161B, 0x1806, 0x1702,
    // connector + safety
    0x1669, 0x1602, 0x1601, 0x1625, 0x1670, 0x164A,
    // electrical at fault
    0x10D4, 0x1654, 0x166C, 0x1621, 0x166B, 0x165E,
    // thermal
    0x1632, 0x1705, 0x1829, 0x182A, 0x1657, 0x1658,
};
static const int ETNGA_DUMP_DID_COUNT = sizeof(etnga_dump_dids) / sizeof(etnga_dump_dids[0]);

// True only for genuine abnormal stops (per the user-confirmed set) — NOT benign stops.
bool OvmsVehicleToyotaETNGA::IsChargeFaultCode(int code)
{
    switch (code & 0xFF) {
        case 0x23: case 0x24: case 0x25: case 0x29:   // AC: Abnormal / Battery / High-Power / System
        case 0x32: case 0x33: case 0x39: case 0x3A:   // DC: Abnormal / Battery / System / Vehicle-System
            return true;
        default:
            return false;
    }
}

// Poller-task callbacks: store the raw response (or an empty marker on failure) keyed by DID,
// and decrement the outstanding counter. No phases[] access here (Events task owns that).
void OvmsVehicleToyotaETNGA::IncomingDumpSuccess(uint16_t type, uint32_t module_sent, uint32_t module_rec,
        uint16_t pid, CAN_frame_format_t format, const std::string& data)
{
    {
        OvmsMutexLock lock(&m_dump_mutex);
        m_dump_results[pid] = data;
    }
    if (--m_dump_remaining <= 0)
        ESP_LOGI(TAG, "Charge-fault DID dump complete (%d DIDs captured)", (int) m_dump_results.size());
}

void OvmsVehicleToyotaETNGA::IncomingDumpFail(uint16_t type, uint32_t module_sent, uint32_t module_rec,
        uint16_t pid, int errorcode)
{
    {
        OvmsMutexLock lock(&m_dump_mutex);
        m_dump_results[pid] = "";   // "" => rendered as "(no reply)"
    }
    ESP_LOGD(TAG, "Charge-fault dump: DID %04X no reply (err %d)", pid, errorcode);
    if (--m_dump_remaining <= 0)
        ESP_LOGI(TAG, "Charge-fault DID dump complete (%d DIDs)", (int) m_dump_results.size());
}

// Events task (called from TransitionToChargeWaitState after CloseChargePhase). If a fault was
// flagged and no dump is already running, fire a OnceOffPoll for every diagnostic DID. Each
// auto-removes after one reply; "!v." prefix => reclaimed on teardown (poller naming contract).
void OvmsVehicleToyotaETNGA::MaybeStartChargeFaultDump()
{
    if (!m_charge_fault_pending.exchange(false))
        return;
    if (m_dump_remaining.load() > 0)
        return;   // a dump is already in flight; don't overlap

    using std::placeholders::_1; using std::placeholders::_2; using std::placeholders::_3;
    using std::placeholders::_4; using std::placeholders::_5; using std::placeholders::_6;

    {
        OvmsMutexLock lock(&m_dump_mutex);
        m_dump_results.clear();
    }
    m_dump_outcome   = m_dump_trigger_outcome;                       // the triggering fault code, not current
    m_dump_phase_idx = (m_dump_trigger_phase >= 0) ? m_dump_trigger_phase
                                                   : (int) m_charge_session.phases.size() - 1;
    m_dump_remaining = ETNGA_DUMP_DID_COUNT;

    char name[24];
    for (int i = 0; i < ETNGA_DUMP_DID_COUNT; i++) {
        uint16_t did = etnga_dump_dids[i];
        auto entry = std::shared_ptr<OvmsPoller::OnceOffPoll>(
            new OvmsPoller::OnceOffPoll(
                std::bind(&OvmsVehicleToyotaETNGA::IncomingDumpSuccess, this, _1, _2, _3, _4, _5, _6),
                std::bind(&OvmsVehicleToyotaETNGA::IncomingDumpFail,    this, _1, _2, _3, _4, _5),
                PLUG_IN_CONTROL_SYSTEM_TX, PLUG_IN_CONTROL_SYSTEM_RX,
                VEHICLE_POLL_TYPE_READDATA, did,
                ISOTP_STD, 0, /*retry_fail=*/1));
        snprintf(name, sizeof(name), "!v.xte.dmp.%04X", did);   // "!v." => teardown-reclaimed
        PollRequest(m_can2, name, entry);
    }
    ESP_LOGW(TAG, "Charge fault (0x%02X) — diagnostic dump of %d OBC DIDs started",
             m_dump_outcome & 0xFF, ETNGA_DUMP_DID_COUNT);
}
