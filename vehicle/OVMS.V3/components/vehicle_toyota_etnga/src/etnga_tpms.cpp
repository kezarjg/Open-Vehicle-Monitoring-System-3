/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Toyota e-TNGA platform — TPMS (gateway-relayed)

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include <vector>
#include "ovms_log.h"
#include "vehicle_toyota_etnga.h"

bool OvmsVehicleToyotaETNGA::TPMSCornerMapValid()
{
    for (int s = 0; s < TPMS_SLOT_COUNT; s++)
        if (m_tpms_corner[s] >= 1 && m_tpms_corner[s] <= 4) return true;
    return false;
}

void OvmsVehicleToyotaETNGA::IncomingTPMS(uint16_t pid)
{
    switch (pid) {
        case PID_TPMS_CORNERS: {
            // 5x u8 corner enum, one per slot (0 none / 1 FL / 2 FR / 3 RL / 4 RR)
            if (m_rxbuf.size() < TPMS_SLOT_COUNT) { ESP_LOGW(TAG, "IncomingTPMS: short buffer for PID %04X", pid); break; }
            for (int s = 0; s < TPMS_SLOT_COUNT; s++)
                m_tpms_corner[s] = static_cast<int8_t>(GetRxBByte(m_rxbuf, s));
            break;
        }

        case PID_TPMS_PRESSURES: {
            // 5x u16 [status][raw]; psi_gauge = raw*0.25 - 7.35; kPa = psi * 6.894757
            if (m_rxbuf.size() < TPMS_SLOT_COUNT * 2) { ESP_LOGW(TAG, "IncomingTPMS: short buffer for PID %04X", pid); break; }
            if (!TPMSCornerMapValid()) { ESP_LOGD(TAG, "IncomingTPMS %04X: corner map not yet cached, deferring", pid); break; }
            std::vector<float> v(4, 0.0f);      // [FL,FR,RL,RR]
            for (int s = 0; s < TPMS_SLOT_COUNT; s++) {
                int corner = m_tpms_corner[s];
                if (corner < 1 || corner > 4) continue;     // unpopulated/spare slot
                uint8_t raw = GetRxBByte(m_rxbuf, 2 * s + 1);
                if (raw == 0) continue;                     // no sensor / "Initial Value"
                float psi = static_cast<float>(raw) * 0.25f - 7.35f;
                v[corner - 1] = psi * 6.894757f;
            }
            StandardMetrics.ms_v_tpms_pressure->SetValue(v);
            break;
        }

        case PID_TPMS_TEMPS: {
            // 5x u8; C = raw - 40
            if (m_rxbuf.size() < TPMS_SLOT_COUNT) { ESP_LOGW(TAG, "IncomingTPMS: short buffer for PID %04X", pid); break; }
            if (!TPMSCornerMapValid()) { ESP_LOGD(TAG, "IncomingTPMS %04X: corner map not yet cached, deferring", pid); break; }
            std::vector<float> v(4, 0.0f);      // [FL,FR,RL,RR]
            for (int s = 0; s < TPMS_SLOT_COUNT; s++) {
                int corner = m_tpms_corner[s];
                if (corner < 1 || corner > 4) continue;
                uint8_t raw = GetRxBByte(m_rxbuf, s);
                if (raw == 0) continue;                     // no sensor / "Initial Value"
                v[corner - 1] = static_cast<float>(raw) - 40.0f;
            }
            StandardMetrics.ms_v_tpms_temp->SetValue(v);
            break;
        }

        default:
            ESP_LOGW(TAG, "IncomingTPMS: unsupported PID %04X", pid);
            break;
    }
}
