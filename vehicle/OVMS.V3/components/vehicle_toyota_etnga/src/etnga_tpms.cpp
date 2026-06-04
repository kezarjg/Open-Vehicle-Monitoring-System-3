/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Toyota e-TNGA platform — TPMS (gateway-relayed)

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include <vector>
#include "ovms_log.h"
#include "ovms_config.h"
#include "vehicle_toyota_etnga.h"

bool OvmsVehicleToyotaETNGA::TPMSCornerMapValid()
{
    for (int s = 0; s < TPMS_SLOT_COUNT; s++)
        if (m_tpms_corner[s] >= 1 && m_tpms_corner[s] <= 4) return true;
    return false;
}

void OvmsVehicleToyotaETNGA::UpdateTPMSAlert()
{
    float p_warn  = MyConfig.GetParamValueFloat("xte", "tpms.pressure.warn",  240.0f);  // kPa
    float p_alert = MyConfig.GetParamValueFloat("xte", "tpms.pressure.alert", 220.0f);  // kPa
    float t_warn  = MyConfig.GetParamValueFloat("xte", "tpms.temp.warn",       90.0f);  // C
    float t_alert = MyConfig.GetParamValueFloat("xte", "tpms.temp.alert",     100.0f);  // C

    std::vector<float> p = StandardMetrics.ms_v_tpms_pressure->AsVector();
    std::vector<float> t = StandardMetrics.ms_v_tpms_temp->AsVector();
    std::vector<short> alert(4, 0);
    for (int i = 0; i < 4; i++) {
        float pi = (i < (int)p.size()) ? p[i] : 0.0f;
        float ti = (i < (int)t.size()) ? t[i] : 0.0f;
        if (pi <= 0.0f) { alert[i] = 0; continue; }   // no reading on this wheel — no false alert
        if (pi <= p_alert || (ti > 0.0f && ti >= t_alert))      alert[i] = 2;
        else if (pi <= p_warn || (ti > 0.0f && ti >= t_warn))   alert[i] = 1;
        else                                                     alert[i] = 0;
    }
    StandardMetrics.ms_v_tpms_alert->SetValue(alert);
}

void OvmsVehicleToyotaETNGA::IncomingTPMS(uint16_t pid)
{
    // Diagnostic: confirms the gateway EXTADR poll actually answered, and how big the payload is.
    ESP_LOGD(TAG, "IncomingTPMS: PID %04X answered, %u payload bytes", pid, (unsigned)m_rxbuf.size());

    switch (pid) {
        case PID_TPMS_CORNERS: {
            // 5x u8 corner enum, one per slot (0 none / 1 FL / 2 FR / 3 RL / 4 RR)
            if (m_rxbuf.size() < TPMS_SLOT_COUNT) { ESP_LOGW(TAG, "IncomingTPMS: short buffer for PID %04X", pid); break; }
            for (int s = 0; s < TPMS_SLOT_COUNT; s++)
                m_tpms_corner[s] = static_cast<int8_t>(GetRxBByte(m_rxbuf, s));
            ESP_LOGD(TAG, "TPMS corner map (slot 0-4): %d %d %d %d %d",
                     m_tpms_corner[0], m_tpms_corner[1], m_tpms_corner[2],
                     m_tpms_corner[3], m_tpms_corner[4]);
            break;
        }

        case PID_TPMS_PRESSURES: {
            // 5x u16 [status][raw]; psi_gauge = raw*0.25 - 7.35; kPa = psi * 6.894757
            if (m_rxbuf.size() < TPMS_SLOT_COUNT * 2) { ESP_LOGW(TAG, "IncomingTPMS: short buffer for PID %04X", pid); break; }
            // Raw low-bytes logged before the skip-on-zero logic: all-zero here = sensors not
            // reporting (motion-activated, stale on short drives); non-zero here with zero metrics
            // would instead point at a decode/remap mismatch.
            ESP_LOGD(TAG, "TPMS pressure raw low-bytes (slot 0-4): %02X %02X %02X %02X %02X",
                     GetRxBByte(m_rxbuf, 1), GetRxBByte(m_rxbuf, 3), GetRxBByte(m_rxbuf, 5),
                     GetRxBByte(m_rxbuf, 7), GetRxBByte(m_rxbuf, 9));
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
            ESP_LOGD(TAG, "TPMS pressure kPa [FL,FR,RL,RR]: %.1f %.1f %.1f %.1f", v[0], v[1], v[2], v[3]);
            StandardMetrics.ms_v_tpms_pressure->SetValue(v);
            UpdateTPMSAlert();
            break;
        }

        case PID_TPMS_TEMPS: {
            // 5x u8; C = raw - 40
            if (m_rxbuf.size() < TPMS_SLOT_COUNT) { ESP_LOGW(TAG, "IncomingTPMS: short buffer for PID %04X", pid); break; }
            ESP_LOGD(TAG, "TPMS temp raw bytes (slot 0-4): %02X %02X %02X %02X %02X",
                     GetRxBByte(m_rxbuf, 0), GetRxBByte(m_rxbuf, 1), GetRxBByte(m_rxbuf, 2),
                     GetRxBByte(m_rxbuf, 3), GetRxBByte(m_rxbuf, 4));
            if (!TPMSCornerMapValid()) { ESP_LOGD(TAG, "IncomingTPMS %04X: corner map not yet cached, deferring", pid); break; }
            std::vector<float> v(4, 0.0f);      // [FL,FR,RL,RR]
            for (int s = 0; s < TPMS_SLOT_COUNT; s++) {
                int corner = m_tpms_corner[s];
                if (corner < 1 || corner > 4) continue;
                uint8_t raw = GetRxBByte(m_rxbuf, s);
                if (raw == 0) continue;                     // no sensor / "Initial Value"
                v[corner - 1] = static_cast<float>(raw) - 40.0f;
            }
            ESP_LOGD(TAG, "TPMS temp C [FL,FR,RL,RR]: %.0f %.0f %.0f %.0f", v[0], v[1], v[2], v[3]);
            StandardMetrics.ms_v_tpms_temp->SetValue(v);
            UpdateTPMSAlert();
            break;
        }

        default:
            ESP_LOGW(TAG, "IncomingTPMS: unsupported PID %04X", pid);
            break;
    }
}
