/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Toyota e-TNGA platform — TPMS (gateway-relayed)

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include "ovms_log.h"
#include "vehicle_toyota_etnga.h"
#include <vector>

void OvmsVehicleToyotaETNGA::IncomingTPMS(uint16_t pid)
{
    switch (pid) {
        case PID_TPMS_CORNERS: {
            // 5x u8 corner enum, one per slot (0 none / 1 FL / 2 FR / 3 RL / 4 RR)
            for (int s = 0; s < 5; s++)
                m_tpms_corner[s] = static_cast<int8_t>(GetRxBByte(m_rxbuf, s));
            break;
        }

        default:
            break;
    }
}
