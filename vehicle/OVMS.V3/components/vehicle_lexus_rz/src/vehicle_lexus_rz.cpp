/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Lexus RZ
   Date:          7th June 2026

   (C) 2026       Jerry Kezar <solterra@kezarnet.com>

   EXPERIMENTAL / UNVALIDATED — see vehicle_lexus_rz.h. Fork-only.

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include "ovms_log.h"
#include "vehicle_lexus_rz.h"

OvmsVehicleLexusRZ::OvmsVehicleLexusRZ()
{
  ESP_LOGI(TAG, "Lexus RZ vehicle module (EXPERIMENTAL / UNVALIDATED)");

  // Best-guess shared-EV-core pack: same as the Solterra/bZ4X (96S CATL "Type B",
  // 24 temperature sensors) until validated on a real RZ. The e-TNGA base decodes
  // a fixed 96-cell 0x182E / 24-sensor 0x1814 reply, so an RZ variant with a
  // differently-sized pack (e.g. RZ 450e) would report an incorrect cell count.
  BmsSetCellArrangementVoltage(96, 24);
  BmsSetCellArrangementTemperature(24, 6);

  BmsSetCellLimitsVoltage(2.5f, 4.3f);
  BmsSetCellLimitsTemperature(-30.0f, 60.0f);

  BmsSetCellDefaultThresholdsVoltage(0.020f, 0.030f);
  BmsSetCellDefaultThresholdsTemperature(4.0f, 8.0f);
}

OvmsVehicleLexusRZ::~OvmsVehicleLexusRZ()
{
  ESP_LOGI(TAG, "Shutdown Lexus RZ vehicle module");
}

class OvmsVehicleLexusRZInit
{
  public:
    OvmsVehicleLexusRZInit();
} MyOvmsVehicleLexusRZInit __attribute__ ((init_priority (9000)));

OvmsVehicleLexusRZInit::OvmsVehicleLexusRZInit()
  {
  ESP_LOGI(OvmsVehicleLexusRZ::TAG, "Registering Vehicle: Lexus RZ (9000) [EXPERIMENTAL/UNVALIDATED]");

  MyVehicleFactory.RegisterVehicle<OvmsVehicleLexusRZ>("LEXRZ","Lexus RZ");
  }
