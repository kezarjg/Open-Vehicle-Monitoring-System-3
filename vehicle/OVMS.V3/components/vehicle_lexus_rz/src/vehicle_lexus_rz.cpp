/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Lexus RZ
   Date:          7th June 2026

   (C) 2026       Jerry Kezar <solterra@kezarnet.com>

   EXPERIMENTAL / UNVALIDATED -- see vehicle_lexus_rz.h. Fork-only.

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include "ovms_log.h"
#include "vehicle_lexus_rz.h"

OvmsVehicleLexusRZ::OvmsVehicleLexusRZ()
{
  ESP_LOGI(TAG, "Lexus RZ vehicle module (EXPERIMENTAL / UNVALIDATED)");
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
