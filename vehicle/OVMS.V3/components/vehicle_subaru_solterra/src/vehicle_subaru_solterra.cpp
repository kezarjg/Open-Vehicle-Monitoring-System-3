/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Subaru Solterra
   Date:          4th June 2023

   (C) 2023       Jerry Kezar <solterra@kezarnet.com>

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include "ovms_log.h"
#include "vehicle_subaru_solterra.h"

// Constructor for the OvmsVehicleSubaruSolterra class
OvmsVehicleSubaruSolterra::OvmsVehicleSubaruSolterra()
{
  ESP_LOGI(TAG, "Subaru Solterra vehicle module");  // Log an informational message

  // Battery pack: 96S CATL (Toyota EM "Type B"), 24 temperature sensors.
  // Arrangement mirrors Toyota's own "Stack 1-4 Cell Average Voltage" grouping
  // from the EM datalist: 4 stacks of 24 cells; 24 sensors map 6 per stack.
  BmsSetCellArrangementVoltage(96, 24);
  BmsSetCellArrangementTemperature(24, 6);

  // Sanity bounds for accepting incoming readings as real (Type B cells: 3.0-4.208 V).
  BmsSetCellLimitsVoltage(2.5f, 4.3f);
  BmsSetCellLimitsTemperature(-30.0f, 60.0f);

  // Per-cell deviation thresholds for warn / alert. Starting points;
  // tighten after observing real cell spread on a healthy pack.
  BmsSetCellDefaultThresholdsVoltage(0.020f, 0.030f);     // 20 mV warn / 30 mV alert
  BmsSetCellDefaultThresholdsTemperature(4.0f, 8.0f);     // 4 °C warn / 8 °C alert
}

// Destructor for the OvmsVehicleSubaruSolterra class
OvmsVehicleSubaruSolterra::~OvmsVehicleSubaruSolterra()
{
  ESP_LOGI(TAG, "Shutdown Subaru Solterra vehicle module");  // Log an informational message
}

// Initialization class for the Subaru Solterra vehicle module
class OvmsVehicleSubaruSolterraInit
{
  public: 
    OvmsVehicleSubaruSolterraInit();
} MyOvmsVehicleSubaruSolterraInit  __attribute__ ((init_priority (9000)));

// Constructor for the OvmsVehicleSubaruSolterraInit class
OvmsVehicleSubaruSolterraInit::OvmsVehicleSubaruSolterraInit()
  {
  ESP_LOGI(OvmsVehicleSubaruSolterra::TAG, "Registering Vehicle: Subaru Solterra (9000)");  // Log an informational message

  MyVehicleFactory.RegisterVehicle<OvmsVehicleSubaruSolterra>("SUBSOL","Subaru Solterra");  // Register the OvmsVehicleSubaruSolterra class with a vehicle factory
  }
