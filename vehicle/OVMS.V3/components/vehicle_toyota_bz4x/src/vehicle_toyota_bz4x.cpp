/*
    Project:       Open Vehicle Monitor System
    Module:        Vehicle Toyota bZ4X
    Date:          27th May 2023

   (C) 2023       Jerry Kezar <solterra@kezarnet.com>

   Licensed under the MIT License. See the LICENSE file for details.
*/

#include "ovms_log.h"
#include "vehicle_toyota_bz4x.h"

// Constructor for the OvmsVehicleToyotaBz4x class
OvmsVehicleToyotaBz4x::OvmsVehicleToyotaBz4x()
{
  ESP_LOGI(TAG, "Toyota bZ4X vehicle module");  // Log an informational message

  // Battery pack: 96S CATL (Toyota EM "Type B"), 24 temperature sensors — the
  // bZ4X is mechanically the Solterra's twin, so this mirrors that arrangement.
  // 4 stacks of 24 cells (Toyota EM "Stack 1-4 Cell Average Voltage" grouping);
  // 24 sensors map 6 per stack.
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

// Destructor for the OvmsVehicleToyotaBz4x class
OvmsVehicleToyotaBz4x::~OvmsVehicleToyotaBz4x()
{
  ESP_LOGI(TAG, "Shutdown Toyota bZ4X vehicle module");  // Log an informational message
}

// Initialization class for the Toyota bZ4X vehicle module
class OvmsVehicleToyotaBz4xInit
{
  public: 
    OvmsVehicleToyotaBz4xInit();
} OvmsVehicleToyotaBz4xInit  __attribute__ ((init_priority (9000)));

// Constructor for the OvmsVehicleToyotaBz4xInit class
OvmsVehicleToyotaBz4xInit::OvmsVehicleToyotaBz4xInit()
  {
  ESP_LOGI(OvmsVehicleToyotaBz4x::TAG, "Registering Vehicle: Toyota bZ4X (9000)");

  MyVehicleFactory.RegisterVehicle<OvmsVehicleToyotaBz4x>("TOYBZ4X","Toyota bZ4X");
  }
