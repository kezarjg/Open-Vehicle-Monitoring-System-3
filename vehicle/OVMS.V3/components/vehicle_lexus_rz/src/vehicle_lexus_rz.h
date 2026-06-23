/*
   Project:       Open Vehicle Monitor System
   Module:        Vehicle Lexus RZ
   Date:          7th June 2026

   (C) 2026       Jerry Kezar <solterra@kezarnet.com>

   EXPERIMENTAL / UNVALIDATED: this wrapper inherits the Toyota e-TNGA logic
   wholesale and adds nothing. It has NOT been validated against a real Lexus RZ.
   Fork-only -- do not upstream until validated on hardware.

   Licensed under the MIT License. See the LICENSE file for details.
*/

#ifndef __VEHICLE_LEXUS_RZ_H__
#define __VEHICLE_LEXUS_RZ_H__

#include "../../vehicle_toyota_etnga/src/vehicle_toyota_etnga.h"  // Toyota e-TNGA base

class OvmsVehicleLexusRZ : public OvmsVehicleToyotaETNGA
{
public:
  OvmsVehicleLexusRZ();
  ~OvmsVehicleLexusRZ();
  static constexpr const char* TAG = "v-lexus-rz";

};

#endif // __VEHICLE_LEXUS_RZ_H__
