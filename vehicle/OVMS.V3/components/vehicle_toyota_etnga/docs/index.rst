======================
Toyota e-TNGA platform
======================

Support for Toyota e-TNGA platform. This module is used by Toyota bZ4X and Subaru Solterra.

----------------
Support Overview
----------------

=========================== ==============
Function                    Support Status
=========================== ==============
Hardware                    Any OVMS v3 (or later) module.
Vehicle Cable               OBD-II to DB9 Data Cable for OVMS (1441200 right, or 1139300 left)
GSM Antenna                 1000500 Open Vehicles OVMS GSM Antenna (or any compatible antenna)
GPS Antenna                 1020200 Universal GPS Antenna (SMA Connector) (or any compatible antenna)
SOC Display                 Yes
Range Display               No
GPS Location                Yes
Speed Display               Yes
Temperature Display         Yes
BMS v+t Display             No
TPMS Display                No
Charge Status Display       Yes
Charge Interruption Alerts  No
Charge Control              No
Cabin Pre-heat/cool Control No
Lock/Unlock Vehicle         No
Valet Mode Control          No
Others                      VIN
=========================== ==============

PID Polling Logic
=================

State transition variables should be polled frequently (10s) for the states they contribute to
* PID_READY_SIGNAL                  { 0, 10, 10, 0}     
* PID_CHARGING_LID                  { 0, 10, 0, 10}
* PID_PISW_STATUS                   { 0, 0, 0, 10}
* PID_CHARGING                      { 0, 0, 0, 10}

Driving polls used for efficiency and power integration should be updated frequently (1s)
* PID_BATTERY_VOLTAGE_AND_CURRENT   { 0, 1, 1, 1}   - Consider using a different PID for charging power
* PID_ODOMETER                      { 0, 0, 1, 0}
* PID_VEHICLE_SPEED                 { 0, 0, 1, 0}

Some 'environmental' PIDs can be checked less frequently
* PID_AMBIENT_TEMPERATURE           { 0, 60, 60, 60}
* PID_BATTERY_TEMPERATURES          { 0, 60, 60, 60}
* PID_SHIFT_POSITION                { 0, 0, 15, 0}

Battery SOC PID should be measured frequently (15s)
* PID_BATTERY_SOC                   { 0, 15, 15, 15}

Some PIDs are on demand only
* PID_VIN
* PID_CHARGING_CONTROL_STATUS       - Determines if vehicle is charging
* PID_CHARGING_VOLTAGE_TYPE         - Determines type1, type2, or ccs