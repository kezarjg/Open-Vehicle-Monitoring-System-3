===============
Subaru Solterra
===============

The Subaru Solterra is built on the :doc:`Toyota e-TNGA platform </components/vehicle_toyota_etnga/docs/index>`.
The e-TNGA module provides the CAN polling, charging state machine, and the common support baseline shared by
all e-TNGA vehicles. This page documents only what is specific to the Solterra; see the e-TNGA module for
everything else.

----------------
Vehicle identity
----------------

:Short type code: SUBSOL
:Vehicle name: Subaru Solterra
:Log tag: ``v-subaru-solterra``

-------------
Battery / BMS
-------------

The Solterra declares its pack-specific BMS configuration, which lets the e-TNGA platform route the
per-cell voltage and temperature readings (polled from ``0x182E``) through the BMS API — enabling per-cell
history, deviation flags, and pack statistics. Without this configuration (as on the bare e-TNGA baseline)
only the raw cell-voltage metric is published.

* Pack: 96S CATL, Toyota EM "Type B" cells, 24 temperature sensors
* Cell arrangement: 96 voltages in 4 modules of 24; 24 temperatures, 6 per module
* Accept limits: 2.5 to 4.3 V per cell; -30 to +60 °C
* Deviation thresholds: 20 mV warn / 30 mV alert; 4 °C warn / 8 °C alert

--------------------------------
Differences from e-TNGA baseline
--------------------------------

=========================== ==============
Function                    Support Status
=========================== ==============
BMS v+t Display             Yes
=========================== ==============
