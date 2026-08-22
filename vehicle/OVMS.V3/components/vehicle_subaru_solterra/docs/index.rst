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
:Log tag: ``v-subsol`` (wrapper only). Nearly all logging comes from the shared
   e-TNGA platform under ``v-etnga`` — set that one to collect diagnostics, e.g.
   ``log level verbose v-etnga``.

-------------
Battery / BMS
-------------

The pack arrangement is owned by the **e-TNGA base**, not by the Solterra: the base declares it
for every e-TNGA vehicle and derives the actual cell and temperature-sensor counts from the
``0x182E`` / ``0x1814`` reply length at runtime.  The Solterra therefore adds no BMS code of its
own — per-cell history, deviation flags and pack statistics come from the shared platform.

The Solterra is the pack the platform defaults to and the only one validated on hardware:

* Pack: 96S CATL, Toyota EM "Type B" cells, 24 temperature sensors
* Cell arrangement: 96 voltages in 4 modules of 24; 24 temperatures, 6 per module
* Accept limits: 2.5 to 4.3 V per cell; -30 to +60 °C
* Deviation thresholds: 20 mV warn / 30 mV alert; 4 °C warn / 8 °C alert

These values live in the e-TNGA base constructor; see the
:doc:`e-TNGA index </components/vehicle_toyota_etnga/docs/index>` for the pack variants the
same code covers.

--------------------------------
Differences from e-TNGA baseline
--------------------------------

The Solterra adds no behavioural overrides on top of the e-TNGA platform — the vehicle class is
a registration wrapper.  Its whole support baseline is the platform's.

Config namespace
----------------

The Solterra inherits the ``xte`` config namespace from the e-TNGA base.  All
configuration parameters (e.g. TPMS alert thresholds) use the ``[xte]`` instance::

    config set xte tpms.pressure.warn 240

Web UI
------

All e-TNGA web UI pages are available on the Solterra; see the
:doc:`e-TNGA index </components/vehicle_toyota_etnga/docs/index>` for the
full list.  ``/bms/cellmon`` is fully populated — per-cell voltage and
temperature history, deviation flags, and pack statistics are all enabled —
because the e-TNGA base declares the BMS pack arrangement for every e-TNGA
vehicle.
