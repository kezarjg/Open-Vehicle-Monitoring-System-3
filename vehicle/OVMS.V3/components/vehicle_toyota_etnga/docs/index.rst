======================
Toyota e-TNGA platform
======================

Support for the Toyota e-TNGA platform. This is a shared base module, not a directly selectable vehicle:
it provides the CAN polling, charging state machine, and the common support baseline for the vehicles built
on it:

* :doc:`Toyota bZ4X </components/vehicle_toyota_bz4x/docs/index>`
* :doc:`Subaru Solterra </components/vehicle_subaru_solterra/docs/index>`

Those pages list only what is specific to each vehicle; the Support Overview below is the shared baseline
they inherit.

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
BMS v+t Display             Yes (see note below)
TPMS Display                Yes
Charge Status Display       Yes
Charge Interruption Alerts  Yes (see note below)
Charge Control              No
Cabin Pre-heat/cool Control No
Lock/Unlock Vehicle         No
Valet Mode Control          No
Others                      VIN
=========================== ==============

.. note::

   **Charge Interruption Alerts** are produced by the OVMS framework, not by this module:
   e-TNGA writes ``ms_v_charge_state = "stopped"`` whenever a charge phase ends, and the
   ``OvmsVehicle`` base turns that into a ``charge.stopped`` notification.  The module does
   not set ``ms_v_charge_substate``, so the notification is raised at **alert** priority
   rather than **info** — including at an ordinary phase boundary, such as a scheduled
   charge pausing and resuming, which is not a fault.

.. note::

   **BMS v+t Display** is ``Yes`` for all e-TNGA vehicles: the base class declares the HV pack
   arrangement (Toyota EM "Type B" chemistry, shared across the platform) and derives the actual
   cell and temperature-sensor counts from the ``0x182E`` / ``0x1814`` reply length at runtime, so
   per-cell history, deviation flags, and pack statistics work across pack variants (96-cell
   2022-24; 78-cell and 104-cell 2025/26 refresh). Only the 96-cell pack is on-vehicle validated.

Validation status
=================

e-TNGA is developed against a single available vehicle — a 96-cell Subaru Solterra.  Much of
the decode logic is therefore derived from CAN reverse engineering rather than confirmed
against an independent reference, and several pack and charging variants have no hardware
behind them at all.  This table records which of the behaviours described elsewhere in these
docs have actually been observed on a car, so that a measured behaviour can be told apart
from an inferred one.  The vehicle pages built on this base inherit these statuses.

Vehicle-validated
   Observed on a real vehicle, with the date of the session that confirmed it.

Vehicle-validated (partial)
   Confirmed on some code paths but not all; the unexercised path is named in the
   Evidence / gap column.

Log-inferred
   Decoded from captured CAN traffic or module logs and self-consistent, but never
   cross-checked against an independent ground truth — a scan tool, a published
   specification, or a physically known value.

Unvalidated
   Reasoned from a specification, an analogous DID, or another platform.  Never
   exercised on hardware.

Last reviewed: 2026-08-22.

State machine and sleep
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 32 22 46

   * - Behaviour
     - Status
     - Evidence / gap
   * - ``SLEEP`` / ``AWAKE`` / ``DRIVING`` transitions
     - Vehicle-validated
     - Continuous daily-driver use since 2026-06, including the ``AWAKE → DRIVING``
       edge driven by ``0x10D1`` on the Plug-In Control ECU.
   * - ``v.e.awake`` decoupled from CAN2 bus-liveness
     - Vehicle-validated
     - A 14.6 kWh charge on 2026-07-13 produced zero spurious "Vehicle is idling"
       alerts.  The true-positive path is confirmed as well: over the following
       week exactly two alerts fired (2026-07-16 18:43 and 2026-07-18 14:00), both
       with the vehicle stationary, parked, **not** charging and drawing
       1.0–3.1 kW — the genuine idling condition — against hourly spam before
       the fix.
   * - Adaptive parked-sleep cooldown backoff
     - Unvalidated
     - Merged, but the escalating cooldown has never been confirmed on a vehicle.

Charging
--------

.. list-table::
   :header-rows: 1
   :widths: 32 22 46

   * - Behaviour
     - Status
     - Evidence / gap
   * - AC path (``CHARGE_HANDSHAKE`` → ``CHARGE_WAIT`` → ``CHARGE_AC``)
     - Vehicle-validated
     - Many sessions.  Multi-phase pause/resume within one plug-in confirmed
       2026-06-24 (report ``20260624T002537Z``, ``Phases: 2``, energy reconciling
       across phases).
   * - DC path (``CHARGE_DC``)
     - Vehicle-validated
     - Repeated DC fast charges across the 2026-07-16→19 road trip, including
       sessions of 50.58 kWh (6%→84%) and 28.01 kWh (35%→76%).
   * - Charge port ``v.d.cp`` latched at handshake
     - Vehicle-validated (partial)
     - Confirmed for plug-in while already ``AWAKE``.  The plug-in that *wakes the
       module from* ``SLEEP`` — the case the fix was written for, where PISW
       ``0x1669`` at 5 s beats the lid signal ``0x1625`` at 10 s — is unvalidated.
   * - ``v.c.type`` AC (``type1`` / ``type2``)
     - Vehicle-validated
     - Read correctly across AC sessions.
   * - ``v.c.type`` DC (``ccs``)
     - Vehicle-validated
     - 27 ``*-LOG-Grid`` notification records across the 2026-07-16→19 road trip
       carry ``ccs`` in the charge-type field, paired with ``charging`` /
       ``stopped`` states at distinct highway locations.
   * - Charge power derived from pack V×I
     - Vehicle-validated
     - Energy reconciliation on 2026-06-24: 88% efficiency, station 0.17 kWh ≈
       battery 0.15 kWh, replacing earlier 52%/158% garbage.
   * - Charge-fault diagnostic DID dump
     - Vehicle-validated (partial)
     - A real ``0x29`` fault on 2026-06-24 fired the dump, labelled the triggering
       outcome correctly, and rendered decoded values.  The trigger also fired on
       *healthy* scheduled AC charges, because it read the retained ``0x1688``
       value rather than a live fault; that false positive was fixed on
       2026-08-22 and the fix awaits one scheduled charge to confirm.
   * - CAN-stale logging/accounting suspend
     - Vehicle-validated
     - A real lock produced an 85 s CSV gap with no stale rows, 2026-06-24.
   * - DC limiting-side attribution (car vs station)
     - Vehicle-validated
     - 15 classifications on real DC sessions over 2026-07-16→19, exercising
       **both** branches: 14 × "limited by car" (46.3–78.6 kW, consistent with
       battery taper) and 1 × "limited by station (100.0 kW)".  The thresholds
       (2 kW margin, 25 °C cold-battery) remain untuned against ground truth —
       there is no station nameplate data to check the 100.0 kW figure against.
   * - Charge-power DID scale factors
     - Log-inferred
     - Units inferred by analogy to the ``0x161D`` grid-power DID; already flagged
       ``(unit inferred)`` in :doc:`state_machine`.
   * - HLC handshake (``0x1666``) and AC-Op (``0x1684``) enum labels
     - Log-inferred
     - Labels come from CAN reverse engineering.  They render correctly in the
       2026-06-24 fault dump, but the enum *semantics* have not been cross-checked
       against a scan tool.
   * - Ambient temperature during charge (``0x1F46``, HCS ECU)
     - Log-inferred
     - Selected because the A/C ECU ``0x7C4`` sleeps while charging.  Returns a
       plausible value; never compared against a known ambient reading.

Battery and 12V
---------------

.. list-table::
   :header-rows: 1
   :widths: 32 22 46

   * - Behaviour
     - Status
     - Evidence / gap
   * - 96-cell pack arrangement and per-cell decode
     - Vehicle-validated
     - ``bms status`` on 2026-06-23 reported exactly 96 cells in 4 modules of 24,
       24 temperature sensors at 6 per module, 0 warnings and 0 alerts.
   * - 78-cell and 104-cell pack variants
     - Unvalidated
     - No such hardware available; both are reasoned from published pack specs.
       The runtime auto-arrange is deliberately **grow-only** — it cannot shrink,
       because a short reply is indistinguishable from a truncated one.
   * - Per-cell voltage polling during ``CHARGE_AC``
     - Unvalidated
     - ``0x182E`` now polls at 60 s in ``CHARGE_AC`` (previously not polled in that
       state at all, while the temperature array was).  Needs one AC session to
       confirm the cell data updates and that the added array poll does not disturb
       the charge.  It also closes a latent variant bug: ``m_bms_modules`` is
       resolved from this reply and drives temperature grouping, so a module
       booting straight into an AC charge previously kept its constructor default
       for the whole session — harmless on the 96-cell pack, wrong on 78/104.
   * - ``v.b.cac`` / ``v.b.soh`` / ``v.b.capacity`` from ``0x1D3E``
       (HV Battery ECU ``0x747``/``0x74F``)
     - Vehicle-validated
     - Five weeks of logs showed ``0x1D3E`` declining monotonically; it is now
       summed into ``v.b.cac``, with ``v.b.soh`` and ``v.b.capacity`` derived
       against the pack nominal.  Confirmed on-module 2026-08-22: the live
       metrics reproduce exactly from the eight raw per-module slots
       (197.656 Ah / 98.2875 % / 62.96 kWh).  Only the 96-cell pack has an
       established nominal; on any other pack ``v.b.soh`` and ``v.b.capacity``
       stay empty until ``[xte] bat.nominal.ah`` is set by hand.
   * - Capacity array ``0x1D3F`` (HV Battery ECU ``0x747``/``0x74F``)
     - Log-inferred
     - Re-latches each cycle with no trend; semantics unconfirmed, so it is
       collected (``xte.v.b.cap.alt``) but not exposed as a standard metric.
   * - Lifetime counters ``0x1D70`` (HV Battery ECU ``0x747``/``0x74F``)
     - Unvalidated
     - Collected raw into ``xte.v.b.lifetime.acc`` / ``.min`` and the charge CSV;
       the accumulator's scale is unresolved.  It is frozen across a whole AC
       charge, so resolving it needs a **drive**, not a charge session.
   * - ``v.e.charging12v`` union rule
     - Vehicle-validated
     - The 2026-07-16→20 road trip closed the last two gaps: the ``CHARGE_DC``
       term (16 of 16 fast charges) and the 12V rising-edge wake trigger (4 clean
       fires).
   * - 12V current from ``0x15F7`` (EV-ECU ``0x7D2``)
     - Vehicle-validated (partial)
     - Direct read confirmed on-module 2026-06-21 after the dead ``0x15FD`` was
       replaced.  The under-load swing is still unexercised.

Driver inputs and TPMS
----------------------

.. list-table::
   :header-rows: 1
   :widths: 32 22 46

   * - Behaviour
     - Status
     - Evidence / gap
   * - TPMS pressures and temperatures
     - Vehicle-validated
     - Two drive sessions on 2026-06-04 polled cleanly via the gateway relay
       (target ``0x750``, sub-target ``0x2A``): ~280→310 kPa, 34–38 °C, no
       timeouts.  Zeros only before the sensors wake at drive start.
   * - Throttle, foot brake and park brake (Brake/EPB ECU ``0x7B0``)
     - Vehicle-validated
     - Throttle logged 90,752 change events spanning 0–100%, foot brake 11,239
       events spanning 0–100% (mean 22%), park brake 163 Applied / 151 Released.
       The foot brake does reach full scale, so the suspected scaling problem
       appears unfounded.
   * - Drive mode and AWD mode (Brake/EPB ECU ``0x7B0``)
     - Unvalidated
     - Neither metric has produced a single change event in any captured log,
       while sibling metrics decoded from the same ECU logged tens of thousands.
       This is consistent with a driver who simply never changes mode, but the
       decode is unexercised either way.  Cheapest possible check: change drive
       mode once and watch for the log line.

PID Polling Logic
=================

The table below shows seven columns, one per ``PollState`` value.  The number in each column is
the poll interval in seconds (``0`` = not polled in that state).  All rows use ``ISOTP_STD``
addressing unless noted.

The seven columns are a **merged view**.  A poll list supports only ``VEHICLE_POLL_NSTATES`` (4)
states, so the implementation splits the seven states across two poll series in
``vehicle_toyota_etnga.cpp``:

* ``obdii_polls_base`` at state offset 0, whose four columns map to
  ``SLEEP`` / ``AWAKE`` / ``DRIVING`` / (unused);
* ``obdii_polls_charge`` registered at offset ``CHARGE_HANDSHAKE`` (3), whose four columns map to
  ``CHARGE_HANDSHAKE`` / ``CHARGE_WAIT`` / ``CHARGE_AC`` / ``CHARGE_DC``.

A PID polled in both a non-charge and a charge state therefore appears in **both** arrays, each
carrying only its own side's cadences; the row below is the union of the two.  The fourth column
of ``obdii_polls_base`` must stay ``0`` in every row — it aliases ``CHARGE_HANDSHAKE``, which
block B owns, and a nonzero value there would double-poll during handshake.

Within ``obdii_polls_charge`` the row order is also load-bearing: the poller services the list
top-down each cycle and may not finish before the next tick, so the per-second power and SOC
channels are listed first and the heavy multiframe arrays last, which makes a cut-short cycle
defer the slow arrays rather than the live readings.

.. list-table::
   :header-rows: 1
   :widths: 22 10 8 8 8 8 8 8 10 20

   * - PID (DID)
     - ECU
     - SLEEP
     - AWAKE
     - DRIVING
     - HS
     - WAIT
     - AC
     - DC
     - Purpose
   * - ``PID_CONTROL_SYSTEM_MODE`` (``0x10D1``)
     - Plug-In Control
     - 0
     - 1
     - 1
     - 1
     - 10
     - 1
     - 1
     - Control mode (CS_NONE / CS_DRIVING / CS_CHARGING); @1 s except WAIT@10 s
       (slowed to protect the 12 V battery during a scheduled wait)
   * - ``PID_CHARGING_LID`` (``0x1625``)
     - Plug-In Control
     - 0
     - 10
     - 10
     - 0
     - 0
     - 0
     - 0
     - Charge lid open/closed; AWAKE + DRIVING only
   * - ``PID_BATTERY_SOC`` (``0x1738``)
     - Plug-In Control
     - 0
     - 0
     - 1
     - 0
     - 0
     - 1
     - 1
     - SOC; DRIVING + AC + DC @1 s
   * - ``PID_BATTERY_SOC_BMS`` (``0x1F5B``)
     - HV Battery
     - 0
     - 0
     - 1
     - 0
     - 0
     - 1
     - 1
     - BMS SOC; mirrors ``PID_BATTERY_SOC`` pattern
   * - ``PID_READY_SIGNAL`` (``0x1076``)
     - Hybrid Control
     - 0
     - 0
     - 1
     - 0
     - 0
     - 0
     - 0
     - Ready / READY lamp; DRIVING only
   * - ``PID_BATTERY_VOLTAGE_AND_CURRENT`` (``0x1F9A``)
     - Hybrid Control
     - 0
     - 0
     - 1
     - 0
     - 0
     - 1
     - 1
     - Pack voltage + current; DRIVING + AC + DC @1 s
   * - ``PID_BATTERY_TEMPERATURES`` (``0x1814``)
     - HV Battery
     - 0
     - 0
     - 10
     - 0
     - 0
     - 30
     - 20
     - Cell-temperature array; DRIVING@10 s, AC@30 s, DC@20 s
   * - ``PID_BATTERY_CELL_VOLTAGES`` (``0x182E``)
     - HV Battery
     - 0
     - 0
     - 5
     - 0
     - 0
     - 60
     - 30
     - Per-cell voltages; DRIVING@5 s, AC@60 s, DC@30 s
   * - ``PID_BATTERY_CAPACITY`` (``0x1D3E``)
     - HV Battery
     - 0
     - 60
     - 120
     - 0
     - 0
     - 60
     - 60
     - 8× per-module full-charge capacity (Ah) → ``v.b.cac``, and ``v.b.soh`` /
       ``v.b.capacity`` derived from it against the pack nominal
   * - ``PID_BATTERY_CAPACITY_ALT`` (``0x1D3F``)
     - HV Battery
     - 0
     - 60
     - 120
     - 0
     - 0
     - 60
     - 60
     - 8× parallel capacity array; function unconfirmed; data collection
   * - ``PID_BATTERY_LIFETIME`` (``0x1D70``)
     - HV Battery
     - 0
     - 0
     - 60
     - 0
     - 0
     - 30
     - 30
     - Lifetime counters (244 B, ~36 frames) → ``xte.v.b.lifetime.acc`` / ``.min``;
       units unresolved, logged raw for analysis (``xte.v.b.lifetime.min`` is the
       second counter in the same reply)
   * - ``PID_VEHICLE_SPEED`` (``0x1F0D``)
     - Hybrid Control
     - 0
     - 0
     - 1
     - 0
     - 0
     - 0
     - 0
     - Vehicle speed; DRIVING only
   * - ``PID_SHIFT_POSITION`` (``0x1061``)
     - Hybrid Control
     - 0
     - 0
     - 1
     - 0
     - 0
     - 0
     - 0
     - Gear selector position; DRIVING only
   * - ``PID_ODOMETER`` (``0x1FA6``)
     - Hybrid Control
     - 0
     - 0
     - 1
     - 0
     - 0
     - 0
     - 0
     - Odometer; DRIVING only
   * - ``PID_AC_CONSUMPTION`` / driving (``0x106E``)
     - Hybrid Control
     - 0
     - 0
     - 1
     - 0
     - 0
     - 0
     - 0
     - HVAC power while driving @1 s; feeds cabin-energy integrator
   * - ``PID_THROTTLE`` (``0x1060``)
     - Hybrid Control
     - 0
     - 0
     - 1
     - 0
     - 0
     - 0
     - 0
     - Accelerator position (byte 1) → ``v.e.throttle``; DRIVING only
   * - ``PID_DRIVE_MODE_SELECT`` (``0x1004``)
     - Hybrid Control
     - 0
     - 0
     - 5
     - 0
     - 0
     - 0
     - 0
     - Drive mode Eco/Normal/Power (byte 1) → ``v.e.drivemode``; DRIVING only.
       Note this DID number is ``0x1004`` **on the Hybrid Control ECU**; the same
       number on the TPMS gateway is tyre temperatures
   * - ``PID_AWD_MODE`` (``0x1087``)
     - Hybrid Control
     - 0
     - 0
     - 5
     - 0
     - 0
     - 0
     - 0
     - AWD / X-MODE status (byte 2) → ``xte.v.e.awd``; DRIVING only
   * - ``PID_BRAKE_PEDAL_STROKE`` (``0x104C``)
     - Brake/EPB (``0x7B0``)
     - 0
     - 0
     - 1
     - 0
     - 0
     - 0
     - 0
     - Brake pedal stroke (byte 1) → ``v.e.footbrake``; DRIVING only
   * - ``PID_EPB_STATUS`` (``0x1045``)
     - Brake/EPB (``0x7B0``)
     - 0
     - 5
     - 5
     - 0
     - 0
     - 0
     - 0
     - Electric parking brake actuator status (byte 1) → ``v.e.handbrake``.
       Polled in AWAKE too: the EPB stays alive in the parked body tail
   * - ``PID_AMBIENT_TEMPERATURE`` (``0x1002``)
     - Air Conditioner
     - 0
     - 0
     - 10
     - 0
     - 0
     - 0
     - 0
     - Ambient temperature; DRIVING only
   * - ``PID_CABIN_TEMPERATURE`` (``0x1001``)
     - Air Conditioner
     - 0
     - 0
     - 10
     - 0
     - 0
     - 0
     - 0
     - Cabin temperature; DRIVING only
   * - ``PID_HVAC_SETPOINT`` (``0x1036``)
     - Air Conditioner
     - 0
     - 0
     - 10
     - 0
     - 0
     - 0
     - 0
     - HVAC setpoint; DRIVING only
   * - ``PID_HEATER_POWER`` (``0x1086``)
     - Air Conditioner
     - 0
     - 0
     - 10
     - 0
     - 0
     - 0
     - 0
     - HV heater power; DRIVING only (>0 → ``v.e.heating``)
   * - ``PID_BLOWER_LEVEL`` (``0x2801``)
     - Air Conditioner
     - 0
     - 0
     - 10
     - 0
     - 0
     - 0
     - 0
     - Blower level 1–7; DRIVING only (→ ``v.e.cabinfan`` %)
   * - ``PID_AMBIENT_TEMPERATURE_EV`` (``0x1F46``)
     - Hybrid Control
     - 0
     - 0
     - 0
     - 30
     - 30
     - 30
     - 30
     - Ambient temperature during charging; all four charge states @30 s.  The
       A/C ECU (``0x7C4``) sleeps while charging, so its ``0x1002`` ambient is
       DRIVING-only and this is the in-charge source (``ambient_c`` in the CSV)
   * - ``PID_PISW_STATUS`` (``0x1669``)
     - Plug-In Control
     - 0
     - 5
     - 0
     - 1
     - 30
     - 10
     - 10
     - PISW cable-seated signal; AWAKE@5 s (wake-reconcile); HS/AC/DC dense
   * - ``PID_CHARGING_VOLTAGE_TYPE`` (``0x161C``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 5
     - 0
     - 0
     - 0
     - Charger power-supply voltage type (Type 1 / Type 2); HANDSHAKE@5 s only
   * - ``PID_AC_CHARGING_OP_STATUS`` (``0x1684``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 1
     - 10
     - 1
     - 1
     - AC charger operation status; drives HANDSHAKE→AC transition.  WAIT@10 s
       (slowed from 1 s for 12 V; an AC engage is still caught within 10 s)
   * - ``PID_HLC_STATE`` (``0x1666``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 1
     - 10
     - 0
     - 1
     - DC HLC state; drives HANDSHAKE→DC / DC→WAIT transitions.  WAIT@10 s
       (slowed from 1 s for 12 V; a DC re-engage is still caught within 10 s)
   * - ``PID_MIN_PERMISSION_POWER`` (``0x16A1``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 1
     - 1
     - Minimum charging permission power (taper floor); AC + DC @1 s
   * - ``PID_TARGET_CHARGING_CURRENT`` (``0x166D``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 1
     - 1
     - Target charging current set by vehicle; AC + DC @1 s
   * - ``PID_BATTERY_CHARGING_POWER`` (``0x10D4``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 1
     - 1
     - Battery-delivered charge power (valid AC + DC); AC + DC @1 s
   * - ``PID_CHARGER_INPUT_POWER`` (``0x161D``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 5
     - 5
     - Grid input power (AC only; 0 during DC); AC + DC @5 s
   * - ``PID_DC_CHARGER_PRESENT_CURRENT`` (``0x166C``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 0
     - 1
     - DC station present current → ``v.c.current``; DC @1 s
   * - ``PID_DC_CHARGER_PRESENT_VOLTAGE`` (``0x166B``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 0
     - 1
     - DC station present voltage → ``v.c.voltage``; DC @1 s
   * - ``PID_DC_CHARGER_MAX_POWER`` (``0x166A``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 0
     - 5
     - DC station max power (negotiated); DC @5 s
   * - ``PID_DC_CHARGER_MAX_CURRENT`` (``0x1679``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 0
     - 5
     - DC station max current (CCS contract); DC @5 s
   * - ``PID_DC_CHARGER_MAX_VOLTAGE`` (``0x1681``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 0
     - 5
     - DC station max voltage (CCS contract); DC @5 s
   * - ``PID_CHARGER_STATE_CLUSTER`` (``0x1619``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 1
     - 0
     - AC charger state cluster (target power, op status, current limit); AC @1 s
   * - ``PID_CHARGER_OUTPUT_POWER`` (``0x161E``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 5
     - 0
     - AC charger output cluster (output + target power raw); AC @5 s
   * - ``PID_AC_USABLE_POWER`` (``0x1665``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 5
     - 0
     - A/C useable power raw; AC @5 s
   * - ``PID_MYROOM`` (``0x1692``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 5
     - 5
     - My Room active flag; AC + DC @5 s
   * - ``PID_AC_CONSUMPTION`` / charging (``0x106E``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 0
     - 0
     - 2
     - 2
     - HVAC/cabin power while charging (OBC); AC + DC @2 s
   * - ``PID_CHARGE_HISTORY`` (``0x1688``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 5
     - 10
     - 5
     - 5
     - Charging History outcome enum (``xte.v.c.outcome``); HS/WAIT/AC/DC
   * - ``PID_CHARGE_STOP_REQ`` (``0x1667``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 1
     - 30
     - 5
     - 5
     - Charge stop request from CCM; HS dense, WAIT/AC/DC sparse
   * - ``PID_TPMS_CORNERS`` (``0x2021``) *ISOTP_EXTADR*
     - TPMS GW (``0x750``/``0x2A``)
     - 0
     - 0
     - 60
     - 0
     - 0
     - 0
     - 0
     - Slot→corner remap table; DRIVING @60 s
   * - ``PID_TPMS_PRESSURES`` (``0x1005``) *ISOTP_EXTADR*
     - TPMS GW (``0x750``/``0x2A``)
     - 0
     - 0
     - 60
     - 0
     - 0
     - 0
     - 0
     - TPMS pressures; DRIVING @60 s
   * - ``PID_TPMS_TEMPS`` (``0x1004``) *ISOTP_EXTADR*
     - TPMS GW (``0x750``/``0x2A``)
     - 0
     - 0
     - 60
     - 0
     - 0
     - 0
     - 0
     - TPMS temperatures; DRIVING @60 s
   * - ``PID_AUX_BATTERY_CURRENT`` (``0x15F7``)
     - EV ECU ``0x7D2``
     - 0
     - 0
     - 10
     - 0
     - 0
     - 10
     - 10
     - 12V aux current, ``(raw-0x8000)×0.0038147`` A (bidirectional) → ``v.b.12v.current`` + ``v.e.charging12v`` / ``v.e.aux12v``
   * - ``PID_AUX_BATTERY_VOLTAGE`` (``0x15EE``)
     - EV ECU ``0x7D2``
     - 0
     - 0
     - 30
     - 0
     - 0
     - 30
     - 30
     - 12V aux voltage (hi-res), ``u16×5/4096`` V → ``xte.v.b.12v.voltage``
   * - ``PID_AUX_BATTERY_TEMP`` (``0x15F8``)
     - EV ECU ``0x7D2``
     - 0
     - 0
     - 120
     - 0
     - 0
     - 120
     - 120
     - 12V aux temp, ``(raw-400)×0.1`` °C
   * - ``PID_AUX_BATTERY_FULL_CHARGE`` (``0x15E5``)
     - EV ECU ``0x7D2``
     - 0
     - 0
     - 120
     - 0
     - 0
     - 120
     - 120
     - 12V aux capacity (CAC), ``u8×0.5`` Ah
   * - ``PID_AUX_BATTERY_INTEGRATORS`` (``0x15E8``)
     - EV ECU ``0x7D2``
     - 0
     - 0
     - 120
     - 0
     - 0
     - 120
     - 120
     - 12V lifetime charge/discharge Ah + ready-on hours

Column headers: **HS** = CHARGE_HANDSHAKE, **WAIT** = CHARGE_WAIT, **AC** = CHARGE_AC,
**DC** = CHARGE_DC.  ``0`` = not polled in that state.

The following PID is declared in the source but **not currently polled** (all columns are 0)
and is noted for future use:

* ``PID_CHARGING_CONTROL_INFORMATION`` (``0x1689``) — deferred.

Web UI
======

The following pages are registered by ``WebInit()`` for all e-TNGA vehicles and appear in the
vehicle menu of the OVMS web interface.  Page titles use the active vehicle name (e.g. "Subaru
Solterra" or "Toyota bZ4X").

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - URL
     - Menu
     - Purpose
   * - ``/bms/cellmon``
     - Vehicle
     - BMS cell monitor — per-cell voltage and temperature display.  Populated for all e-TNGA
       vehicles (the base declares the pack arrangement).
   * - ``/xte/charge``
     - Vehicle
     - Live charging metrics dashboard — key charge telemetry during a session.
   * - ``/xte/reports``
     - Vehicle
     - Saved charge-report browser — lists stored session reports with links to the HTML
       report and the per-sample CSV download.
   * - ``/xte/report``
     - (none)
     - Serves one stored report or CSV by filename (linked from ``/xte/reports``).
   * - ``/xte/config``
     - Vehicle
     - Vehicle configuration — TPMS alert thresholds and the battery capacity reference,
       i.e. all six ``[xte]`` parameters, via a form rather than the shell ``config set``
       command.  Also displays the detected pack and the nominal currently in use.

Charge session report
=====================

At the end of each charging session (plug-in to unplug), the module writes a self-contained
HTML report and a fine-grained per-sample CSV to the charge-report directory.  The directory
is ``/sd/charge-reports/`` when an SD card is mounted, falling back to
``/store/charge-reports/`` on internal flash.  The newest 50 sessions are retained; older
sessions (both ``.html`` and ``.csv``) are pruned automatically.  Orphan CSV files whose
session never produced an HTML report (e.g. aborted sessions) are also removed at close.

Report contents
---------------

The self-contained HTML report includes:

* **Summary** — plug-in and unplug timestamps (UTC), duration, plug-in GPS location (with
  OpenStreetMap link), ambient temperature range, charge type (AC / DC fast), SOC start→end,
  delivered energy (kWh) and grid energy (kWh), peak and average power, battery temperature
  range, and the 0x1688 outcome decoded to a human-readable label (e.g. "DC Charging Stop
  (System)") with raw hex fallback for unknown codes.
* **Inline SVG power/SOC chart** — downsampled (≤300 points) delivered-power trace and a
  light SOC overlay, rendered on-module without any external dependencies.
* **Session event log** — timestamped table of state-machine events (plug-in, charge phases
  started/ended, unplug).
* **Estimates** — charging efficiency (AC sessions, where grid kWh is available) and implied
  pack capacity derived from delivered Ah and SOC delta.
* **Link to the per-sample CSV** — download the raw time-series data for offline analysis.

Per-sample CSV columns
----------------------

The CSV is streamed to disk one row per second during active charging (``CHARGE_AC`` or
``CHARGE_DC``).  The header row defines the columns::

    phase, elapsed_s, soc_pct, bms_soc_pct, station_kw, battery_kw, hvac_kw,
    pack_v, pack_a, batt_temp_c, ambient_c, state,
    station_max_kw, station_max_a, station_max_v,
    car_perm_kw, car_target_a,
    station_grid_kw, station_present_v, station_present_a, obc_kw,
    batt_tmin_c, batt_tmax_c,
    lifetime_min, lifetime_acc, delivered_ah

Grouped by what each column represents:

* **Session position** — ``phase`` (1-based; a plug-in can contain several charge phases
  separated by a pause) and ``elapsed_s`` (seconds since the session opened, not since
  the phase).
* **State of charge** — ``soc_pct`` as the vehicle reports it, ``bms_soc_pct`` as the BMS
  does; the two differ, which is why both are recorded.
* **Actual power** — ``station_kw`` drawn from the EVSE, ``battery_kw`` into the pack,
  ``hvac_kw`` into the cabin.  These are measurements, not a split of one figure: the
  difference between them is losses plus anything else the car is running.
* **Pack** — ``pack_v``, ``pack_a``, ``batt_temp_c``, plus ``ambient_c`` (empty, not
  ``0.0``, until the first in-charge reading arrives roughly 30 s in — an empty field is
  distinguishable from a genuine 0 °C).
* **Pack temperature spread** — ``batt_tmin_c`` and ``batt_tmax_c``, the coolest and
  hottest of the pack's temperature sensors.  ``batt_temp_c`` is their **mean**, and the
  spread is not negligible: roughly 3 °C between coolest and hottest even during a gentle
  AC charge.  A BMS derates on its **hottest** sensor, so a charging curve should be keyed
  to ``batt_tmax_c`` rather than the mean.  Both read ``0.0`` until the first temperature
  reply of the session arrives.
* **``state``** — ``AC`` or ``DC``.
* **The station's caps** — ``station_max_kw`` / ``_a`` / ``_v``.
* **The car's asks** — ``car_perm_kw`` (DID ``0x16A1``) and ``car_target_a``
  (DID ``0x166D``), both from the Plug-In Control / OBC ECU (``0x745``).  ``car_perm_kw``
  is stored **signed** and reads negative while charging.
* **Raw station telemetry** — ``station_grid_kw``, ``station_present_v``,
  ``station_present_a`` (zero when not applicable).
* **``obc_kw``** — diagnostic only: the raw DID ``0x10D4`` reading, which under-reads on
  DC charging.  Use ``battery_kw`` for real power.
* **Lifetime counters and delivered charge** — ``lifetime_min`` and ``lifetime_acc`` are the
  two counters read from DID ``0x1D70`` on the HV Battery ECU, logged raw because their units
  are not established; ``delivered_ah`` is the module's own charge integral for the session.
  The three are recorded together so the counters can be calibrated against a known
  ampere-hour figure offline.

.. note::

   **Parse by header name, not by column index.**  The column set has grown across
   releases and one column (``phase``) was inserted at the front rather than appended, so
   positional parsers written against an older file will silently misread every field.
   The header row is authoritative and is written at the top of every file, so a
   name-based reader stays correct across all versions.

   For reference when handling older files: the original layout had 16 columns beginning
   with ``elapsed_s``; ``phase`` and ``bms_soc_pct`` were added later, the single
   ``delivered_kw`` column was replaced by the separate ``station_kw`` / ``battery_kw`` /
   ``hvac_kw`` measurements, ``obc_kw`` was added at the end, and ``target_a`` / ``grid_kw``
   / ``present_v`` / ``present_a`` were renamed to their ``car_`` and ``station_``
   prefixed forms.

Configuration
=============

All e-TNGA vehicles share the ``xte`` config instance, registered by the base module.  Every
parameter can be set from the shell with ``config set xte <param> <value>`` or from the
``/xte/config`` web page.

.. list-table::
   :header-rows: 1
   :widths: 28 12 60

   * - Parameter
     - Default
     - Meaning
   * - ``tpms.pressure.warn``
     - 240
     - Tyre pressure (kPa) below which a warning is raised
   * - ``tpms.pressure.alert``
     - 220
     - Tyre pressure (kPa) below which an alert is raised
   * - ``tpms.temp.warn``
     - 90
     - Tyre temperature (°C) above which a warning is raised
   * - ``tpms.temp.alert``
     - 100
     - Tyre temperature (°C) above which an alert is raised
   * - ``bat.nominal.ah``
     - 0
     - Pack nominal full-charge capacity (Ah), the denominator for ``v.b.soh``.
       ``0`` means derive it from the detected pack, which is only established for
       the 96-cell pack; on other variants ``v.b.soh`` stays empty until this is set
   * - ``bat.nominal.volt``
     - 0
     - Pack nominal voltage (V), used to convert the capacity to kWh for
       ``v.b.capacity``.  ``0`` derives it from the detected pack, 96-cell only

Metrics
=======

Standard OVMS metrics populated by the e-TNGA module (in addition to the obvious SOC, speed,
odometer, temperature, and VIN):

* ``v.e.cooling`` — A/C cooling active (true when ``0x106E`` HVAC power > 0)
* ``v.e.heating`` — HV heater active (true when ``0x1086`` heater power > 0)
* ``v.e.cabinfan`` — Blower level as a percentage (from ``0x2801``, levels 1–7)
* ``v.b.consumption`` — Trip-average energy consumption (Wh/km), computed each tick while
  DRIVING from net trip energy / trip distance
* ``v.b.coulomb.used`` / ``v.b.coulomb.recd`` — Per-trip charge/discharge coulombs (Ah),
  reset at trip start; ``v.b.coulomb.used.total`` / ``v.b.coulomb.recd.total`` are
  persistent lifetime accumulators
* ``v.b.energy.used`` / ``v.b.energy.recd`` — Per-trip energy (kWh); ``.total`` variants
  are persistent
* ``v.c.power`` (``ms_v_charge_power``) — Battery-delivered charge power (DID ``0x10D4``),
  valid for both AC and DC charging
* ``v.c.kwh.grid`` / ``v.c.kwh.grid.total`` — AC grid energy delivered this session /
  lifetime, integrated from ``0x161D`` (AC only; reads 0 during DC)
* ``v.b.cac`` — Pack full-charge capacity (Ah) as measured by the BMS, summed from the
  eight per-module slots of ``0x1D3E`` on the HV Battery ECU (``0x747``)
* ``v.b.soh`` / ``v.b.capacity`` — State of health (%) and usable capacity (kWh), derived
  from ``v.b.cac`` against the pack nominal.  Only the 96-cell pack has a built-in nominal;
  on other variants both stay empty until ``[xte] bat.nominal.ah`` / ``bat.nominal.volt``
  are set, since a wrong nominal yields a believable but wrong percentage.  ``v.b.soh`` is
  deliberately **not** clamped at 100 % — a reading above 100 % means the configured
  nominal does not match the pack

Custom metrics (``xte.*`` namespace; 18 ``xte.v.c.*`` charge metrics plus the battery,
12 V and climate metrics below):

* ``xte.v.c.gridpower`` — Grid input power (kW) from DID ``0x161D``; AC charging only
  (the grid-side companion to ``v.c.power``)
* ``xte.v.e.hvac.power`` — HVAC / cabin power draw (kW); sourced from the OBC (``0x745``,
  DID ``0x106E``) while charging and from the hybrid control ECU (``0x7D2``) while driving
* ``xte.v.e.hvac.kwh`` — My-Room cabin energy (kWh); direct time-integral of
  ``xte.v.e.hvac.power`` over the My-Room-active interval, reset at charge start; valid for
  both AC and DC
* ``xte.v.e.hvac.kwh.drive`` — Per-trip driving cabin/HVAC energy (kWh); time-integral of
  HVAC power while in the DRIVING state, reset at trip start
* ``xte.v.c.hlcstate`` / ``xte.v.c.piswraw`` — The two raw charge-protocol signals the state
  machine runs on: the DC HLC state from ``0x1666`` and the PISW cable-seated signal from
  ``0x1669``, both on the Plug-In Control ECU.  Exposed unmodified so a session can be
  followed, or a stuck handshake diagnosed, from the metrics alone.  See
  :doc:`state_machine` for the values each transition tests

HV battery internals (``0x747`` unless noted):

* ``xte.v.b.soc.bms`` — SOC as the BMS reports it (``0x1F5B``), which differs from the
  displayed ``v.b.soc``: the displayed figure spans only the usable part of the pack and
  reaches 100 % at roughly 95 % BMS
* ``xte.v.b.temp.coolant`` / ``xte.v.b.temp.heater`` — Battery coolant and coolant-heater
  temperatures (°C)
* ``xte.v.b.heater`` — Battery coolant heater relay engaged
* ``xte.v.b.cap.full`` / ``xte.v.b.cap.alt`` — The raw eight-slot per-module capacity arrays
  from ``0x1D3E`` and ``0x1D3F``.  ``cap.full`` is what ``v.b.cac`` is summed from; ``cap.alt``
  is collected for analysis only, its function unconfirmed
* ``xte.v.b.lifetime.acc`` / ``xte.v.b.lifetime.min`` — The two lifetime counters from
  ``0x1D70``, logged raw because their units are unresolved

12 V auxiliary battery (EV ECU ``0x7D2``; see the poll table for the DIDs and scaling):

* ``xte.v.b.12v.voltage`` — Hi-res aux voltage from ``0x15EE``, alongside the module's own
  ADC reading in ``v.b.12v.voltage``
* ``xte.v.b.12v.temp`` — Aux battery temperature (°C) from ``0x15F8``
* ``xte.v.b.12v.cac`` — Aux battery full-charge capacity (Ah) from ``0x15E5``
* ``xte.v.b.12v.charge.ah`` / ``xte.v.b.12v.discharge.ah`` / ``xte.v.b.12v.readyon.h`` —
  Lifetime charge and discharge integrals (Ah) and integrated Ready-ON time (h), all from
  ``0x15E8``

.. toctree::
   :maxdepth: 1

   state_machine
