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
Charge Interruption Alerts  No
Charge Control              No
Cabin Pre-heat/cool Control No
Lock/Unlock Vehicle         No
Valet Mode Control          No
Others                      VIN
=========================== ==============

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

Last reviewed: 2026-07-26.

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
     - Vehicle-validated (partial)
     - A 14.6 kWh charge on 2026-07-13 produced zero spurious "Vehicle is idling"
       alerts.  The **true-positive** path — a genuine idle alert that *should*
       fire — remains unexercised.
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
     - 16 of 16 DC fast charges on the 2026-07-20 road trip.
   * - Charge port ``v.d.cp`` latched at handshake
     - Vehicle-validated (partial)
     - Confirmed for plug-in while already ``AWAKE``.  The plug-in that *wakes the
       module from* ``SLEEP`` — the case the fix was written for, where PISW
       ``0x1669`` at 5 s beats the lid signal ``0x1625`` at 10 s — is unvalidated.
   * - ``v.c.type`` AC (``type1`` / ``type2``)
     - Vehicle-validated
     - Read correctly across AC sessions.
   * - ``v.c.type`` DC (``ccs``)
     - Unvalidated
     - DC sessions have occurred since the change merged, but the ``ccs`` value was
       never checked in the logs.  A cheap gap to close on the next DC charge.
   * - Charge power derived from pack V×I
     - Vehicle-validated
     - Energy reconciliation on 2026-06-24: 88% efficiency, station 0.17 kWh ≈
       battery 0.15 kWh, replacing earlier 52%/158% garbage.
   * - Charge-fault diagnostic DID dump
     - Vehicle-validated
     - A real ``0x29`` fault on 2026-06-24 fired the dump, labelled the triggering
       outcome correctly, and rendered decoded values.
   * - CAN-stale logging/accounting suspend
     - Vehicle-validated
     - A real lock produced an 85 s CSV gap with no stale rows, 2026-06-24.
   * - DC limiting-side attribution (car vs station)
     - Unvalidated
     - Merged without DC verification.  Only the AC path — correctly emitting no
       "Limiting" row — has been seen.  Thresholds (2 kW margin, 25 °C cold-battery)
       are provisional and expected to need tuning against real data.
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
   * - Capacity arrays ``0x1D3E`` / ``0x1D3F`` (HV Battery ECU ``0x747``/``0x74F``)
     - Log-inferred
     - Five weeks of logs show ``0x1D3E`` declining monotonically — a good
       ``v.b.cac`` candidate — while ``0x1D3F`` re-latches each cycle with no trend.
       Semantics unconfirmed, which is why neither is exposed as ``v.b.cac``.
   * - ``v.e.charging12v`` union rule
     - Vehicle-validated
     - 2026-07-20 road trip closed the last two gaps: the ``CHARGE_DC`` term
       (16/16 fast charges) and the 12V rising-edge wake trigger (4 clean fires).
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
   * - Throttle, drive mode, AWD, foot brake, park brake (Brake/EPB ECU ``0x7B0``)
     - Unvalidated
     - Merged but never checked against the car.  The foot-brake full-scale value
       in particular is expected to need retuning.

PID Polling Logic
=================

The poll list uses seven columns corresponding to the seven ``PollState`` values.  The number in
each column is the poll interval in seconds (``0`` = not polled in that state).  All rows use
``ISOTP_STD`` addressing unless noted.

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
     - 1
     - 1
     - 1
     - Control mode (CS_NONE / CS_DRIVING / CS_CHARGING); all active states @1 s
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
     - 0
     - 20
     - Cell-temperature array; DRIVING@10 s, DC@20 s
   * - ``PID_BATTERY_CELL_VOLTAGES`` (``0x182E``)
     - HV Battery
     - 0
     - 0
     - 5
     - 0
     - 0
     - 0
     - 30
     - Per-cell voltages (96 cells); DRIVING@5 s, DC@30 s
   * - ``PID_BATTERY_CAPACITY`` (``0x1D3E``)
     - HV Battery
     - 0
     - 60
     - 120
     - 0
     - 0
     - 60
     - 60
     - 8× per-module full-charge capacity (Ah); data collection
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
     - 0
     - 0
     - 0
     - 0
     - Not polled in any state; deferred until confirmed useful
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
     - 30
     - 1
     - 1
     - AC charger operation status; drives HANDSHAKE→AC transition
   * - ``PID_HLC_STATE`` (``0x1666``)
     - Plug-In Control
     - 0
     - 0
     - 0
     - 1
     - 30
     - 0
     - 1
     - DC HLC state; drives HANDSHAKE→DC / DC→WAIT transitions
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
     - TPMS alert threshold configuration — sets ``[xte] tpms.*`` config parameters via a
       form rather than the shell ``config set`` command.

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

    elapsed_s, soc_pct, delivered_kw, pack_v, pack_a, batt_temp_c, ambient_c, state,
    station_max_kw, station_max_a, station_max_v, car_perm_kw, target_a,
    grid_kw, present_v, present_a

Where ``state`` is ``AC`` or ``DC`` and ``grid_kw`` / ``present_v`` / ``present_a`` are
grid input power and DC station telemetry respectively (zero when not applicable).

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

Key custom metrics (``xte.*`` namespace, 18 total ``xte.v.c.*`` plus supporting metrics):

* ``xte.v.c.gridpower`` — Grid input power (kW) from DID ``0x161D``; AC charging only
  (the grid-side companion to ``v.c.power``)
* ``xte.v.e.hvac.power`` — HVAC / cabin power draw (kW); sourced from the OBC (``0x745``,
  DID ``0x106E``) while charging and from the hybrid control ECU (``0x7D2``) while driving
* ``xte.v.e.hvac.kwh`` — My-Room cabin energy (kWh); direct time-integral of
  ``xte.v.e.hvac.power`` over the My-Room-active interval, reset at charge start; valid for
  both AC and DC
* ``xte.v.e.hvac.kwh.drive`` — Per-trip driving cabin/HVAC energy (kWh); time-integral of
  HVAC power while in the DRIVING state, reset at trip start

.. toctree::
   :maxdepth: 1

   state_machine
