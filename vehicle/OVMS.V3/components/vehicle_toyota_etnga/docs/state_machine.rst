=============
State Machine
=============

The eTNGA driver — and therefore the Subaru Solterra driver, which is a thin
subclass that adds nothing of its own — runs two coupled state variables.
This page documents how they interact, what drives transitions, and the
quirks worth knowing when extending the module.

Two parallel states
===================

.. list-table::
   :header-rows: 1
   :widths: 20 25 30 25

   * - State
     - Type
     - Source
     - Values
   * - ``PollState`` (``m_s_pollstate``)
     - Driver-owned; selects which OBD-II PIDs are polled
     - Internal transitions in ``etnga_poll_states.cpp``
     - ``SLEEP``, ``AWAKE``, ``DRIVING``, ``CHARGE_HANDSHAKE``,
       ``CHARGE_WAIT``, ``CHARGE_AC``, ``CHARGE_DC``
   * - ``ControlState`` (``xte.s.controlstate``)
     - Vehicle-reported
     - PID ``0x10D1`` on the Plug-In Control ECU
     - ``CS_NONE=0``, ``CS_DRIVING=1``, ``CS_CHARGING=3``

``PollState`` selects which PIDs are polled — see the ``{S, A, D, C}``
columns in the ``obdii_polls[]`` table in ``vehicle_toyota_etnga.cpp``.
``ControlState`` is the vehicle's own self-report and is the primary trigger
for the ``AWAKE → DRIVING`` edge.

Charge entry trigger
--------------------

The old 4-state model entered a single ``CHARGING`` state on
``controlstate == CS_CHARGING`` (0x10D1 = 0x03).  The 7-state model no
longer uses ``CS_CHARGING`` as a charge-entry trigger.  Instead,
``AWAKE → CHARGE_HANDSHAKE`` fires when the PISW cable-seated signal
(DID ``0x1669``) reads ``≥ 0x02``.  This is *earlier* in the sequence —
the plug-in is detected before the HV control mode flips to
``CS_CHARGING`` — and allows the driver to track the full negotiation
window.

Tick loop
=========

``Ticker1()`` runs once per second and does two things:

1. ``ResetStaleMetrics()`` manually clears ``controlstate``,
   ``ms_v_env_awake``, ``ms_v_door_chargeport``, ``ms_v_charge_pilot``,
   and ``ms_v_bat_power`` if they have gone stale but still hold a
   non-default value.  This is the only way ``ms_v_env_awake`` ever drops
   back to ``false``.
2. Dispatches to the appropriate handler —
   ``HandleSleepState()``, ``HandleAwakeState()``,
   ``HandleDrivingState()``, ``HandleChargeHandshakeState()``,
   ``HandleChargeWaitState()``, ``HandleChargeAcState()``, or
   ``HandleChargeDcState()`` — based on the current ``m_poll_state``.

VIN acquisition is not driven from the tick loop.  ``RequestVIN()`` is
called once on entry to ``DRIVING`` and once on entry to
``CHARGE_HANDSHAKE`` (see the transition table); it short-circuits if
``ms_v_vin`` is already populated, and the underlying poll is a
``OnceOffPoll`` with three retries.

Transition diagram
==================

::

                        ┌──────────────────────────────────┐
                        │                                  │
                        ▼                                  │
          ┌─────────────────┐                              │
   start →│      SLEEP      │◄─────────────────────┐       │
          └─────────────────┘                      │       │
              │     ▲                              │       │
   CAN traffic│     │ env_awake stale (~120 s)      │       │
   OR 12V >   │     │ OR 5-min / 15-min watchdog    │       │
   ref+0.2 V  │     │ (arms 10 s cooldown)          │       │
              ▼     │                              │       │
          ┌─────────────────┐                      │       │
          │      AWAKE      │◄──────────────────┐  │       │
          └─────────────────┘                   │  │       │
              │              │                  │  │       │
              │ ctrl =       │ PISW ≥ 0x02       │  │       │
              │ CS_DRIVING   │ (cable seated)    │  │       │
              ▼              ▼                  │  │       │
          ┌──────────┐   ┌──────────────────┐   │  │       │
          │ DRIVING  │   │ CHARGE_HANDSHAKE │   │  │       │
          └──────────┘   └──────────────────┘   │  │       │
              │            │     │    │          │  │       │
   ctrl ≠     │   PISW=0   │     │    │60s stuck │  │       │
   CS_DRIVING │   (unplug) │     │    │at ac_op  │  │       │
              │            │  HLC│    │Stop+cable│  │       │
              │     ac_op  │  0A-│    │          │  │       │
              │     =0x02  │  12 │    ▼          │  │       │
              │            │     │  ┌──────────┐ │  │       │
              │            │     │  │   WAIT   │─┘──┘       │
              │            │     │  └──────────┘            │
              │            │     │    │     │  PISW=0        │
              │            ▼     ▼    │     │  (cable gone)  │
              │          ┌──────────┐ │     │                │
              │          │CHARGE_AC │ │     │                │
              │          └──────────┘ │     │                │
              │              │        │     │                │
              │    ac_op=0   │        │     │                │
              │    (Stop)    │        │     └───────────────►│
              │    or PISW=0 │        │     (WAIT→AWAKE =    │
              │              ▼        │      session end)    │
              │          ┌──────────┐ │                      │
              │          │CHARGE_DC │ │                      │
              │          └──────────┘ │                      │
              │              │        │                      │
              │  hlc=0xFF    │        │                      │
              │  or PISW=0   │        │                      │
              │              └────────┘                      │
              │           (AC/DC → WAIT)                     │
              └──────────────────────────────────────────────┘
                     (DRIVING → AWAKE)

There is no direct ``DRIVING → SLEEP`` or charge-state → ``SLEEP`` edge
(except ``CHARGE_WAIT`` on bus-dead) — most paths pass through ``AWAKE``.
There is also no direct edge between ``DRIVING`` and any charge state.

The polling feedback loop
=========================

The ``SLEEP → AWAKE`` edge is driven by external CAN traffic (any frame
on CAN2 sets ``ms_v_env_awake``).  Once in ``AWAKE``, however, the
poller is actively transmitting OBD-II requests on the same bus.  The
ECUs reply, and those replies are themselves CAN traffic — so the
driver's own polling refreshes ``ms_v_env_awake`` on every tick and
prevents the auto-stale that would otherwise bring it back to ``SLEEP``.

Practical consequences:

* ``AWAKE`` will not exit on its own via the ``env_awake`` stale path
  while the poller is running.  The 5-minute forced-sleep watchdog in
  ``HandleAwakeState`` exists precisely to break this loop when the
  vehicle never reports a clear ``DRIVING`` state and no cable is
  inserted.
* The watchdog must be paired with the cooldown latch — without it, the
  next poll reply after the forced ``TransitionToSleepState`` would
  immediately bounce the driver back to ``AWAKE``.
* Any future change that reduces poll throttling, adds more PIDs to the
  ``A`` column of ``obdii_polls[]``, or shortens the watchdog needs to
  consider whether it could keep a quiescent vehicle awake long enough
  to drain the 12 V battery.

Logging
=======

Every state transition is logged at ``ESP_LOGI`` level by
``SetPollState`` in ``etnga_metrics.cpp``::

    Transitioning from the <FROM> to the <TO> state

All seven ``TransitionTo*`` helpers route through ``SetPollState``, so
this single log line covers every edge in the diagram above.  Additional
context-specific log lines are emitted by ``HandleSleepState`` (12 V
wake, CAN reset result, cooldown expiry), ``HandleAwakeState``
(forced-sleep watchdog, wake-reconcile), and
``TransitionToChargeHandshakeState`` (session open).

Transition table
================

All edges live in ``etnga_poll_states.cpp``.

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - From → To
     - Condition
     - Notes
   * - ``SLEEP → AWAKE``
     - ``ms_v_env_awake == true``
     - Set by ``IncomingFrameCan2()`` on any CAN2 frame, gated by the
       cooldown latch (see below).
   * - ``SLEEP → AWAKE``
     - 12 V > 12 V-ref + 0.2 V
     - Issues ``m_can2->Reset()`` first to recover from a stuck bus.
       **Not** gated by the cooldown latch.
   * - ``AWAKE → SLEEP``
     - ``ms_v_env_awake == false``
     - Triggered when ``env_awake`` auto-stales (~120 s of no CAN
       frames) and ``ResetStaleMetrics`` flips it false.
   * - ``AWAKE → DRIVING``
     - ``controlstate == CS_DRIVING``
     - Marks the trip-start metric stale on entry so it resets on the
       next odometer reading.  Also calls ``RequestVIN()`` (no-op if VIN
       already cached).
   * - ``AWAKE → CHARGE_HANDSHAKE``
     - PISW (DID ``0x1669``) ``≥ 0x02`` (cable seated)
     - **Changed from old model** (was ``controlstate == CS_CHARGING``).
       Opens the in-RAM charge session if not already open.  Calls
       ``RequestVIN()``.  Sets ``ms_v_charge_state = "prepared"``.
   * - ``AWAKE → SLEEP`` (forced, door watch)
     - ``monotonic - m_v_env_awaketime > 300`` AND charge door never opened
     - 5-minute watchdog when awake with no ``CS_DRIVING`` and charge
       door not opened.  Arms the 10-second cooldown latch.
   * - ``AWAKE → SLEEP`` (forced, cable watch)
     - ``monotonic - m_cable_watch_start > 900`` (armed but no cable plug-in)
     - 15-minute watchdog: charge door was seen open (``m_armed_for_charge``
       set), but no cable seated within 15 minutes.  Arms the cooldown
       latch.
   * - ``DRIVING → AWAKE``
     - ``controlstate != CS_DRIVING``
     - Clears ready status, speed, gear, and env/cabin temperatures.
   * - ``CHARGE_HANDSHAKE → AWAKE``
     - PISW ``== 0x00`` (premature unplug)
     - Re-arms ``m_armed_for_charge`` and restarts the 15-min cable watch
       so the next plug-in gets a fresh window.  Resets the charge session
       and sets ``ms_v_charge_state = ""``.
   * - ``CHARGE_HANDSHAKE → CHARGE_DC``
     - HLC (DID ``0x1666``) in range ``0x0A–0x12``
     - DC fast-charge HLC sequence active.  Sets
       ``ms_v_charge_state = "charging"``.
   * - ``CHARGE_HANDSHAKE → CHARGE_AC``
     - ``ac_op`` (DID ``0x1684``) ``== 0x02`` (Running)
     - Only ``0x02`` (not ``0x01`` Startup) to avoid premature AC entry
       during negotiation.  Sets ``ms_v_charge_state = "charging"``.
   * - ``CHARGE_HANDSHAKE → CHARGE_WAIT``
     - 60 s elapsed AND ``ac_op == 0x00`` (Stop) AND PISW ``≥ 0x02``
     - Scheduled-charge heuristic: AC Op stuck at Stop for 60 s with cable
       present.  Sets ``ms_v_charge_state = "stopped"``.
   * - ``CHARGE_WAIT → AWAKE``
     - PISW ``== 0x00`` (cable removed)
     - Session end — resets the charge session and sets
       ``ms_v_charge_state = ""``.
   * - ``CHARGE_WAIT → CHARGE_AC``
     - ``ac_op == 0x01`` (Startup) or ``0x02`` (Running)
     - EVSE engaged (WAIT accepts Startup too, unlike HANDSHAKE).  Sets
       ``ms_v_charge_state = "charging"``.
   * - ``CHARGE_WAIT → CHARGE_DC``
     - HLC in range ``0x0A–0x12``
     - DC engaged from wait.  Sets ``ms_v_charge_state = "charging"``.
   * - ``CHARGE_WAIT → SLEEP``
     - ``ms_v_env_awake == false``
     - Bus went dead during scheduled wait (OBC slept or gateway isolated
       OBD).  Arms the 10-second cooldown latch.
   * - ``CHARGE_AC → CHARGE_WAIT``
     - ``ac_op == 0x00`` (Stop) OR PISW ``== 0x00``
     - Phase ended cleanly or cable pulled.  Sets
       ``ms_v_charge_state = "stopped"``.
   * - ``CHARGE_DC → CHARGE_WAIT``
     - ``hlc == 0xFF`` (Unconnected) OR PISW ``== 0x00``
     - DC phase ended or cable pulled.  Sets
       ``ms_v_charge_state = "stopped"``.

Charge state strings (``ms_v_charge_state``)
============================================

``SetChargeState()`` in ``etnga_metrics.cpp`` maps ``PollState`` to the
OVMS standard charge-state string written to ``ms_v_charge_state``:

.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - PollState
     - String
     - When
   * - ``CHARGE_HANDSHAKE``
     - ``"prepared"``
     - Cable seated; negotiation in progress; not yet delivering energy
   * - ``CHARGE_WAIT``
     - ``"stopped"``
     - Plugged in but not charging (scheduled wait, between phases, etc.)
   * - ``CHARGE_AC``
     - ``"charging"``
     - AC energy delivery active (``ms_v_charge_inprogress = true``)
   * - ``CHARGE_DC``
     - ``"charging"``
     - DC fast-charge active (``ms_v_charge_inprogress = true``)
   * - session end (``AWAKE``)
     - ``"done"``
     - Written in ``TransitionToAwakeState()`` when arriving from
       ``CHARGE_AC`` or ``CHARGE_DC``
   * - session cancelled (``AWAKE``)
     - ``""``
     - Written in ``TransitionToAwakeState()`` when arriving from
       ``CHARGE_HANDSHAKE`` or ``CHARGE_WAIT`` (no energy delivered or
       premature unplug)

Charge session (in-RAM)
=======================

A ``ChargeSessionState`` struct (member ``m_charge_session``) tracks the
current plug-in event entirely in RAM — there is no flash persistence:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Field
     - Meaning
   * - ``in_session``
     - ``true`` from first ``CHARGE_HANDSHAKE`` entry until session close
   * - ``start_monotonic``
     - ``ms_m_monotonic`` value at session open (seconds since boot)
   * - ``start_soc``
     - ``ms_v_bat_soc`` integer value at session open

**Session open:** ``TransitionToChargeHandshakeState()`` opens the
session (sets ``in_session = true``) on first entry.  If the driver
bounces HANDSHAKE → AWAKE → HANDSHAKE (e.g. DCFC retry dance), the
open-guard (``if (!m_charge_session.in_session)``) prevents a second
open.

**Session close:** ``TransitionToAwakeState()`` resets
``m_charge_session`` to its default (``in_session = false``) whenever it
arrives from any charge state.

**Wake-reconcile:** ``PollState::SLEEP`` is stop-polling only — the
ESP32 stays running.  If the driver goes to SLEEP while a session is
open (cable still in but EVSE off, bus silenced), the session struct
survives.  On the next ``SLEEP → AWAKE`` transition,
``HandleAwakeState()`` checks: if ``in_session`` is true *and* the PISW
metric is fresh (non-stale) *and* PISW reports ``0x00`` (unconnected),
the cable was removed during the sleep gap — the driver finalises the
session immediately (sets ``ms_v_charge_state = "done"`` and resets
``m_charge_session``).  The AWAKE-column PISW poll (DID ``0x1669``) in
``obdii_polls[]`` is required for this reconcile to fire — removing it
would silently break the wake-reconcile path.

Charge curve & station metrics
==============================

Tasks 1–4 of the ``v3-charge-statemachine`` feature added eleven custom
``xte.v.c.*`` metrics and began populating two standard OVMS metrics
during charging.  They are decoded by ``IncomingPlugInControlSystem()``
in ``etnga_poll_processor.cpp`` (with scale/decode math in
``etnga_metrics.cpp``) as the poller runs in the ``CHARGE_AC`` and
``CHARGE_DC`` states.

DC / universal custom metrics
------------------------------

These are populated during DC charging (and where applicable, AC charging).

.. list-table::
   :header-rows: 1
   :widths: 20 12 12 56

   * - Metric
     - DID
     - Unit
     - Meaning
   * - ``xte.v.c.perm``
     - ``0x16A1``
     - kW
     - Minimum charging permission power — the taper floor of the DC
       charge curve.  Decoded as ``s16 BE × 0.01 kW/LSB``.  The
       ``0x8000`` sentinel is skipped (metric left unchanged).
   * - ``xte.v.c.tgti``
     - ``0x166D``
     - A
     - Target charging current set by the vehicle during DC
       fast-charge.
   * - ``xte.v.c.stamaxp``
     - ``0x166A``
     - kW
     - DC station maximum power capability as negotiated via HLC.
   * - ``xte.v.c.stamaxi``
     - ``0x1679``
     - A
     - DC station maximum current (CCS contract).
   * - ``xte.v.c.stamaxv``
     - ``0x1681``
     - V
     - DC station maximum voltage (CCS contract).

Standard metrics driven during DC charging
-------------------------------------------

These standard OVMS metrics are populated (for the first time in this
module) while ``PollState == CHARGE_DC``:

.. list-table::
   :header-rows: 1
   :widths: 25 12 12 51

   * - Metric
     - DID
     - Unit
     - Meaning
   * - ``v.c.voltage`` (``ms_v_charge_voltage``)
     - ``0x166B``
     - V
     - DC station present voltage.
   * - ``v.c.current`` (``ms_v_charge_current``)
     - ``0x166C``
     - A
     - DC station present current.

AC-only custom metrics
-----------------------

These metrics are populated only while ``PollState == CHARGE_AC``.  Note
that ``xte.v.c.chgrop`` (charger operation status, from ``0x1619`` byte 3)
is distinct from ``xte.v.c.acop`` (``0x1684``), which drives the
``CHARGE_HANDSHAKE → CHARGE_AC`` transition.

.. list-table::
   :header-rows: 1
   :widths: 20 16 12 52

   * - Metric
     - DID / bytes
     - Unit
     - Meaning
   * - ``xte.v.c.actgtp``
     - ``0x1619`` b1–2
     - kW
     - AC target charging power.  Decoded as ``u16 BE biased-32768
       × 0.01 kW/LSB``.
   * - ``xte.v.c.chgrop``
     - ``0x1619`` b3
     - enum
     - Charger operation status (see source for enum values).
   * - ``xte.v.c.acilim``
     - ``0x1619`` b4–5
     - RAW
     - AC charging current upper limit (raw integer; scale pending).
   * - ``xte.v.c.chgout``
     - ``0x161E`` b1–2
     - RAW
     - Charger output power (raw integer; scale pending).
   * - ``xte.v.c.chgotgt``
     - ``0x161E`` b3–4
     - RAW
     - Target-from-charger power (raw integer; scale pending).
   * - ``xte.v.c.acusbl``
     - ``0x1665``
     - RAW
     - A/C useable power (raw integer; scale pending).

.. note::

   The four RAW AC channels (``xte.v.c.acilim``, ``xte.v.c.chgout``,
   ``xte.v.c.chgotgt``, ``xte.v.c.acusbl``) store unscaled integer
   values.  Their physical scales are deferred pending a sustained
   AC-charge capture.

Charge-report supporting channels
----------------------------------

These four metrics feed the in-module charge report (work item D).
``xte.v.c.myroom`` and ``xte.v.c.acpwr`` are polled only in the
``CHARGE_AC`` and ``CHARGE_DC`` states; ``xte.v.c.outcome`` and
``xte.v.c.stopreq`` are also polled in ``CHARGE_HANDSHAKE`` and
``CHARGE_WAIT``.

.. list-table::
   :header-rows: 1
   :widths: 22 12 10 56

   * - Metric
     - DID / bytes
     - Unit
     - Meaning
   * - ``xte.v.c.myroom``
     - ``0x1692`` b2 bit 0
     - bool
     - My Room active — cabin run on grid power while plugged in or
       charging.  This is the only on-bus live My-Room signal.
   * - ``xte.v.c.acpwr``
     - ``0x106E``
     - kW
     - A/C consumption power (cabin / HVAC draw).  Decoded as
       ``u8 × 0.05 kW/LSB`` at offset 0.  ``0x106E`` is a dedicated
       cabin-power PID, so the My-Room cabin energy is the **direct
       time-integral of this channel** (∫ acpwr dt over the
       My-Room-active interval) — *not* an EVSE-to-cabin
       (input − pack) split.  The old delta method was AC-only and
       read 0 for DC (``0x161D`` charger-input is 0 during DC), so the
       direct integral is used for both AC and DC (confirmed
       2026-06-03; host ``charge_report_writer.py`` / ``analyze-myroom``
       use the same direct integral).
   * - ``xte.v.c.outcome``
     - ``0x1688``
     - enum
     - Charging History outcome — why the session ended (e.g.
       ``0x39`` = DC Charging Stop (System)).  26-state enum; 6
       states empirically confirmed.  **Retained between sessions**:
       the vehicle does not reset this on plug-in, so a report must
       scope it per-session (capture value at session open and
       compare at session close).
   * - ``xte.v.c.stopreq``
     - ``0x1667``
     - enum
     - Charge Sequence Stop Request from the CCM — HLC-layer fault
       reason.  ``0x00`` = Normal; ``0x06`` = "HLC Detection
       Communication Error".  Full enum TBD (partial decode).

.. note::

   **Limiting-side attribution (work item D, not yet implemented).**
   When the report attributes who capped the charge rate, compare the
   car's permission ``0x16A1`` against the station's **declared max**
   ``0x166A`` and take the lower as the binding cap.  Do **not** fold the
   station's instantaneous V×I output (``0x166B`` × ``0x166C``) in as a
   "station cap": that is delivered power — always ≤ the true cap — so it
   makes every BMS taper misread as station-limited.  Label a non-cold
   car-limited phase "taper".  (Host-side ``charge_report_writer`` had
   exactly this bug; fixed 2026-06-03 — a real DC session that tapered
   from 61.8 → 51.3 kW under a 100 kW station was wrongly reported
   "station 60.6 kW" before the fix.)

Cooldown latch
==============

To prevent flapping after the forced-sleep watchdogs,
``HandleAwakeState`` sets ``m_allow_wake = false`` and records
``m_sleep_entry_time`` before transitioning to ``SLEEP``.  While the
latch is held:

* ``IncomingFrameCan2`` does **not** call ``SetAwake(true)``, so trailing
  post-shutdown CAN traffic cannot bounce the driver straight back to
  ``AWAKE``.
* After 10 seconds, ``HandleSleepState`` clears the latch and CAN frames
  can wake the driver again.

The 12 V-based wake path is **not** gated by ``m_allow_wake`` — high aux
voltage will pull the driver out of ``SLEEP`` even mid-cooldown.

``CHARGE_WAIT → SLEEP`` also arms the cooldown latch (same 10-second
window) to prevent immediate re-wake from bus noise after the OBC sleeps.

Boot
====

The constructor calls ``TransitionToSleepState()`` and
``PollSetThrottling(0)``.  The driver always starts in ``SLEEP`` and
waits for CAN activity or a 12 V bump before doing anything else.
``TransitionToSleepState()`` also clears ``m_armed_for_charge``.

Notes and quirks
================

* **Two views of "vehicle on".** ``ms_v_env_awake`` (anything on the bus)
  drives ``SLEEP ↔ AWAKE``; ``controlstate`` (vehicle-reported mode)
  drives ``AWAKE ↔ DRIVING``; PISW (cable-seated signal) drives
  ``AWAKE → CHARGE_HANDSHAKE``.  Future wake/sleep tweaks should preserve
  this split.
* **Charge entry no longer uses CS_CHARGING.** The old
  ``controlstate == CS_CHARGING`` trigger is gone; PISW ``≥ 0x02`` fires
  earlier (plug-in detected before HV mode flips).
* **``env_awake`` is never explicitly cleared.** It falls to ``false``
  only via auto-stale plus the manual reset in ``ResetStaleMetrics``.
  Anything that changes its auto-stale period changes the ``SLEEP``
  detection latency.
* **Direct ``DRIVING ↔ charge-state`` is impossible.** A vehicle that
  flips control mode from drive to charge spends at least one tick in
  ``AWAKE`` in between, which clears trip metrics.
* **AC timeout resistance.** ``HandleChargeAcState`` and
  ``HandleChargeDcState`` intentionally do not tear down the session on
  OBC poll timeouts.  Locking the car during charging causes the gateway
  to isolate OBD from the OBC; on unlock polling resumes normally.  Only
  an explicit fresh ``PISW = 0x00`` or a clean phase-end signal
  (``ac_op == 0x00`` / ``hlc == 0xFF``) terminates a charge phase.
* **The 12 V wake threshold depends on a calibrated reference.**
  ``HandleSleepState`` compares against
  ``ms_v_bat_12v_voltage_ref + 0.2``; on an uncalibrated module the
  CAN-frame path is the reliable wake mechanism.
* **Driving-state exit clears more than charge-state exit.**
  ``HandleDrivingState`` clears speed, gear, and temperatures; charge
  states clean up via ``SetChargingStatus(false)`` and ``SetChargeState``
  only.  Charge metrics added in the future may need explicit clearing on
  exit.
* **No flash persistence for charge sessions.** ``m_charge_session`` lives
  only in RAM.  A hard reset or power cycle mid-session will lose the
  session-open state; the wake-reconcile will not fire because
  ``in_session`` will be ``false`` after boot.

TPMS
====

Tyre pressure and temperature data are retrieved from the TPMS ECU via the
gateway sub-target ``0x2A`` at address ``0x750``, using the module's
``ISOTP_EXTADR`` mixed-addressing mode.  This is the first gateway-relay poll
in the eTNGA module.  Readings are collected every 60 seconds while in the
``DRIVING`` poll state.  The gateway sub-target ``0x2A`` only answers while the
car is driving (or in *My Room*); parked-asleep and during charging it does not
respond, so polling it in ``AWAKE`` only produced timeouts and was dropped.

Metrics
-------

Three standard OVMS vector metrics are populated, one element per wheel in
canonical order ``[FL, FR, RL, RR]`` (indices 0–3):

.. list-table::
   :header-rows: 1
   :widths: 30 35 10 25

   * - Metric
     - OVMS name
     - Unit
     - Meaning
   * - ``v.t.pressure``
     - ``ms_v_tpms_pressure``
     - kPa
     - Tyre pressures
   * - ``v.t.temp``
     - ``ms_v_tpms_temp``
     - °C
     - Tyre temperatures
   * - ``v.t.alert``
     - ``ms_v_tpms_alert``
     - enum
     - 0 = normal, 1 = warning, 2 = alert

Source DIDs
-----------

All reads go to gateway ``0x750`` sub-target ``0x2A`` via ``ISOTP_EXTADR``.

.. list-table::
   :header-rows: 1
   :widths: 12 20 68

   * - DID
     - Data
     - Decode
   * - ``0x1005``
     - Pressures (5 × u16 status/raw pairs)
     - ``psi = raw × 0.25 − 7.35``; convert to kPa (``× 6.89476``).
       A slot with raw value 0 is treated as no-sensor and excluded from
       alerts.
   * - ``0x1004``
     - Temperatures (5 slot bytes)
     - ``°C = raw − 40``.
   * - ``0x2021``
     - Slot→corner map (5 corner bytes)
     - Each byte gives the corner ID for that slot: 1 = FL, 2 = FR,
       3 = RL, 4 = RR.  Re-read every poll cycle to keep the remap table
       current.

Slot→corner remap
-----------------

The TPMS ECU numbers sensor *slots* (physical transmitter positions as
learned during the last relearn), not *corners* (FL/FR/RL/RR positions on
the car).  DID ``0x2021`` provides the current slot→corner mapping; the
module re-reads it every poll cycle (60 s) and caches the latest mapping.

The mapping places each reading at ``vector_index = corner_id − 1``:

* corner 1 (FL) → index 0
* corner 2 (FR) → index 1
* corner 3 (RL) → index 2
* corner 4 (RR) → index 3

.. note::

   The mapping is car-specific and changes after a tyre rotation or TPMS
   relearn.  Because ``0x2021`` is polled every cycle (60 s), the module
   picks up the new assignment automatically within about a minute — no
   restart required.

Alert thresholds
----------------

Four ``xte`` config params control the alert levels.  Pressure uses a
low-pressure test (``≤``); temperature uses an overheat test (``≥``):

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Config key
     - Default
     - Meaning
   * - ``tpms.pressure.warn``
     - 240 kPa
     - Pressure at or below which a tyre enters the *warning* state
   * - ``tpms.pressure.alert``
     - 220 kPa
     - Pressure at or below which a tyre enters the *alert* state
   * - ``tpms.temp.warn``
     - 90 °C
     - Temperature at or above which a tyre enters the *warning* state
   * - ``tpms.temp.alert``
     - 100 °C
     - Temperature at or above which a tyre enters the *alert* state

.. warning::

   For the three-level (normal / warning / alert) behaviour to work
   correctly, the pressure warn threshold must be **above** the alert
   threshold (``warn ≥ alert``) and the temperature warn threshold must be
   **below** the alert threshold (``warn ≤ alert``).  Reversing this
   collapses the intermediate warning state: a tyre would jump straight
   from normal to alert with no warning step.

Notes
-----

* **Values are stale when parked.** TPMS sensors are motion-activated —
  they transmit only while the wheels are rolling.  The ECU receiver holds
  the last values reported while the car was in motion; these are the values
  the module reads.  Freshness is not guaranteed for a vehicle that has been
  stationary for an extended period.
* **No-sensor slots publish 0 and are excluded from alerts.** A slot whose
  pressure raw byte is 0 is treated as unpopulated; the corresponding
  ``v.t.pressure`` element is published as 0.0 and is skipped when evaluating
  alert thresholds.
