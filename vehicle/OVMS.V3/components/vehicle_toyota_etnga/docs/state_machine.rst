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
     - ``SLEEP``, ``AWAKE``, ``DRIVING``, ``CHARGING``
   * - ``ControlState`` (``xte.s.controlstate``)
     - Vehicle-reported
     - PID ``0x10D1`` on the Plug-In Control ECU
     - ``CS_NONE=0``, ``CS_DRIVING=1``, ``CS_CHARGING=3``

``PollState`` selects which PIDs are polled — see the ``{S, A, D, C}``
columns in the ``obdii_polls[]`` table in ``vehicle_toyota_etnga.cpp``.
``ControlState`` is the vehicle's own self-report and is the primary
trigger for the ``AWAKE → DRIVING`` and ``AWAKE → CHARGING`` edges.

Tick loop
=========

``Ticker1()`` runs once per second and does two things:

1. ``ResetStaleMetrics()`` manually clears ``controlstate``,
   ``ms_v_env_awake``, ``ms_v_door_chargeport``, ``ms_v_charge_pilot``,
   and ``ms_v_bat_power`` if they have gone stale but still hold a
   non-default value. This is the only way ``ms_v_env_awake`` ever drops
   back to ``false``.
2. Dispatches to ``HandleSleepState()``, ``HandleAwakeState()``,
   ``HandleDrivingState()``, or ``HandleChargingState()`` based on the
   current ``m_poll_state``.

VIN acquisition is not driven from the tick loop. ``RequestVIN()`` is
called once on entry to ``DRIVING`` and once on entry to ``CHARGING``
(see the transition table); it short-circuits if ``ms_v_vin`` is already
populated, and the underlying poll is a ``OnceOffPoll`` with three
retries.

Transition diagram
==================

::

                          ┌──────────────────────────────┐
                          │                              │
                          ▼                              │
            ┌─────────────────┐                          │
     start →│      SLEEP      │                          │
            └─────────────────┘                          │
                │     ▲                                  │
     CAN traffic│     │ env_awake stale (~120 s no CAN)  │
     OR 12V >   │     │ OR 5-min awake watchdog          │
     ref+0.2 V  │     │ (arms 10 s cooldown)             │
     (CAN reset)│     │                                  │
                ▼     │                                  │
            ┌─────────────────┐                          │
            │      AWAKE      │◄────────┐    ◄───────────┤
            └─────────────────┘         │                │
                │              │        │                │
                │ ctrl =       │ ctrl = │ ctrl ≠         │
                │ DRIVING      │ CHARG. │ DRIVING/CHARG. │
                ▼              ▼        │                │
            ┌──────────┐   ┌──────────┐ │                │
            │ DRIVING  │   │ CHARGING │─┘                │
            └──────────┘   └──────────┘                  │
                │              │                         │
                └──────────────┴── (no direct edge) ─────┘

There is no direct ``DRIVING → SLEEP`` or ``CHARGING → SLEEP`` edge —
those paths always pass through ``AWAKE``.

The polling feedback loop
=========================

The ``SLEEP → AWAKE`` edge is driven by external CAN traffic (any frame
on CAN2 sets ``ms_v_env_awake``). Once in ``AWAKE``, however, the
poller is actively transmitting OBD-II requests on the same bus. The
ECUs reply, and those replies are themselves CAN traffic — so the
driver's own polling refreshes ``ms_v_env_awake`` on every tick and
prevents the auto-stale that would otherwise bring it back to
``SLEEP``.

Practical consequences:

* ``AWAKE`` will not exit on its own via the ``env_awake`` stale path
  while the poller is running. The 5-minute forced-sleep watchdog in
  ``HandleAwakeState`` exists precisely to break this loop when the
  vehicle never reports a clear ``DRIVING`` or ``CHARGING`` state.
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

All four ``TransitionTo*`` helpers route through ``SetPollState``, so
this single log line covers every edge in the diagram above. Additional
context-specific log lines are emitted by ``HandleSleepState`` (12 V
wake, CAN reset result, cooldown expiry) and ``HandleAwakeState``
(forced-sleep watchdog).

Transition table
================

All edges live in ``etnga_poll_states.cpp``.

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

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
       next odometer reading. Also calls ``RequestVIN()`` (no-op if VIN
       already cached).
   * - ``AWAKE → CHARGING``
     - ``controlstate == CS_CHARGING``
     - Sets ``ms_v_charge_inprogress = true``. Also calls
       ``RequestVIN()`` (no-op if VIN already cached).
   * - ``AWAKE → SLEEP`` (forced)
     - ``monotonic - m_v_env_awaketime > 300``
     - 5-minute watchdog when awake but no clear ``DRIVING``/``CHARGING``
       state. Arms the 10-second cooldown latch.
   * - ``DRIVING → AWAKE``
     - ``controlstate != CS_DRIVING``
     - Clears ready, speed, gear, and env temperatures.
   * - ``CHARGING → AWAKE``
     - ``controlstate != CS_CHARGING``
     - Clears env temperature; sets ``ms_v_charge_inprogress = false``.

Cooldown latch
==============

To prevent flapping after the 5-minute forced-sleep watchdog,
``HandleAwakeState`` sets ``m_allow_wake = false`` and records
``m_sleep_entry_time`` before transitioning to ``SLEEP``. While the
latch is held:

* ``IncomingFrameCan2`` does **not** call ``SetAwake(true)``, so trailing
  post-shutdown CAN traffic cannot bounce the driver straight back to
  ``AWAKE``.
* After 10 seconds, ``HandleSleepState`` clears the latch and CAN frames
  can wake the driver again.

The 12 V-based wake path is **not** gated by ``m_allow_wake`` — high aux
voltage will pull the driver out of ``SLEEP`` even mid-cooldown.

Boot
====

The constructor calls ``TransitionToSleepState()`` and
``PollSetThrottling(0)``. The driver always starts in ``SLEEP`` and waits
for CAN activity or a 12 V bump before doing anything else.

Notes and quirks
================

* **Two views of "vehicle on".** ``ms_v_env_awake`` (anything on the bus)
  drives ``SLEEP ↔ AWAKE``; ``controlstate`` (vehicle-reported mode)
  drives ``AWAKE ↔ DRIVING/CHARGING``. Future wake/sleep tweaks should
  preserve this split.
* **``env_awake`` is never explicitly cleared.** It falls to ``false``
  only via auto-stale plus the manual reset in ``ResetStaleMetrics``.
  Anything that changes its auto-stale period changes the ``SLEEP``
  detection latency.
* **Direct ``DRIVING ↔ CHARGING`` is impossible.** A vehicle that flips
  control mode from drive to charge spends at least one tick in
  ``AWAKE`` in between, which clears trip metrics.
* **The 12 V wake threshold depends on a calibrated reference.**
  ``HandleSleepState`` compares against ``ms_v_bat_12v_voltage_ref + 0.2``;
  on an uncalibrated module the CAN-frame path is the reliable wake
  mechanism.
* **Driving-state exit clears more than charging-state exit.**
  ``HandleDrivingState`` clears speed, gear, and temperatures;
  ``HandleChargingState`` only clears ``charge_inprogress`` and the env
  temperature. Charging metrics added in the future may need explicit
  clearing on exit.
