===========
Toyota bZ4X
===========

The Toyota bZ4X is built on the :doc:`Toyota e-TNGA platform </components/vehicle_toyota_etnga/docs/index>`.
The e-TNGA module provides the CAN polling, charging state machine, and the common support baseline shared by
all e-TNGA vehicles. This page documents only what is specific to the bZ4X; see the e-TNGA module for
everything else.

Supported model years: 2023 and 2024.

----------------
Vehicle identity
----------------

:Short type code: TOYBZ4X
:Vehicle name: Toyota bZ4X
:Log tag: ``v-toyota-bz4x``
:Config namespace prefix: ``xte`` (inherited from e-TNGA base; bZ4X registers no namespace of its own)

------------------------
Vehicle-specific support
------------------------

The bZ4X currently adds no behavioural overrides on top of the e-TNGA platform, so its supported functions
match the e-TNGA baseline. In particular it does not declare a BMS pack arrangement, so per-cell BMS
voltage/temperature monitoring is not enabled — unlike the
:doc:`Subaru Solterra </components/vehicle_subaru_solterra/docs/index>`.

Web UI
------

All e-TNGA web UI pages are available on the bZ4X; see the
:doc:`e-TNGA index </components/vehicle_toyota_etnga/docs/index>` for the
full list.  Note that ``/bms/cellmon`` will appear in the menu but shows
no cell data because the bZ4X does not declare a BMS pack arrangement.
The charging pages (``/xte/charge``, ``/xte/reports``) and the TPMS
configuration page (``/xte/config``) work normally.
