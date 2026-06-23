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

The bZ4X adds no behavioural overrides on top of the e-TNGA platform; all of its support — including
per-cell BMS voltage/temperature monitoring — comes from the shared e-TNGA base, which owns the HV
pack arrangement for every e-TNGA vehicle and derives the actual cell/sensor counts from the battery
replies at runtime.

.. note::

   Per-cell BMS has been validated on the 96-cell Solterra. The e-TNGA base also supports the
   2025/26 refresh packs (78-cell FWD, 104-cell AWD) by deriving the count from the bus, but those
   are reasoned from spec and **not yet validated on bZ4X hardware**.

Web UI
------

All e-TNGA web UI pages are available on the bZ4X; see the
:doc:`e-TNGA index </components/vehicle_toyota_etnga/docs/index>` for the
full list.  The per-cell ``/bms/cellmon`` page is populated because the e-TNGA base declares a BMS
pack arrangement for all e-TNGA vehicles (see above).  The charging pages (``/xte/charge``, ``/xte/reports``) and the TPMS
configuration page (``/xte/config``) work normally.
