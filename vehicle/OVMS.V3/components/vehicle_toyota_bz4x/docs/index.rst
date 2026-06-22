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

The only behaviour the bZ4X adds on top of the e-TNGA platform is its BMS pack arrangement; everything
else matches the e-TNGA baseline. Being mechanically the Subaru Solterra's twin, it declares the same
96S CATL ("Type B") pack arrangement (96 cells in 4 stacks of 24; 24 temperature sensors, 6 per stack),
which enables per-cell BMS voltage/temperature monitoring via the standard ``/bms/cellmon`` page — just
like the :doc:`Subaru Solterra </components/vehicle_subaru_solterra/docs/index>`.

.. note::

   The per-cell BMS support has been validated on the Solterra but is **not yet validated on bZ4X
   hardware**. It assumes the bZ4X carries the same 96-cell pack; a variant with a differently-sized
   pack would report an incorrect cell count.

Web UI
------

All e-TNGA web UI pages are available on the bZ4X; see the
:doc:`e-TNGA index </components/vehicle_toyota_etnga/docs/index>` for the
full list.  The per-cell ``/bms/cellmon`` page is populated because the
bZ4X declares a BMS pack arrangement (see above).  The charging pages
(``/xte/charge``, ``/xte/reports``) and the TPMS configuration page
(``/xte/config``) work normally.
