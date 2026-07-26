.. highlight:: none

=========================
Client Mode Configuration
=========================

----------------
Using the Web UI
----------------

First you need to configure the SSID and passphrase for the known Wifi networks you would like to
connect the module to in Config → Wifi:

.. image:: wifi-1.png

(you can enter multiple Wifi networks)

Then you enable the client mode by choosing a Wifi mode of either "Client mode" or
"Access point + Client" (preferred) in Config → Autostart, field ``Wifi mode``:

.. image:: wifi-autostart-client-1.png

Finally you will choose between two behaviours for the module - in Config → Autostart,
field ``... client mode SSID``:

- choose the best one to connect to : ``Any known SSID (scan mode)``
- connect to only one of the previously defined Wifi networks ("normal mode")

.. image:: wifi-autostart-client-2.png


---------------
Using the shell
---------------

You can also configure all those parameters with the shell.

To configure the **SSID and passphrase** for the known Wifi networks, use the ``config set`` command::

  OVMS# config set wifi.ssid <ssid> <passphrase>

(you may use this command multiple time to enter multiple SSID information. You can check
the defined ones with ``config list wifi.ssid``)


To enable the **client mode**, you will need to chose ``client`` (or preferably ``apclient``) using
the ``config set`` command on the configuration item ``auto wifi.mode``::

  OVMS# config set auto wifi.mode client


To have the module connect to the best Wifi network and continuously monitor the signal quality
("scan mode"), you need to ensure that the configuration item ``auto wifi.ssid.client`` is empty::

  OVMS# config set auto wifi.ssid.client ""

While if you prefer that the module always connects to a specific Wifi network ("normal mode"), you
set the SSID of this network in the configuration item ``auto wifi.ssid.client``::

  OVMS# config set auto wifi.ssid.client "My Wifi Network"


.. warning:: All three steps (setting **SSID and passphrase** with ``config set wifi.ssid <ssid> <passphrase>``, enabling
  the **client mode** with ``config set auto wifi.mode client`` or ``config set auto wifi.mode apclient``, and choosing
  between "scan mode" or a specific Wifi Network with ``config set wifi.ssid.client <ssid>``) are necessary for proper operation
  of the client mode.

.. note:: This configuration of the Wifi mode will only be applied on the next reboot. To force a mode
  change immediately, you need to use the ``wifi mode`` command.
  If you want to use the "scan mode" (i.e. able to switch from one known SSID to another), you just
  enter the mode without any following parameter::

    wifi mode client

  While if you want that the module always connects to a specific Wifi network ("normal mode"), you
  add the SSID of this network just after the mode::

    wifi mode client "My Wifi Network"


----------------------
WiFi Priority Networks
----------------------

When using scan mode (multiple known SSIDs, no fixed client SSID), you can optionally tell the
module to prefer networks in a specific order and automatically upgrade to a more-preferred one
when it comes into range.

**What it does:** At connect time the module joins the highest-priority known network that is in
range. While connected to a lower-priority network it periodically rescans and switches to any
higher-priority network whose signal is at or above the good-signal threshold
(``network wifi.sq.good``). A brief disconnect occurs on each upgrade. No scanning is done while
on the highest-priority network.

**Enabling priority networks:**

Enable the feature and define the ordered list of SSIDs::

  OVMS# config set network wifi.priority.enable yes
  OVMS# config set network wifi.priority "home,hotspot,cafe"

Each SSID in the list must still have its password configured as usual::

  OVMS# config set wifi.ssid home <passphrase>
  OVMS# config set wifi.ssid hotspot <passphrase>

Optionally tune the upgrade-scan interval (default 60 seconds, minimum 10)::

  OVMS# config set network wifi.priority.interval 30

**Behaviour summary:**

- At connection time, the module joins the highest-priority in-range known network.
- While on a lower-priority network, the module scans every ``wifi.priority.interval`` seconds
  and switches to any higher-priority network that is in range with signal >=
  ``network wifi.sq.good``. Expect a brief disconnect on switch.
- No background scanning is done while already on the top-priority network.
- The feature is active only in scan mode (``auto wifi.ssid.client`` empty) with ``client`` or
  ``apclient`` WiFi mode.
- Networks with hidden SSIDs cannot participate in priority ordering.
- ``wifi status`` shows the current network's priority rank.

Configuring priority networks from the web UI
---------------------------------------------

The priority list can be managed on the **Config → Wifi** page, in the
*Wifi priority networks* section. Tick the networks to prioritise and use the
▲/▼ buttons to order them; rank 1 is preferred. Only networks already saved
under *Wifi client networks* can be selected, so an SSID cannot be mistyped.

Priority networks are inactive — regardless of the list — when any of these
apply. The web page warns about each:

- a fixed client SSID is configured on the Autostart page (the module then
  connects to that SSID only, instead of scanning),
- the Wifi mode is not *client* or *access point + client* (the mode defaults
  to *access point*, so priority is inactive until you change it),
- the priority list is empty.

A listed network with no saved password is skipped, and while priority
networks are enabled a saved network that is **not** on the list is never
joined. Because of this, open (passwordless) networks can never participate
in priority, even when listed.
