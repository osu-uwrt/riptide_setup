Jumpers & Wiring
================

.. contents::
   :backlinks: top
   :local:

Wiring
------

USB3
----

- LORD MicroStrain IMU-3DM-GX4
- PointGrey BlackFly - Left
- PointGrey BlackFly - Right
- PointGrey BlackFly - Down

.. warning::
  Should further USB ports be required it will be necessary to connect them to header pins located somewhere on the motherboard. These are not USB3 capable?!


RS-232
------

- DB-9 Connector to something...
- DB-9 Connector to something else...
- Motherboard header to this...
- Motherboard header to that...




Jumpers
-------


JCMOS1: "Normal"

PSON1: "AT MODE"

JCASE1: "" <-- No connection.

JCOMPWR1: "+12V"
JCOMPWR2: "+12V"

JMSATASW1: "M-SATA"

JPSEL1: "DC-In: 12V"


BIOS
----

Press <ESC> or <DEL> to enter.


Boot Config
-----------

sudo nano /etc/default/grub

GRUB_CMDLINE_LINUX=”"

becomes:

GRUB_CMDLINE_LINUX=”text usbcore.usbfs_memory_mb=1000”

Uncomment:

#GRUB_TERMINAL=console

sudo update-grub
