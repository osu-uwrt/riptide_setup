Electronic Installation
=======================

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
