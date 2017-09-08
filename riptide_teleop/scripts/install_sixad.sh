#!/bin/bash
# Script to install the PS3 controller driver sixad
# Based on https://needforbits.wordpress.com/2017/05/10/dualshock-3-controller-ps3-sixaxis-gamepad-on-ubuntu-16-04/

sudo apt-get install dialog build-essential pyqt4-dev-tools libusb-dev libbluetooth-dev python-dbus -y
wget https://github.com/RetroPie/sixad/archive/master.zip -O sixad-master.zip
unzip sixad-master.zip
cd sixad-master
make
sudo make install
cd ..
rm -rf sixad-master sixad-master.zip
echo Plugin the controller and run "sudo sixpair" to use the PS3 Controller

# After verifying sixpair success, unplug the controller and run sudo sixad -s.
# Leave that program running to maintain PS3 controller connection!
