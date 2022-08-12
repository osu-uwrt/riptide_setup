#!/bin/bash

if [ "$EUID" -ne 0 ]; then
    echo "This script can oly be run as root"
    exit
fi

ARCH=$(dpkg --print-architecture)
if [ ! "$ARCH" == *"aarch64"* ]; then
    echo "Updating fan control"
    # comes from https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/JetsonXavierNxSeriesAndJetsonAgxXavierSeries.html?highlight=fan#fan-profile-control
    systemctl stop nvfancontrol
    sed -i.bak -e 's/FAN_DEFAULT_PROFILE.*/FAN_DEFAULT_PROFILE cool/' /etc/nvfancontrol.conf
    rm /var/lib/nvfancontrol/status
    systemctl start nvfancontrol

    echo "Disabling desktop"
    systemctl set-default multi-user.target

    echo "Launching nmtui to configure hostname and static IP"
    # nmtui
else
    echo "This script is not running on jetson hardware. Aborting"
    exit
fi