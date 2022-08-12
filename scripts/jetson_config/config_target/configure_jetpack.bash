#!/bin/bash

echo "Updating fan control"
# comes from https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance/JetsonXavierNxSeriesAndJetsonAgxXavierSeries.html?highlight=fan#fan-profile-control
sudo systemctl stop nvfancontrol
sudo sed -i.bak -e 's/FAN_DEFAULT_PROFILE.*/FAN_DEFAULT_PROFILE cool/' /etc/nvfancontrol.conf
sudo rm /var/lib/nvfancontrol/status
sudo systemctl start nvfancontrol

echo "Disabling desktop"
sudo systemctl set-default multi-user.target

echo "Launching nmtui to configure hostname and static IP"
sudo nmtui