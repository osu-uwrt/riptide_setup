# UWRT Dependencies
# Install ROS-related dependencies
sudo apt-get -y install ros-kinetic-pointgrey-camera-driver 
sudo apt-get -y install ros-kinetic-control-toolbox
sudo apt-get -y install ros-kinetic-joy

# Install Ceres
sh install_ceres.sh

# Install Acoustics Dependencies
sh install_acoustics.sh

echo "Installed all UWRT dependencies"