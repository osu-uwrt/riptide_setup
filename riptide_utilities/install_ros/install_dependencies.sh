# UWRT Dependencies
# Install ROS-related dependencies
sudo apt-get -y install ros-kinetic-pointgrey-camera-driver 
sudo apt-get -y install ros-kinetic-control-toolbox
sudo apt-get -y install ros-kinetic-joy

# Install Ceres
sudo apt-get -y install cmake
sudo apt-get -y install libgoogle-glog-dev
sudo apt-get -y install libatlas-base-dev
sudo apt-get -y install libeigen3-dev
sudo apt-get -y install libsuitesparse-dev
mkdir ~/ceres-temp
cd ~/ceres-temp
wget http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
tar zxf ceres-solver-1.14.0.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver-1.14.0
make -j3
sudo make install
rm -rf ~/ceres-temp

# Setup Acoustics
tar -zxf ~/osu-uwrt/riptide_software/src/riptide_hardware/resources/acoustics/FrontPanel-Ubuntu16.04LTS-x64-5.0.1.tgz
mv FrontPanel-Ubuntu16.04LTS-x64-5.0.1 ~
cd ~/FrontPanel-Ubuntu16.04LTS-x64-5.0.1
chmod +x install.sh
sudo install.sh
sudo cp API/libokFrontPanel.so /usr/lib/
rm -rf ~/FrontPanel-Ubuntu16.04LTS-x64-5.0.1

echo "Completed installation of UWRT dependencies\n"