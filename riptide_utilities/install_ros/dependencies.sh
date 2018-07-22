# ROS Dependencies
sudo apt-get -y install ros-kinetic-pointgrey-camera-driver 
sudo apt-get -y install ros-kinetic-control-toolbox
sudo apt-get -y install ros-kinetic-joy
# Ceres install
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