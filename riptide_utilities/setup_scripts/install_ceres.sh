#!/bin/sh

# Install Ceres
ceres_version="ceres-solver-1.14.0"
apt-get -y install cmake
apt-get -y install libgoogle-glog-dev
apt-get -y install libatlas-base-dev
apt-get -y install libeigen3-dev
apt-get -y install libsuitesparse-dev
wget http://ceres-solver.org/$ceres_version.tar.gz
tar zxf $ceres_version.tar.gz
rm $ceres_version.tar.gz
mkdir ceres-bin
cd ceres-bin
cmake ../$ceres_version
make -j3
make install
cd ..
rm -rf $ceres_version/ ceres-bin/

echo "Installed ceres"