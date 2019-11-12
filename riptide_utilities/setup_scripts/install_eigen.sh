#!/bin/sh

# Install Eigen
eigen_version="3.3.7"
wget --no-check-certificate https://bitbucket.org/eigen/eigen/get/$eigen_version.tar.bz2
tar -xf $eigen_version.tar.bz2
mv eig* eigen
mkdir eigen/build_dir
cd eigen/build_dir
cmake ..
make install
cd ../..
rm -rf eigen/ $eigen_version.tar.bz2

echo "Installed Eigen"