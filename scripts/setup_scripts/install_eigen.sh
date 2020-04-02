#!/bin/bash

# Install Eigen
if [ ! -d "/usr/lib/cmake/eigen3" ]; then
    eigen_version="3.3.7"
    wget --no-check-certificate https://bitbucket.org/eigen/eigen/get/$eigen_version.tar.bz2
    tar -xf $eigen_version.tar.bz2
    mv eig* eigen
    mkdir eigen/build_dir
    cd eigen/build_dir
    cmake ..
    make install -j8
    cd ../..
    rm -rf eigen/ $eigen_version.tar.bz2

    echo "Installed Eigen"
else
    echo "Eigen is already installed"
fi