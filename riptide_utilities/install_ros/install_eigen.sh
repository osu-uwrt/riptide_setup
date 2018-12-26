# Install Eigen
eigen_version="3.3.7"
wget --no-check-certificate https://bitbucket.org/eigen/eigen/get/$eigen_version.tar.bz2
mkdir ~/eg
tar -xf $eigen_version.tar.bz2
mv eig* eigen
cd eigen
mkdir build_dir
cd build_dir
cmake ..
sudo make install
cd ..
rm -rf eigen $eigen_version.tar.bz2
echo "Installed Eigen"

