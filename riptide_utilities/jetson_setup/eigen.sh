cd ~
wget --no-check-certificate https://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2
mkdir ~/eg
tar -xf 3.3.4.tar.bz2 -C eg
cd ~/eg/
mv eig* eigen
mkdir ~/eg/eigen/build_dir
cd ~/eg/eigen/build_dir
cmake ~/eg/eigen
sudo make install
rm -rf ~/eg ~/3.3.4.tar.bz2
