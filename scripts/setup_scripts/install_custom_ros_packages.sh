#!/bin/bash


cd ~/osu-uwrt/
mkdir -p dependencies
cd dependencies

git -C MYNT pull || git clone https://github.com/slightech/MYNT-EYE-D-SDK.git MYNT
cd MYNT
make init
make ros
cd ..

mkdir src
cd src

vcs import < ~/osu-uwrt/riptide_setup/scripts/setup_scripts/dependencies.repos . --recursive

if [ -d ~/osu-uwrt/riptide_software/src/riptide_gazebo ] 
then
  echo "Downloading sim dependencies..."
  git -C uuv_simulator pull || git clone https://github.com/osu-uwrt/uuv_simulator.git
else
  echo "No riptide_gazebo found. Not downloading sim dependencies."
fi 

cd ..

rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y -r

catkin config --install --extend ~/osu-uwrt/dependencies/MYNT/wrappers/ros/devel --cmake-args -DCMAKE_BUILD_TYPE=Release


########################## Stuff for Groot (Behvaiortree's Graphical Editor)
# groot requires Cmake and QT5. should already have cmake. 
#Assigning default value as 1: Desktop full install
if [ ! -d "src/Groot/build" ]; then
	echo
	echo
	read -p "Would you like to install Groot [A graphical editor for task code] (Y/n)? " -n 1 answer 
	echo
	echo
	case "$answer" in
	y|Y|"" ) 
			sudo apt-get install qt5-default
			cd src/Groot
			git submodule update --init --recursive
			mkdir build
			cd build
			cmake ..
			make
			sudo cp Groot /usr/local/bin
			cd ../..
			
		;;
	esac
fi

# Set up dependencies for Cartographer
sudo apt-get install -y python3-wstool python3-rosdep ninja-build stow
sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp
src/cartographer/scripts/install_abseil.sh

catkin build



#Old install groot code (not needed?)
#sudo add-apt-repository ppa:beineri/opt-qt596-focal -y
#sudo apt-get update
#sudo apt-get install qt59base qt59svg -y


#cd src/Groot
#wget "https://github.com/probonopd/linuxdeployqt/releases/download/continuous/linuxdeployqt-continuous-x86_64.AppImage" -O ~/linuxdeployq.AppImage
#chmod +x ~/linuxdeployq.AppImage

#mkdir build
#cd build
#source /opt/qt59/bin/qt59-env.sh
#cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=install ..
#make -j$(nproc) install

#export VERSION=$(git describe --abbrev=0 --tags); echo $VERSION
#unset QTDIR; unset QT_PLUGIN_PATH ; unset LD_LIBRARY_PATH
#~/linuxdeployq.AppImage ./install/share/applications/Groot.desktop  -appimage
#cd ..
#cd ..



