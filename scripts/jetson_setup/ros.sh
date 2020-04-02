sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install -y ros-kinetic-ros-base
sudo apt-get install -y python-rosdep
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install -y python-rosinstall
sudo apt-get install -y ros-kinetic-urdf ros-kinetic-diagnostic-updater
sudo apt-get install -y ros-kinetic-roslint
sudo apt-get install -y ros-kinetic-gazebo-ros
sudo apt-get install -y ros-kinetic-cv-bridge
sudo apt-get install -y ros-kinetic-control-toolbox
sudo apt-get install -y ros-kinetic-xacro
