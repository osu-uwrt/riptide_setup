wget http://snapshots.ros.org/kinetic/2019-05-10/ubuntu/pool/main/r/ros-kinetic-pointgrey-camera-driver/ros-kinetic-pointgrey-camera-driver_0.13.4-0xenial-20190320-183759-0800_amd64.deb
sudo dpkg -i ros-kinetic-pointgrey-camera-driver_0.13.4-0xenial-20190320-183759-0800_amd64.deb
sudo apt-get -f -y install
rm ros-kinetic-pointgrey-camera-driver_0.13.4-0xenial-20190320-183759-0800_amd64.deb

# Note: Only works on amd64 ubuntu 16.04. On all other versions you have to build
# https://github.com/ros-drivers/pointgrey_camera_driver.git and have flycapture installed first