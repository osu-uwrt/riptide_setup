# This Dockerfile starts from a base ubuntu 12.04 image and installs
# a full desktop ROS hydro.
#
# It also creates a catkin workspace for a single user named YOURNAME 
# with password YOURPASSWORD.
#
# GUI applications in the container will connect to the host's X11 server
# by mounting the .X11-unix folder on the host OS. I have no idea if this 
# works on Mac if Windows.
# (http://fabiorehm.com/blog/2014/09/11/running-gui-apps-with-docker/)
#
# Another alternative is to install a VNC server inside the container and 
# remote desktop into it. Yet another alternative is to ssh into the container
# with X11 forwarding. I haven't tried either those options.
#
# I use this ROS environment by running a terminal emulator, using it 
# exclusively to run ROS programs. The actual editing of source code 
# happens on my host OS. This requires exposing a folder to the container
# with the -v option in "docker run". 
# (https://docs.docker.com/userguide/dockervolumes/)

FROM ros:kinetic-ros-core-xenial

MAINTAINER Parth Parekh MAINTAINER Parth Parekh 

# install ros packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-base=1.3.2-0* \
&& rm -rf /var/lib/apt/lists/*


# Add universe repository 
#RUN echo "deb http://us.archive.ubuntu.com/ubuntu/ 16.04 main universe" >> /etc/apt/sources.list

# Install things needed to add ROS repository 

RUN apt-get update && apt-get install -y \
    apt-utils \
    wget \
    ros-kinetic-catkin\
    python-catkin-tools\
    python3-vcstool\
    git\
    ca-certificates

RUN useradd osu -m


# Initialize ROS 
RUN rosdep update

# Setup catkin workspace
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN bin/bash -c "source /opt/ros/kinetic/setup.bash && \
                 mkdir -p ~/osu-uwrt/riptide_software/src && \
                 cd ~/osu-uwrt/riptide_software/ && \
                 catkin init"              
                 

RUN bin/bash -c "source /opt/ros/kinetic/setup.bash && \
                 apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
                 apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
                 apt update && \
                 apt-get -y install python-rosinstall python-rosinstall-generator python-wstool build-essential && \
                 apt-get -y install ros-kinetic-control-toolbox && \
                 apt-get -y install ros-kinetic-joy && \
                 cd ~/osu-uwrt/riptide_software/ && \
                 rm -rf src && \
                 git clone https://github.com/juched/riptide_software.git src &&  \
                 cd src && \
                 git checkout autonomy && \
                 apt-get install libraw1394-11 libavcodec-ffmpeg56                    \
                 libavformat-ffmpeg56 libswscale-ffmpeg3 libswresample-ffmpeg1        \
		         libavutil-ffmpeg54 libgtkmm-2.4-dev libglademm-2.4-dev               \
		         libgtkglextmm-x11-1.2-dev libusb-1.0-0    && \ 
                 cd && \
                 git clone https://github.com/Juched/flycap-mirror.git && \
                 cd flycap-mirror && \
                 ./install_flycapture.sh && \
                 cd ~/osu-uwrt/riptide_software/src && \
                 git clone https://github.com/ros-drivers/pointgrey_camera_driver.git ros-kinetic-pointgrey-camera-driver && \
                 rosdep update && \
                 rosdep install --from-paths ~/osu-uwrt/riptide_software/src -i -r -y "




RUN bin/bash -c  "source /opt/ros/kinetic/setup.bash && \
                 mkdir ~/dependencies && \
                 cd ~/dependencies && \
                 cd ~/osu-uwrt/riptide_software/src/riptide_utilities/setup_scripts/ && \
                 apt-get -y install libx11-dev && \
                 ./install_imu_darknet.sh && \
                 ./install_eigen.sh && \
                 ./install_ceres.sh && \
                 ./install_rosdeps.sh && \
                 ./setup_uwrt_env.sh && \
                 cd ~/osu-uwrt/riptide_software/ && \
                 catkin_make"

RUN echo "source ~/osu-uwrt/devel/setup.bash" >> ~/.bashrc


# This is needed to run GUI applications
ENV DISPLAY localhost:0 

#############################################################################
######################### Graphics Card (Optional) ##########################
#############################################################################

# Install graphics card driver, if applicable
# Make sure the driver is the same version as the your host's
#ADD ./nvidia /tmp/nvidia
#RUN /tmp/nvidia/NVIDIA-Linux-x86_64-340.32.run -s -N --no-kernel-module

#############################################################################
############################ Other Custom Stuff #############################
#############################################################################

# Install terminal emulator and font of choice 
USER root 
RUN apt-get update && apt-get install -y \
    terminator 
RUN chsh -s /bin/bash osuuwrt
USER osuuwrt 


# Images generated from this should be run with two options:
#  -v /tmp/.X11-unix:/tmp/.X11-unix (to display on the host X server, this is
#                                    needed to run GUI applications)
#  --privileged                     (to access the graphics card)