# Riptide Software

## The Riptide AUV Software Platform

This repository serves as a setup utility for the Underwater Robotics Team at The Ohio State University. Our mission is to develop the software which powers our autonomous underwater vehicles \(AUVs\) that compete in the Association for Unmanned Vehicle Systems International \(AUVSI\) [RoboSub](https://robonation.org/programs/robosub/) Competition in San Diego, CA. The AUV software is built on [Robot Operating System 2](http://www.ros.org/) and handles low level controls, computer vision, mission-specific programs, and everything in between.

Supported Operating Systems: Ubuntu 22.04

**The Underwater Robotics Team**  
The Ohio State University

[Website](http://uwrt.club/) \| [License](https://github.com/osu-uwrt/riptide_software/tree/fac98cfa750df74dbb107f83064c3767e6346cc4/LICENSE/README.md)

![OSU UWRT Logo](logos/UWRT_logo_small.png)

## Initial Setup

### Cloning riptide\_software

NOTE: It is common to see brackets such as `<>` to act as placeholders for actual code. Make sure you replace the ENTIRE phrase `<SOMETHING_HERE>` with the requested info.

```text
mkdir -p ~/osu-uwrt/
cd ~/osu-uwrt/
git clone https://github.com/osu-uwrt/riptide_software.git
```

### Installing ROS and/or Dependencies

The `riptide_software` base currently uses ROS2 Humble Hawksbill and is dependent on various ROS packages and other libraries. Once you have cloned this repository run the following command to set everything up.
```
./setup

```
During setup, the script will prompt the user for multiple options including the following:
* desktop vs base ROS installation, (default: desktop) for a normal install press enter
* release installation, (default: no) for a normal install press enter, yes is for deploying to the vehicle
* simulator installation, (default no) for a no-simulation install press enter

## Sourcing

There are a number of environment variables \(mostly pertaining to ROS features and package paths\) that are required when compiling/running our code. Each time you want to run one of those commands in a terminal, those environment variables will need to be set. This process is called "sourcing", and it is specific to working with catkin workspaces.

To automate this process, we add various commands to the `bashrc` file indicating what needs to be sourced. As a note, bashrc = Born Again Shell Run-Commands. Each time a new terminal/shell is opened, it executes any commands within this file.

To edit/open the `bashrc` file, use the terminal text editor, nano \(this is a hidden file, hence the "."\):

```text
nano ~/.bashrc
```

To exit and save your changes, press CTRL-X, type "y", then press ENTER. For the changes to take effect, close and re-open all terminals. If you are lazy and don't want to close your terminals, then you will need to source the `bashrc` file in each terminal that you have open:

```text
source ~/.bashrc
```

### Chaining Workspaces

Each of the above repositories are built on colcon as well. For the entire set of repositories to work together, we must "chain" their `setup.bash` files within the `bashrc` file for complete functionality \(in essence, we must source these `setup.bash` files in a specific order\).

Below is a complete section of code that must be placed within the `bashrc` file for ALL of our working workspaces to be chained properly. The lines pertaining to `software` and `dependencies` can be commented/omitted if the user decides not to use them. The lines pertaining to `/opt/ros/humble` must remain.

```text
source /opt/ros/humble/setup.bash
source ~/osu-uwrt/development/software/install/setup.bash
source ~/osu-uwrt/development/dependencies/install/setup.bash
```

## Building riptide\_software

ROS2 is compiled using the colcon build system, and so all of our repos will use colcon. The build command is  `colcon build` while your terminal is in the correct directory

## Riptide Software Hierarchy

Below is a flow chart relating all major ROS nodes in our software base broken down by system level. Boxes with sharp corners represent physical objects \(hardware, actuators\), boxes with rounded corners represent individual ROS nodes, and the arrows connecting represent the direction of data communication via ROS topics. Note: only the major topics are displayed, otherwise the diagram would be too cluttered.

![riptide\_software\_flow\_chart](.gitbook/assets/riptide_software_stack_full.png)

### Primary Packages

Below is a list of the primary packages we use to run our vehicle. Feel free to click on each package to learn more about its functionality.

#### [riptide\_msgs](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_msgs)

This package only contains custom ROS messages used throughout riptide packages.

#### [riptide\_hardware](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_hardware)

This package handles the hardware-software interface for working with all of our vehicle's sensors \(data collection, data processing, etc.\).

#### [riptide\_controllers](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_controllers)

This package handles all-things controls regarding movement of the vehicle, using a system of decoupled PID controllers.

#### [riptide\_vision](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_vision)

This package contains vision processing algorithms, such as using OpenCV to extract additional features from our camera footage, or to further process the output from darknet\_ros.

#### [riptide\_bringup](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_bringup)

This package only contains a series of launch files used to "bring-up" our vehicle. Ex. launch a mission, launch our PS3 controller, etc.

#### [riptide\_autonomy](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_autonomy)

This package contains the RoboSub competition-specific task code. Our ultimate goal is to have a semi-autonomous system \(since full autonomoy is too hard right now\).

### Miscellaneous Packages

#### [riptide\_utilities](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_utilities)

This is actually not a catkin package. This is a utility folder specific for UWRT software members for interfacing our computers with ROS and our vehicle's computer system.

#### [riptide\_teleop](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_teleop)

This package contains the code to control the vehicle via PS3 controller.

#### [riptide\_description](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_description)

This package contains the URDF files so we can build our vehicle in the Gazebo simulator.

