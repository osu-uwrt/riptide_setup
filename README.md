# Riptide Software

## The Riptide AUV Software Platform

This repository serves as a setup utility for the Underwater Robotics Team at The Ohio State University. Our mission is to develop the software which powers our autonomous underwater vehicles \(AUVs\) that compete in the Association for Unmanned Vehicle Systems International \(AUVSI\) [RoboSub](https://robonation.org/programs/robosub/) Competition in San Diego, CA. The AUV software is built on [Robot Operating System 2](http://www.ros.org/) and handles low level controls, computer vision, mission-specific programs, and everything in between.

Supported Operating Systems: Ubuntu 22.04

**The Underwater Robotics Team**  
The Ohio State University

[Website](https://org.osu.edu/osu-uwrt/) \| [License](https://github.com/osu-uwrt/riptide_software/tree/fac98cfa750df74dbb107f83064c3767e6346cc4/LICENSE/README.md)

![OSU UWRT Logo](logos/UWRT_Logo_small.png)

## Initial Setup

### Cloning riptide\_setup

NOTE: It is common to see brackets such as `<>` to act as placeholders for actual code. Make sure you replace the ENTIRE phrase `<SOMETHING_HERE>` with the requested info.

```text
mkdir -p ~/osu-uwrt/
cd ~/osu-uwrt/
git clone https://github.com/osu-uwrt/riptide_setup.git
```

### Installing ROS and/or Dependencies

The `riptide_software` base currently uses ROS2 Humble Hawksbill and is dependent on various ROS packages and other libraries. Once you have cloned this repository run the following command to set everything up.
```
~/osu-uwrt/riptide_setup/setup.bash
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