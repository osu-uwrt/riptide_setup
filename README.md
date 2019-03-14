The Riptide AUV Software Platform
=================================

This repository is the main codebase for the Underwater Robotics Team at The Ohio State University. Our mission is to develop the software which powers our autonomous underwater vehicles (AUVs) that compete in the Association for Unmanned Vehicle Systems International (AUVSI) [RoboSub](https://www.auvsifoundation.org/competition/robosub) Competition in San Diego, CA. The software in this repository is built on [Robot Operating System](http://www.ros.org/) and handles low level controls, computer vision, mission-specific programs, and everything in between.

Supported Operating Systems: Ubuntu 16.04

**The Underwater Robotics Team**  
The Ohio State University

[Website](https://uwrt.engineering.osu.edu) | [License](LICENSE)


![OSU UWRT Logo](logos/UWRT_Logo_small.png)


# Initial Setup

## Cloning riptide_software
To collaborate with the `riptide_software` platform, you must fork this repo (click "Fork" at the top-right of this page). When executing the commands below, you will need to enter the URL to your forked repo. From YOUR forked repo, click "Clone or download" at the top-right of the page, copy the URL, and then insert that URL in place of `<your_forked_repo>`. Do NOT forget the `src` at the end of the last line. This is a catkin-specific requirement that all source code be placed within a folder called `src`

NOTE: It is common to see brackets such as `<>` to act as placeholders for actual code. Make sure you replace the ENTIRE phrase `<your_forked_repo>` with the URL.
```
mkdir -p ~/osu-uwrt/riptide_software/
cd ~/osu-uwrt/riptide_software/
git clone <your_forked_repo> src
```

## Setting up Git Remotes
Since you just cloned your fork to your computer, your remote called `origin` will point to your fork. Now, create a new remote called `upstream` that points to this main repo.
```
cd ~/osu-uwrt/riptide_software/src/
git remote add upstream https://github.com/osu-uwrt/riptide_software.git
```

Now, if you type:
```
git remote -v
```
You will see both a remote to your fork and to the main repo. You will use these two remotes a lot when pushing code to your fork, submitting pull-requests, and pulling the latest code.

## Installing ROS and/or Dependencies
The `riptide_software` base currently uses ROS Kinetic Kame and is dependent on various ROS packages and other libraries. We created the [riptide_dependencies](https://github.com/osu-uwrt/riptide_dependencies) repository for the sole purpose of containing everything you need to install on your computer so you can use the `riptide_software` platform. Please go to our `riptide_dependencies` repo and follow the necessary instructions to install the required libraries onto your computer.

Once you have installed all of our dependencies, you will need to run one final script to finish setting up the UWRT environment:
```
cd ~/osu-uwrt/riptide_software/src/riptide_utilities/setup/
./setup_uwrt_env.sh
```
If everything compiled successfully, then you're all set to start coding. If you wish to contribute towards our other repositories than can be used in conjunction with `riptide_software`, then read through the section "Working with Our Other Repositories". It is recommended that you read through the section "Sourcing", as this is an important feature of catkin workspaces.

# Sourcing
There are a number of environment variables (mostly pertaining to ROS features and package paths) that are required when compiling/running our code. Each time you want to run one of those commands in a terminal, those environment variables will need to be set. This process is called "sourcing", and it is specific to working with catkin workspaces. 

To automate this process, we add various commands to the `bashrc` file indicating what needs to be sourced. As a note, bashrc = Born Again Shell Run-Commands. Each time a new terminal/shell is opened, it executes any commands within this file.

To edit/open the `bashrc` file, use the terminal text editor, nano (this is a hidden file, hence the "."):
```
nano ~/.bashrc
```
To exit and save your changes, press CTRL-X, type "y", then press ENTER. For the changes to take effect, close and re-open all terminals. If you are lazy and don't want to close your terminals, then you will need to source the `bashrc` file in each terminal that you have open:
```
source ~/.bashrc
```

# Working with Our Other Repositories
UWRT has a few other repositories that can be used in conjunction with this one.
1. [sim_software](https://github.com/osu-uwrt/sim_software)
2. [shared_software](https://github.com/osu-uwrt/shared_software)

## Chaining Workspaces
Each of the above repositories (sim_softare, shared_software) are built on catkin as well. For the entire set of repositories to work together, we must "chain" their `setup.bash` files within the `bashrc` file for complete functionality (in essence, we must source these `setup.bash` files in a specific order).

Below is a complete section of code that must be placed within the `bashrc` file for ALL of our working workspaces to be chained properly. The lines pertaining to `sim_software` and `shared_software` can be commented/omitted if the user decides not to use them. The lines pertaining to `/opt/ros/kinetic` and `riptide_software` must remain.
```
source /opt/ros/kinetic/setup.bash
source ~/osu-uwrt/sim_software/devel/setup.bash
source ~/osu-uwrt/shared_software/devel/setup.bash
source ~/osu-uwrt/riptide_software/devel/setup.bash
```

# Building riptide_software
ROS is compiled using the catkin build system, and so all of our repos will use catkin.

## Compiling
To compile this repo, you simply execute the `catkin_make` command from a terminal. As a word of caution, you MUST be inside the folder `~/osu-uwrt/riptide_software` to run `catkin_make`
```
cd ~/osu-uwrt/riptide_software/
catkin_make
```

In the near future, you will have to clone and compile the [control_toolbox](https://github.com/osu-uwrt/control_toolbox) because this repo will be dependent on it.

# Riptide Software Hierarchy

Below is a flow chart relating all major ROS nodes in our software base broken down by system level. Boxes with sharp corners represent physical objects (hardware, actuators), boxes with rounded corners represent individual ROS nodes, and the arrows connecting represent the direction of data communication via ROS topics. Note: only the major topics are displayed, otherwise the diagram would be too cluttered.

![riptide_software_flow_chart](diagrams/Riptide_Software_Stack_Full.png)

## Primary Packages
Below is a list of the primary packages we use to run our vehicle. Feel free to click on each package to learn more about its functionality.
### [riptide_msgs](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_msgs)
This package only contains custom ROS messages used throughout riptide packages.
### [riptide_hardware](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_hardware)
This package handles the hardware-software interface for working with all of our vehicle's sensors (data collection, data processing, etc.).
### [riptide_controllers](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_controllers)
This package handles all-things controls regarding movement of the vehicle, using a system of decoupled PID controllers.
### [riptide_vision](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_vision)
This package contains vision processing algorithms, such as using OpenCV to extract additional features from our camera footage, or to further process the output from darknet_ros.
### [riptide_bringup](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_bringup)
This package only contains a series of launch files used to "bring-up" our vehicle. Ex. launch a mission, launch our PS3 controller, etc.
### [riptide_autonomy](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_autonomy)
This package contains the RoboSub competition-specific task code. Our ultimate goal is to have a semi-autonomous system (since full autonomoy is too hard right now).

## Miscellaneous Packages
### [riptide_utilities](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_utilities)
This is actually not a catkin package. This is a utility folder specific for UWRT software members for interfacing our computers with ROS and our vehicle's computer system.
### [riptide_teleop](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_teleop)
This package contains the code to control the vehicle via PS3 controller.
### [riptide_description](https://github.com/osu-uwrt/riptide_software/tree/master/riptide_description)
This package contains the URDF files so we can build our vehicle in the Gazebo simulator.

