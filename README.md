The Riptide AUV Software Platform
=================================

This repository is the main codebase for the Underwater Robotics Team at The Ohio State University. Our mission is to develop the software which powers our Autonomous Underwater Vehicles. The software in this repository handles low level controls, computer vision, mission-specific programs, and everything in between. All built on the [Robot Operating System](http://www.ros.org/) framework, version Kinetic Kame.

**The Underwater Robotics Team**  
The Ohio State University

[Website](https://uwrt.engineering.osu.edu) | [RoboSub](https://www.auvsifoundation.org/competition/robosub) | [License](LICENSE)


![OSU UWRT Logo](logos/UWRT_Logo_small.png)


# Initial Setup

## Cloning riptide_software
To collaborate with the riptide_software platform, you must fork this repo (click "Fork" at the top-right of this page). When executing the commands below, you will need to enter the URL to your forked repo. Form YOUR forked repo, click "Clone or download" at the top-right of the page, copy the URL, and then insert that URL in place of "<your_forked_repo>". Do NOT forget the "src" at the end of the last line. This is a catkin-specific requirement that all source code be placed within a folder called "src".
```
mkdir -p ~/osu-uwrt/riptide_software/
cd ~/osu-uwrt/riptide_software/
git clone <your_forked_repo> src
```

## Setting up Git Remotes
Since you just cloned your fork to your computer, your remote called "origin" will point to your fork. Now, create a new remote called "upstream" that points to this main repo.
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
The "riptide_software" base currently uses ROS Kinetic Kame. You will need to have ROS installed on your Ubuntu machine, along with various other dependencies, to compile the code. But don't worry, we have a setup script that will install everything for you:
```
cd ~/osu-uwrt/riptide_software/src/riptide_utilities/install_ros
./setup_uwrt_env.sh
```

If you already have ROS installed, but our dependencies have changed, then all you need to do is run the dependencies script (do not re-run "setup_uwrt_env" because it will repeat lines written to the "bashrc" file):
```
cd ~/osu-uwrt/riptide_software/riptide_utilities/installation
./install_dependencies.sh
```

## The ~/.bashrc File
NOTE: This section is not required, since the setup script from above performs this action. However, it is good to verify this information is setup properly.

There are a number of environment variables (mostly pertaining to ROS features and package paths) that are required when running the code or when using the terminal. Each time you want to run one of those commands in a terminal, those environment variables will need to be set. To automate this process, we add a line to the "bashrc" file (bashrc = Born Again Shell Run-Commands) because each time a new terminal/shell is opened, it executes any commands within this file.

Open it with a terminal text editor (this is a hidden file, hence the "."):
```
nano ~/.bashrc
```
Scroll down to the bottom. Beneath the line
```
source /opt/ros/kinetic/setup.bash
```
add the line (if not already there):
```
source ~/osu-uwrt/riptide_software/devel/setup.bash
```
To exit and save your changes, press CTRL-X, type "y", then press ENTER. Close all terminals and then re-open for changes to take effect. If you are lazy and don't want to close your terminals, then each terminal will need to be "sourced" individually to setup its environment properly. Run the command:
```
source ~/.bashrc
```

# Working with Our Other Repositories
UWRT has a few other repositories that can be used in conjunction with this one.
1. [sim_software](https://github.com/osu-uwrt/sim_software)
2. [shared_software](https://github.com/osu-uwrt/shared_software)

## Chaining Workspaces
Each of the above repositories (sim_softare, shared_software) are built on catkin as well. For the entire set of repositories to work together, we must "chain" their setup.bash files within the "bashrc" file for complete functionality.

Below is a complete section of code that must be placed within the "bashrc" file for ALL of our working workspaces to be chained properly. The lines pertaining to "sim_software" and "shared_software" can be omitted if the user decides not to use them. All others must remain.
```
source /opt/ros/kinetic/setup.bash
source ~/osu-uwrt/sim_software/devel/setup.bash
source ~/osu-uwrt/shared_software/devel/setup.bash
source ~/osu-uwrt/riptide_software/devel/setup.bash
```

# Building riptide_software
ROS is compiled using the catkin build system, and so all of our repos will use catkin.

## Compiling
To compile this repo, you simply execute the "catkin_make" command from a terminal. As a word of caution, you MUST be inside the folder "~/osu-uwrt/riptide_software" to run "catkin_make"
```
cd ~/osu-uwrt/riptide_software/
catkin_make
```

In the near future, you will have to clone and compile the [control_toolbox](https://github.com/osu-uwrt/control_toolbox) because this repo will be dependent on it.
