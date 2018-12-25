The Riptide AUV Software Platform
=================================

This repository is the main codebase for the Underwater Robotics Team at The Ohio State University. Our mission is to develop the software which powers our Autonomous Underwater Vehicles. The software in this repository handles low level controls, computer vision, mission-specific programs, and everything in between. All built on the [Robot Operating System](http://www.ros.org/) framework.

**The Underwater Robotics Team**  
The Ohio State University

[Website](https://uwrt.engineering.osu.edu) | [RoboSub](https://www.auvsifoundation.org/competition/robosub) | [License](LICENSE)


![OSU UWRT Logo](logos/UWRT_Logo_small.png)

# Building
ROS is compiled using the catkin build system, and so all of our repos will use catkin. 

## Cloning
To collaborate with the riptide_software platform, you must fork this repo (click "Fork" at the top-right of this page). When executing the commands below, you will need to enter the URL to your forked repo. Form YOUR forked repo, click "Clone or download" at the top-right of the page, copy the URL, and then insert that URL in place of "<your_forked_repo>".
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
You will see both a remote to your fork and to the main repo. You will use these two remotes a lot when pushing code to your fork, submitting pull-requests, and pulling down the latest code.

## Compiling
To compile this repo, you simply execute the "catkin_make" command from a terminal. As a word of caution, you MUST be inside the folder "~/osu-uwrt/riptide_software" to run "catkin_make"
```
cd ~/osu-uwrt/riptide_software/
catkin_make
```

In the near future, you will have to clone and compile the [control_toolbox](https://github.com/osu-uwrt/control_toolbox) because this repo will be dependent on it.

### Updating the ~/.bashrc File
Because this repo is built on ROS, there are a number of environment variables that are required when running the code. Each time you open a terminal window to run those commands, those environment variables will need to be set. To automate this process, we add a line to the ~/.bashrc file (bashrc = Born Again Shell Run-Commands). Each time a new terminal/shell is opened, it executes any commands within the ~/.bashrc file.

Open it with a terminal text editor:
```
nano ~/.bashrc
```
Scroll down to the bottom. Beneath the line
```
source /opt/ros/kinetic/setup.bash
```
add the line:
```
source ~/osu-uwrt/riptide_software/devel/setup.bash
```
To exit and save your changes, press CTRL-X, type "y", then press ENTER. For these changes to take effect in the CURRENT terminal window we have to "source" it because the terminal was opened a while ago. Run the command:
```
source ~/.bashrc
```

You could also just close the terminal and open a new one, because opening a terminal will automatically run the ~/.bashrc file.

# Other Repositories
UWRT has a few other repositories that can be used in conjunction with this one.
1. [sim_software](https://github.com/osu-uwrt/sim_software)
2. [shared_software](https://github.com/osu-uwrt/shared_software)

## Chaining Workspaces
Each of the above repositories (sim_softare, shared_software) are built on catkin as well. For the entire set of repositories to work together, we must "chain" their setup.bash files within the ~/.bashrc file for complete functionality.

Below is the complete set of lines that must be placed within the ~/.bashrc file for all of these workspaces to be chained properly. Only the lines pertaining to "sim_software" and "shared_software" can be omitted if the user decides not to use them. All others must remain.
```
source /opt/ros/kinetic/setup.bash
source ~/osu-uwrt/sim_software/devel/setup.bash
source ~/osu-uwrt/shared_software/devel/setup.bash
source ~/osu-uwrt/control_toolbox/devel_isolated/setup.bash
source ~/osu-uwrt/riptide_software/devel/setup.bash
```
