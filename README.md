The Riptide AUV Software Platform
=================================

This repository is the main codebase for the Underwater Robotics Team at The Ohio State University. Our mission is to develop the software which powers our Autonomous Underwater Vehicles. The software in this repository handles low level controls, computer vision, mission-specific programs, and everything in between. All built on the Robot Operating System framework.

# Building
ROS is compiled using the catkin build system, and so all of our repos will use catkin. 

## Cloning riptide_software
To collaborate with the riptide_software platform, you must fork this repo (click "Fork" at the top right of this page). When executing the commands below, you will need to enter the URL to your forked repo. Form YOUR forked repo, click "Clone or download" beneath the "Fork" button, copy the URL, and then insert that URL in place of "<your_forked_repo>".
```
mkdir -p ~/osu-uwrt/riptide_software/
cd ~/osu-uwrt/riptide_software/
git clone <your_forked_repo> src
catkin_make
```
As a word of caution, you MUST be inside the folder ~/osu-uwrt/riptide_software to run the command "catkin_make"

## Setting up Git Remotes
Since you just cloned your fork to your computer, your remote called "origin" will point to your fork. Now, create a new remote that points to this main repo.
```
cd ~/osu-uwrt/riptide_software/src
git remote add upstream https://github.com/osu-uwrt/riptide_software.git
```

Now, if you type:
```
git remote -v
```
You will see both a remote to your fork and to the main repo. You will use these two remotes a lot when pushing code to your fork, submitting pull-requrests, and pulling down the latest code.

**The Underwater Robotics Team**  
The Ohio State University

[Website](https://uwrt.engineering.osu.edu) | [RoboSub](https://www.auvsifoundation.org/competition/robosub) | [License](LICENSE)


![OSU UWRT Logo](logos/UWRT_Logo_small.png)
