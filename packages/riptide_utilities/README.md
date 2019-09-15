---
description: riptide_utilities
---

# Utilities

This folder contains a variety of incredibly useful command/setup scripts for UWRT software members.

## Installation \(installation/\)

This folder contains the install scripts for setting up ROS and our dependencies on your computer. The primary install scripts are:

* `setup_uwrt_env.sh`: This script is meant to be used for first-time users who do not have anything setup on their computer. It will install ROS and all of the `riptide_software` dependencies. This script calls all files inside this folder.
* `setup_bashrc`: This script will add all necessary lines to the ~/.bashrc file on your computer so your terminal will be sourced properly.
* `install_dependencies`: This script will install all of the `riptide_software` dependencies.
* `install_ros_kinetic`: This script will install the full desktop version of ROS Kinetic Kame.
* `install_ceres`: This script installs Google's non-linear solver called ceres.
* `install_acoustics`: This script installs two libraries: one is required for the FPGA in our acoustics system, the other is a fast fourier transform \(FFT\) library.

## SSH Scripts \(ssh\_scripts/\)

This folder contains a few scripts that allow you to ssh \("secure shell" - this is you remote into another computer\) without having to type in the password for the remote computer each time. VERY HANDY.

## Date Scripts \(date\_scripts/\)

This folder contains a few files to set the time on our two computers: `riptide` and `jetson`

* Note: This only needs to be done if when pushing code to one of these computers, the code does not compile due to 1\) the time you made the file changes on your computer and 2\) the time indicated by `riptide` or `jetson`.
  * For example, if you changed a file at 2:55pm, but the "local" time on let's say `riptide` is 3:00pm, then `riptide` will not compile this file because changes in the "past" are unimportant.

## Jetson Setup \(jetson\_setup/\)

This folder contains a series of scripts that are used to install ROS and other dependencies on our NVIDIA Jetson TX1.

## VS Code \(vscode/\)

We use Visual Studio Code as our code editor because it is simple to use and has a lot of handy features. This folder contains two important files:

* `keybindings.json`: Open the default `keybindings.json` file and replace all contents with those from this one.
* `tasks.json`: Create a blank tasks file and replace its contents with those from this one.

## Transfer Scripts \(xfer\_scripts/\)

This folder contains a number of scripts that can either transfer or BOTH transfer and compile code from your computer to `riptide` and/or `jetson`.

## Shutdown Scripts \(shutdown\_scripts/\)

This folder contains a number of scripts that can remotely shutdown and/or reboot `riptide` and/or `jetson`.

## GUI \(gui/\)

This folder contains pre-defined layout of an rqt\_gui we often use when testing our vehicle at pool-tests. Import this gui after running: `rosrun rqt_gui rqt_gui`

