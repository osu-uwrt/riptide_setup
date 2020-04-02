~/osu-uwrt/riptide_setup/scripts/ros_scripts/xfer.sh $1
ssh -t ros@$1 'bash -ic "cd ~/osu-uwrt/riptide_software && catkin build"'