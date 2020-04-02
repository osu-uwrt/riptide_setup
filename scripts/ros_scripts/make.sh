~/osu-uwrt/riptide_software/src/riptide_utilities/xfer.sh $1
ssh -t ros@$1 'bash -ic "cd ~/osu-uwrt/riptide_software && catkin build"'
ssh ros@$1 'chmod 700 ~/osu-uwrt/riptide_software/src/riptide_utilities/*'