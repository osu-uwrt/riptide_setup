#ssh ros@riptide 'rm -rf ~/osu-uwrt/riptide_software/src'
#rsync -tvrz ~/osu-uwrt/riptide_software/src ros@riptide:~/osu-uwrt/riptide_software

export ROS_MASTER_URI=http://$HOSTNAME:11311
roscore &
rqt


