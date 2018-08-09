#ssh ros@riptide 'rm -rf ~/osu-uwrt/riptide_software/src'
#rsync -tvrz ~/osu-uwrt/riptide_software/src ros@riptide:~/osu-uwrt/riptide_software

export ROS_MASTER_URI=http://localhost:11311
roscore &
rosrun rqt_gui rqt_gui


