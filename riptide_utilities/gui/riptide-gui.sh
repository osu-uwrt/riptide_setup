#ssh ros@riptide 'rm -rf ~/osu-uwrt/riptide_software/src'
#rsync -tvrz ~/osu-uwrt/riptide_software/src ros@riptide:~/osu-uwrt/riptide_software

ssh ros@riptide 'roscore'

export ROS_MASTER_URI=http://riptide:11311
rosrun rqt_gui rqt_gui


