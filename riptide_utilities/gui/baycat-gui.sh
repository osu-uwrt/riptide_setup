#ssh ros@riptide 'rm -rf ~/osu-uwrt/riptide_software/src'
#rsync -tvrz ~/osu-uwrt/riptide_software/src ros@riptide:~/osu-uwrt/riptide_software

ssh ros@baycat 'bash -i ~/osu-uwrt/riptide_software/src/riptide_utilities/roscore.sh' &

export ROS_MASTER_URI=http://baycat:11311
rosrun rqt_gui rqt_gui
