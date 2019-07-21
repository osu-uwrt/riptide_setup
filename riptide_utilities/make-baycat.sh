#ssh ros@riptide 'rm -rf ~/osu-uwrt/riptide_software/src'
#rsync -tvrz ~/osu-uwrt/riptide_software/src/riptide_bringup ros@jetson:~/osu-uwrt/riptide_software/src
#rsync -tvrz ~/osu-uwrt/riptide_software/src/riptide_hardware ros@jetson:~/osu-uwrt/riptide_software/src
#rsync -tvrz ~/osu-uwrt/riptide_software/src/darknet_ros ros@jetson:~/osu-uwrt/riptide_software/src
#rsync -tvrz ~/osu-uwrt/riptide_software/src/riptide_vision ros@jetson:~/osu-uwrt/riptide_software/src
#rsync -tvrz ~/osu-uwrt/riptide_software/src/riptide_msgs ros@jetson:~/osu-uwrt/riptide_software/src
#rsync -tvrz ~/osu-uwrt/riptide_software/src/riptide_utilities ros@jetson:~/osu-uwrt/riptide_software/src

~/osu-uwrt/riptide_software/src/riptide_utilities/xfer-baycat.sh
ssh ros@baycat 'cd ~/osu-uwrt/riptide_software && source /opt/ros/kinetic/setup.bash && catkin_make && source ~/osu-uwrt/riptide_software/devel/setup.bash'
ssh ros@baycat 'chmod 700 ~/osu-uwrt/riptide_software/src/riptide_utilities/*'
