#ssh ros@riptide 'rm -rf ~/osu-uwrt/riptide_software/src'
#rsync -tvrz ~/osu-uwrt/riptide_software/src ros@riptide:~/osu-uwrt/riptide_software

~/osu-uwrt/riptide_software/src/riptide_utilities/xfer-riptide.sh
ssh ros@riptide 'cd ~/osu-uwrt/riptide_software && source /opt/ros/kinetic/setup.bash && catkin_make && source ~/osu-uwrt/riptide_software/devel/setup.bash'
ssh ros@riptide 'chmod 700 ~/osu-uwrt/riptide_software/src/riptide_utilities/*.sh'
