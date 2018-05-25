#ssh ros@riptide 'rm -rf /home/ros/osu-uwrt/riptide_software/src'
rsync -tvrz ~/osu-uwrt/riptide_software/src ros@jetson:/home/ros/osu-uwrt/riptide_software
ssh ros@jetson 'cd /home/ros/osu-uwrt/riptide_software && source /opt/ros/kinetic/setup.bash && catkin_make && source ~/osu-uwrt/riptide_software/devel/setup.bash'
ssh ros@jetson 'chmod 700 /home/ros/osu-uwrt/riptide_software/src/riptide_utilities/*'
