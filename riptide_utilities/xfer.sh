scp -r ~/osu-uwrt/riptide_software/src ros@riptide:/home/ros/osu-uwrt/riptide_software/
ssh ros@riptide 'cd /home/ros/osu-uwrt/riptide_software && source /opt/ros/kinetic/setup.bash && catkin_make'
