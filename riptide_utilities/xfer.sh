ssh ros@riptide 'rm -rf /home/ros/osu-uwrt/riptide_software/src'
scp -r ~/osu-uwrt/riptide_software/src ros@riptide:/home/ros/osu-uwrt/riptide_software/src
ssh ros@riptide 'cd /home/ros/osu-uwrt/riptide_software && source /opt/ros/kinetic/setup.bash && catkin_make'
