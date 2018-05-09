ssh ros@riptide 'rm -rf /home/ros/osu-uwrt/src'
scp -r ~/osu-uwrt/src ros@riptide:/home/ros/osu-uwrt/
ssh ros@riptide 'cd /home/ros/osu-uwrt && source /opt/ros/kinetic/setup.bash && catkin_make'
