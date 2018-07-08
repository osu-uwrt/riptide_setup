#ssh ros@riptide 'rm -rf ~/osu-uwrt/riptide_software/src'
rsync -tvrz ~/osu-uwrt/riptide_software/src/riptide_bringup ros@jetson:~/osu-uwrt/riptide_software/src
rsync -tvrz ~/osu-uwrt/riptide_software/src/riptide_hardware ros@jetson:~/osu-uwrt/riptide_software/src
rsync -tvrz ~/osu-uwrt/riptide_software/src/imu_3dm_gx4 ros@jetson:~/osu-uwrt/riptide_software/src
rsync -tvrz ~/osu-uwrt/riptide_software/src/darknet_ros ros@jetson:~/osu-uwrt/riptide_software/src
rsync -tvrz ~/osu-uwrt/riptide_software/src/riptide_vision ros@jetson:~/osu-uwrt/riptide_software/src
rsync -tvrz ~/osu-uwrt/riptide_software/src/riptide_msgs ros@jetson:~/osu-uwrt/riptide_software/src
rsync -tvrz ~/osu-uwrt/riptide_software/src/riptide_utilities ros@jetson:~/osu-uwrt/riptide_software/src
ssh ros@jetson 'chmod 700 ~/osu-uwrt/riptide_software/src/riptide_utilities/*'
