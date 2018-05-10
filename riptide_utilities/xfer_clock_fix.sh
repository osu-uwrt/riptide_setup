rsync -tvrz ~/osu-uwrt/riptide_software/src ros@riptide:/home/ros/osu-uwrt/riptide_software
# Time stuff:

date_seconds=$(date +"%s")
echo "Setting RIPTIDE time to $date_seconds"
date_string=$(date --date=@$date_seconds)

# May not work, (sudo)
ssh ros@riptide 'sudo date --set="$date_string"'
ssh ros@riptide 'sudo hwclock --systohc'
# --

ssh ros@riptide 'cd /home/ros/osu-uwrt/riptide_software && source ~/osu-uwrt/riptide_software/devel/setup.bash && source /opt/ros/kinetic/setup.bash && catkin_make'

#sudo visudo
#copy this line to bottom:
#ros ALL=(ALL) NOPASSWD: /bin/date, /sbin/hwclock