date_seconds=$(date +"%s")
date_string=$(date -u --date=@$date_seconds)
echo "$date_string"
ssh ros@$1 "sudo date --set=\"$date_string\""
ssh ros@$1 "sudo hwclock --systohc"

# To change robot clock / hw clock without passwords:
#sudo visudo
#copy this line to bottom:
#ros ALL=(ALL) NOPASSWD: /bin/date, /sbin/hwclock
