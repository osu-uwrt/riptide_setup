#!/bin/bash 

if [ $# -ge 1 ]; then
    ADDRESS=$1
else
    echo "This script needs one argument, ADDRESS"
    echo "A second argment can be added for a remote username other than ros"
    echo "./install_tar.bash <ADRRESS> <USERNAME>"
    exit
fi

if [ $# -ge 2 ]; then
    USERNAME=$2
    echo "login with remote user $USERNAME"
else
    USERNAME="ros"
fi

exit

date_seconds=$(date +"%s")
date_string=$(date -u --date=@$date_seconds)
echo "$date_string"
ssh $USERNAME@$1 "sudo date --set=\"$date_string\""
ssh $USERNAME@$1 "sudo hwclock --systohc"

# To change robot clock / hw clock without passwords:
#sudo visudo
#copy this line to bottom:
#ros ALL=(ALL) NOPASSWD: /bin/date, /sbin/hwclock
