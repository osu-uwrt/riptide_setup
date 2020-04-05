# Delete logs that take a lot of space

sudo rm -f /var/log/syslog
sudo rm -f /var/log/syslog.1
sudo rm -f /var/log/kern.log
sudo rm -f /var/log/kern.log.1

sudo shutdown -h now
