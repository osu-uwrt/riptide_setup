#!/bin/sh

while [ 1 ] ; do
  fan_thresh=200
  max_fan_speed=255

  if [ -e "/proc/uptime" ]; then
  	uptime=`cat /proc/uptime | awk '{printf "%0.f", $1}'` #cat /proc/uptime | cut -d '.' -f1`
    #printf "Uptime: $(uptime) sec\n"

  	if [ $((uptime)) -gt 90 ]; then
  		#printf "Uptime > 90 sec: $((uptime)) seconds!\n"
      cur_fan_pwm=`cat /sys/kernel/debug/tegra_fan/cur_pwm`
      if [ $cur_fan_pwm -lt $fan_thresh]; then
        sh /home/ros/ssd/jetson_clocks.sh
      fi
  	fi
  fi
done
