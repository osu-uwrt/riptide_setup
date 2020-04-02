#!/bin/sh

count=0
num=2
x=5
while [ 1 ]; do
  fan_thresh=200
  max_fan_speed=255

  if [ -e "/proc/uptime" ]; then
  	uptime=`cat /proc/uptime | cut -d '.' -f1` #`cat /proc/uptime` | awk '{printf "%0.f", $1}'` #cat /proc/uptime | cut -d '.' -f1`
    #printf "Uptime: $(uptime) sec\n"
    #cur_fan_pwm=`cat /sys/kernel/debug/tegra_fan/cur_pwm`
    #target_fan_pwm=`cat /sys/kernel/debug/tegra_fan/target_pwm`
    #echo "Current Fan pwm: $cur_fan_pwm\nTarget Fan pwm: $target_fan_pwm\nUptime: $((uptime))" > "/home/ros/ssd/fan_stats"
    #echo "$cur_fan_pwm" >> "/home/ros/ssd/fan_stats/fan_pwm"
    #echo "$target_fan_pwm" >> "/home/ros/ssd/fan_stats/fan_target_pwm"
    #echo "$((uptime))" >> "/home/ros/ssd/fan_stats/uptime"

  	if [ $((uptime)) -gt 10 ]; then
  		printf "Uptime: $((uptime)) seconds!\n"
      #if [ $cur_fan_pwm -lt $fan_thresh] || [$target_fan_pwm -lt $fan_thresh ]; then
      #  ./home/ros/ssd/jetson_clocks.sh
      #  echo "$(count)" > "/home/ros/ssd/count"
      #fi
      if [ $count -lt $num -o $num -lt $x ]; then
        echo "$count" > "/home/ros/ssd/count"
      fi
  	fi
  else
    echo "Current Fan pwm: $cur_fan_pwm\nTarget Fan pwm: $target_fan_pwm\nUptime: -1\nNum. times jetson_clocks ran: $count" > "/home/ros/ssd/fan_stats"
  fi
  sleep 5
  count=$((count+1))
done
