#!/bin/sh

count=0
while [ 1 ]; do
  fan_thresh=200
  max_fan_speed=255

  cur_fan_pwm=`cat /sys/kernel/debug/tegra_fan/cur_pwm`
  target_fan_pwm=`cat /sys/kernel/debug/tegra_fan/target_pwm`

  if [ -e "/proc/uptime" ]; then
    uptime=`cat /proc/uptime | cut -d '.' -f1`
    echo "Current Fan pwm: $cur_fan_pwm\nTarget Fan pwm: $target_fan_pwm\nUptime: $((uptime))\nNum. times jetson_clocks ran: $count" > "/home/ros/ssd/fan_stats"

  	if [ $((uptime)) -gt 90 ]; then
      if [ $cur_fan_pwm -lt $fan_thresh -o $target_fan_pwm -lt $fan_thresh ]; then
        ./home/ros/ssd/jetson_clocks.sh
        count=$((count+1))
      fi
  	fi
  else
    echo "Current Fan pwm: $cur_fan_pwm\nTarget Fan pwm: $target_fan_pwm\nUptime: -1\nNum. times jetson_clocks ran: $count" > "/home/ros/ssd/fan_stats"
  fi
  sleep 5
done
