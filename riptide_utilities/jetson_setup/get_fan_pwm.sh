#!/bin/sh
# Place this script in
while [ 1 ] ; do
  cur_fan_pwm=`cat /sys/kernel/debug/tegra_fan/cur_pwm`
  target_fan_pwm=`cat /sys/kernel/debug/tegra_fan/target_pwm`
  echo "$cur_fan_pwm" > "/home/ros/ssd/fan_pwm"
  echo "$target_fan_pwm" > "/home/ros/ssd/fan_target_pwm"
  sleep 10
done
