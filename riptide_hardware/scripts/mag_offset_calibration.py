#!/usr/bin/env python

import rospy
import csv
import sys
from riptide_msgs.msg import PwmStamped
from imu_3dm_gx4.msg import MagFieldCF

magX = 0
magY = 0
magZ = 0
magTotal = 0
pwm_msg = PwmStamped()

def magCallback(data):
    global magX, magY, magZ, magTotal
    magX = data.mag_field_components.x
    magY = data.mag_field_components.y
    magZ = data.mag_field_components.z
    magTotal = data.mag_field_magnitude

def pwmFillMsg(pwm):
    pwm_msg.header.stamp = rospy.Time.now()
    pwm_msg.pwm.surge_port_hi = 1500
    pwm_msg.pwm.surge_stbd_hi = 1500
    pwm_msg.pwm.surge_port_lo = pwm
    pwm_msg.pwm.surge_stbd_lo = 1500
    pwm_msg.pwm.sway_fwd = 1500
    pwm_msg.pwm.sway_aft = 1500
    pwm_msg.pwm.heave_port_fwd = 1500
    pwm_msg.pwm.heave_stbd_fwd = 1500
    pwm_msg.pwm.heave_port_aft = 1500
    pwm_msg.pwm.heave_stbd_aft = 1500

def writeToFile(filename, pwm):
    with open(filename, 'a+') as file:
        file.write('%i, %f, %f, %f, %f\n' %(pwm, magX, magY, magZ, magTotal))
        print("Wrote values to file")
        print(pwm)
    file.close()

def main():
    rospy.init_node('mag_offset_calibration')
    pwmPub = rospy.Publisher('command/pwm', PwmStamped, queue_size=10)
    magSub = rospy.Subscriber("/imu/magnetic_field", MagFieldCF, magCallback)

    rate = rospy.Rate(100)
    state = 'pubPWM'
    pwm = 1500
    increment = 25
    low = 1300
    high = 1700
    filename = "/home/ros/osu-uwrt/riptide_software/src/riptide_hardware/cfg/test.csv"
    time = rospy.Time.now().to_sec()
    count = 0
    ramp = 0

    while not rospy.is_shutdown():
        #Do three ramps (up, down, up)
        #print(state)
        if state == 'pubPWM':
            pwmFillMsg(pwm)
            pwmPub.publish(pwm_msg)
            if rospy.Time.now().to_sec() - time > 4.0:
                state = 'writeValues'
                time = rospy.Time.now().to_sec()
        elif state == 'writeValues':
            #Write 40 values to file, obain readings at rate of 10 Hz
            if count < 40 and (rospy.Time.now().to_sec() - time) >= 0.1:
                writeToFile(filename, pwm)
                time = rospy.Time.now().to_sec()
                count = count + 1
            elif count == 40:
                state = 'setPWM'
                count = 0
        elif state == 'setPWM':
            #Increment pwm while within range.
            #If outside range, adjust according to the next ramp direction
            if pwm < high and pwm > low:
                pwm = pwm + (-1)**ramp * increment
            elif ramp == 0:
                pwm = high - increment #Start going down
                ramp = ramp + 1
            elif ramp == 1:
                pwm = low + increment #Start going back up
                ramp = ramp + 1
            state = 'pubPWM'
            if ramp == 2 and pwm == 1525:
                state = 'done'
            time = rospy.Time.now().to_sec()

        rate.sleep()

if __name__ == "__main__":
    main()
