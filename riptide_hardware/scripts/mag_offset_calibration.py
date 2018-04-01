#!/usr/bin/env python

import rospy
import csv
from riptide_msgs.msg import PwmStamped
from imu_3dm_gx4.msg import MagFieldCF

magX = 0
magY = 0
magZ = 0
magTotal = 0
pwm_msg = PwmStamped()

def magCallback(data):
    magX = data.mag_field_components.x
    magY = data.mag_field_components.y
    magZ = data.mag_field_components.z
    magTotal = data.mag_field_magnitude

def pwmFillMsg(pwm):
    pwm_msg.header = rospy.Time.now()
    pwm_msg.pwm.surge_port_hi = 1500
    pwm_msg.pwm.surge_stbd_hi = 1500
    pwm_msg.pwm.surge_port_lo = pwm
    pwm_msg.pwm.surge_stbd_lo = 1500
    pwm_msg.pwm.sway_fwd = 1500
    pwm_msg.pwm.sway_fwd = 1500
    pwm_msg.pwm.heave_port_fwd = 1500
    pwm_msg.pwm.heave_stbd_fwd = 1500
    pwm_msg.pwm.heave_port_aft = 1500
    pwm_msg.pwm.heave_stbd_aft = 1500

def main():
    rospy.init_node('mag_offset_calibration')
    pwmPub = rospy.Publisher('command/pwm', PwmStamped)
    magSub = rospy.Subscriber("/imu/imu_magnetic_field", MagFieldCF, magCallback)

    rate = rospy.Rate(100)
    done = False
    pwm = 1500
    increment = 25
    low = 1300
    high = 1700

    while not rospy.is_shutdown():
        with open('test.csv', 'a') as csvfile:
            testWriter = csv.writer(csvfile)

            while done == False:
                #Do three ramps (up, down, up)
                for ramp in range(0,3):
                    while pwm <= high and pwm >= low:
                        pwmFillMsg(pwm)
                        pwmPub.publish(pwm_msg)
                        rospy.sleep(2.0)

                        #Get mag values 5 times
                        for i in range(0, 5):
                            testWriter.writerow([pwm, magX, magY, magZ, magTotal])
                            rospy.sleep(0.5)

                        #If testing is done (ramp = 2, and pwm back at 1500), break
                        if ramp == 2 and pwm == 1500:
                            done = True
                            break

                        #Increment or decrement pwm
                        pwm = pwm + (-1)**ramp * increment
                    if ramp == 0:
                        pwm = high - increment #Start going down
                    elif ramp == 1:
                        pwm = low + increment #Start going back up

        rate.sleep()

if __name__ == "__main__":
    main()
