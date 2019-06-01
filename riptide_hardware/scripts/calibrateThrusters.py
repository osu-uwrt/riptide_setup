#!/usr/bin/env python

import rospy
import serial

from riptide_msgs.msg import PwmStamped, Pwm
from std_msgs.msg import Float32MultiArray, Header


def main():
    global pwmPub
    global ard

    rospy.init_node("calibrate_thrusters")

    ard = serial.Serial('/dev/ttyACM0', 9600, timeout=5)
    pwmPub = rospy.Publisher('/command/pwm', PwmStamped, queue_size=1)
    
    pwmOffset = 40
    done = False

    while not done:
        done |= collect(pwmOffset)
        pwmPub.publish(Header(), Pwm(1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500))
        rospy.sleep(0.5)
        done |= collect(-pwmOffset)
        pwmPub.publish(Header(), Pwm(1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500))
        rospy.sleep(0.5)
        pwmOffset += 10


def collect(offset):

    pwmPub.publish(Header(), Pwm(1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 + offset))
    rospy.sleep(3)
    
    currentAvg = 0
    forceAvg = 0
    for i in range(10):
        currentAvg += rospy.wait_for_message("/state/thruster_currents", Float32MultiArray).data[0]
        force = None
        ard.flushInput()
        ard.readline()
        while force is None:
            try:
                force = float(ard.readline()[:-2])
            except:
                pass
        forceAvg += force

    currentAvg /= 10.0
    forceAvg /= 10.0

    rospy.loginfo(str(1500 + offset) + "," + str(forceAvg) + "," + str(currentAvg))
    return (forceAvg < -10) or (currentAvg > 8)

    


if __name__ == '__main__':
    main()