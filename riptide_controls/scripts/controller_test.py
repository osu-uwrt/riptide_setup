#!/usr/bin/env python

import rospy
from riptide_msgs.msg import ThrustStamped
from geometry_msgs.msg import Accel, Vector3
from riptide_controller import RiptideController


def main():
    controller = RiptideController()
    rospy.init_node('controller_test')
    rate = rospy.Rate(10)
    stop = 100
    i = 0
    while not rospy.is_shutdown() and i < stop:
        controller.translateX(-1)
        i += 1
        rate.sleep()

    controller.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
