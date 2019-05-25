#! /usr/bin/env python

import rospy
import actionlib

from riptide_msgs.msg import DepthCommand, AttitudeCommand
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import riptide_controllers.msg

import time
import math

class CalibrationAction(object):

    def __init__(self):
        self.depthPub = rospy.Publisher("/command/depth", DepthCommand, queue_size=1)  
        self.attitudePub = rospy.Publisher("/command/attitude", AttitudeCommand, queue_size=1)  
        self.fobPub = rospy.Publisher("/state/fob", Float32, queue_size= 1)  
        self._as = actionlib.SimpleActionServer("calibration_action", riptide_controllers.msg.CalibrationAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

      
    def execute_cb(self, goal):
        self.depthPub.publish(True, 1)
        att = AttitudeCommand()
        att.roll_active = True
        att.pitch_active = True
        att.yaw_active = False
        att.x = 0
        att.y = 0
        self.attitudePub.publish(att)

        for i in range(1, 10):
            time.sleep(6)
            force = rospy.wait_for_message("/command/force_depth", Vector3)
            self.fobPub.publish(math.sqrt(force.x**2 + force.y**2 + force.z**2))
        
        self._as.set_succeeded()

        
        
if __name__ == '__main__':
    rospy.init_node('calibration')
    server = CalibrationAction()
    rospy.spin()