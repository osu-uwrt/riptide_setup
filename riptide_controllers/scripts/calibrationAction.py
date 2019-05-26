#! /usr/bin/env python

import rospy
import actionlib
import dynamic_reconfigure.client

from riptide_msgs.msg import DepthCommand, AttitudeCommand
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
import riptide_controllers.msg

import time
import math
import yaml

class CalibrationAction(object):

    def __init__(self):
        self.depthPub = rospy.Publisher("/command/depth", DepthCommand, queue_size=1)  
        self.attitudePub = rospy.Publisher("/command/attitude", AttitudeCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer("calibration_action", riptide_controllers.msg.CalibrationAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

      
    def execute_cb(self, goal):
        client = dynamic_reconfigure.client.Client("thruster_controller", timeout=30)

        with open(rospy.get_param('~properties_file'), 'r') as stream:
            mass = yaml.safe_load(stream)['properties']['mass']

        client.update_configuration({"Buoyant_Force": mass * 9.81, "Buoyancy_X_POS": 0, "Buoyancy_Y_POS": 0, "Buoyancy_Z_POS": 0})

        self.depthPub.publish(True, 1)

        rospy.sleep(2)
        for i in range(1,40)
            force = rospy.wait_for_message("/command/force_depth", Vector3Stamped)
            

        # for i in range(1, 10):
        #     time.sleep(6)
        #     force = rospy.wait_for_message("/command/force_depth", Vector3)
        #     self.fobPub.publish(math.sqrt(force.x**2 + force.y**2 + force.z**2))
        
        self._as.set_succeeded()

        
        
if __name__ == '__main__':
    rospy.init_node('calibration')
    server = CalibrationAction()
    rospy.spin()