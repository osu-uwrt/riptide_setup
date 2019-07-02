#! /usr/bin/env python
import rospy
import actionlib
import dynamic_reconfigure.client

from riptide_msgs.msg import DepthCommand, AttitudeCommand, Constants
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
import riptide_controllers.msg

import time
import math
import yaml

class CalibrateAction(object):

    def __init__(self):
        self.depthPub = rospy.Publisher("/command/depth", DepthCommand, queue_size=1)
        self.rollPub = rospy.Publisher("/command/roll", AttitudeCommand, queue_size=1)
        self.pitchPub = rospy.Publisher("/command/pitch", AttitudeCommand, queue_size=1)
        
        self._as = actionlib.SimpleActionServer("calibrate", riptide_controllers.msg.CalibrateAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

      
    def execute_cb(self, goal):
        client = dynamic_reconfigure.client.Client("thruster_controller", timeout=30)

        with open(rospy.get_param('~properties_file'), 'r') as stream:
            mass = yaml.safe_load(stream)['properties']['mass']

        rospy.loginfo("Starting calibration")

        Fb = mass * 9.81
        CobX = 0
        CobY = 0
        CobZ = 0

        client.update_configuration({"Buoyant_Force": Fb, "Buoyancy_X_POS": CobX, "Buoyancy_Y_POS": CobY, "Buoyancy_Z_POS": CobZ})

        self.depthPub.publish(True, .5)
        self.rollPub.publish(0, AttitudeCommand.POSITION)
        self.pitchPub.publish(0, AttitudeCommand.POSITION)

        rospy.sleep(3)

        # Recalibrate 10 times
        for _ in range(1, 10):
            rospy.sleep(3)

            # Average 10 samples
            forceSum = 0
            for _ in range(1,10):
                forceMsg = rospy.wait_for_message("/command/force_depth", Vector3Stamped).vector
                force = math.sqrt(forceMsg.x**2 + forceMsg.y**2 + forceMsg.z**2)

                if forceMsg.z < 0:
                    force *= -1
                forceSum += 0.1 * force

            # Adjust in the right direction
            Fb += force * 0.8
            client.update_configuration({"Buoyant_Force": Fb, "Buoyancy_X_POS": CobX, "Buoyancy_Y_POS": CobY, "Buoyancy_Z_POS": CobZ})

        rospy.loginfo("Buoyant force calibration complete")

        for _ in range(1, 10):
            rospy.sleep(3)

            CobYSum = 0
            CobXSum = 0
            for _ in range(1,10):
                momentMsg = rospy.wait_for_message("/command/moment", Vector3Stamped).vector
                CobYSum += momentMsg.x / Fb * 0.1
                CobXSum += momentMsg.y / Fb * 0.1

            CobY += CobYSum * 0.8
            CobX -= CobXSum * 0.8

            client.update_configuration({"Buoyant_Force": Fb, "Buoyancy_X_POS": CobX, "Buoyancy_Y_POS": CobY, "Buoyancy_Z_POS": CobZ})

        rospy.loginfo("Buoyancy XY calibration complete")

        self.rollPub.publish(45, AttitudeCommand.POSITION)

        rospy.sleep(3)

        for i in range(1, 10):
            rospy.sleep(3)

            CobZSum = 0
            for _ in range(1,10):
                momentMsg = rospy.wait_for_message("/command/moment", Vector3Stamped).vector
                CobZSum += momentMsg.x / Fb / math.sqrt(2) * 0.1

            CobZ -= CobZSum * 0.8

            client.update_configuration({"Buoyant_Force": Fb, "Buoyancy_X_POS": CobX, "Buoyancy_Y_POS": CobY, "Buoyancy_Z_POS": CobZ})


            
        rospy.loginfo("Calibration complete")

        self.depthPub.publish(False, 0)
        self.rollPub.publish(0, AttitudeCommand.MOMENT)
        self.pitchPub.publish(0, AttitudeCommand.MOMENT)
        
        self._as.set_succeeded()

        
        
if __name__ == '__main__':
    rospy.init_node('calibrate')
    server = CalibrateAction()
    rospy.spin()