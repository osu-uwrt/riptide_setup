#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import DepthCommand, AttitudeCommand, LinearCommand, Imu, Object, Depth
from std_msgs.msg import String, Int32, Float32, Float64
from darknet_ros_msgs.msg import BoundingBoxes
import riptide_controllers.msg

import time

class WaitAction(object):

    def __init__(self):
        self._as = actionlib.SimpleActionServer(
            "wait", riptide_controllers.msg.WaitAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("Waiting for object %s", goal.object)
        # Wait until you see the object a few times
        count = 0
        lastTime = time.time()
        while count < goal.times:
            try:
                boxes = rospy.wait_for_message("/state/bboxes", BoundingBoxes, timeout=1.0)
                for a in boxes.bounding_boxes:
                    if a.Class == goal.object:
                        if (time.time() - lastTime) < 0.3:
                            count += 1
                        else:
                            count = 0
                        lastTime = time.time()
            except:
                pass
            if self._as.is_preempt_requested():
                rospy.loginfo('Preempted Wait')
                self._as.set_preempted()
                return

        rospy.loginfo("Found object %s", goal.object)
        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('wait')
    server = WaitAction()
    rospy.spin()
