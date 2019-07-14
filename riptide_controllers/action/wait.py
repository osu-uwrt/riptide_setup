#! /usr/bin/env python
import rospy
import actionlib
import dynamic_reconfigure.client

from std_msgs import string, int32
from darknet_ros_msgs.msg import BoundingBoxes

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
            boxes = rospy.wait_for_message("/state/bboxes", BoundingBoxes)
            for a in boxes.bounding_boxes:
                if a.Class == goal.object:
                    if (time.time() - lastTime) < 0.3:
                        count += 1
                    else:
                        count = 0
                lastTime = time.time()
                    
        rospy.loginfo("Found object %s", goal.object)


if __name__ == '__main__':
    rospy.init_node('wait')
    server = WaitAction()
    rospy.spin()
