#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import LinearCommand
from std_msgs.msg import String, Float64, Int8, Int32
from darknet_ros_msgs.msg import BoundingBoxes
import riptide_controllers.msg
import riptide_autonomy.msg
from actionTools import *

import time

class GarlicTaskAction(object):

    def __init__(self):
        self.dropperPub = rospy.Publisher("/command/drop", Int8, queue_size=1)
        self._as = actionlib.SimpleActionServer(
            "garlic_task", riptide_autonomy.msg.GarlicTaskAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        task_obj = ""
        confidence_bat = 0.0
        confidence_wolf = 0.0
        detection = False
        # Wait until you see the object a few times
        while not detection:
            boxes = rospy.wait_for_message("/state/bboxes", BoundingBoxes)
            for a in boxes.bounding_boxes:
                if a.Class == "Bat":
                    confidence_bat = a.probability
                    self.detection = True
                if a.Class == "Wolf":
                    confidence_wolf = a.probability
                    self.detection = True

            if self._as.is_preempt_requested():
                rospy.loginfo('Preempted Garlic Task')
                self._as.set_preempted()
                return

        if confidence_bat > confidence_wolf:
            task_obj = "Bat"
        else:
            task_obj = "Wolf"
        
        rospy.loginfo("Found object %s", task_obj)
        alignAction(task_obj, 0.5).wait_for_result()
        moveAction(0, -0.15).wait_for_result()

        #self.dropperPub.publish(0)
        #rospy.sleep(2.0)
        #self.dropperPub.publish(1)
        #rospy.sleep(2.0)

        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('garlic_task')
    server = GarlicTaskAction()
    rospy.spin()
