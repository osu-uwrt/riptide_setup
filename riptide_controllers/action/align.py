#!/usr/bin/env python

import rospy
import actionlib

from riptide_msgs.msg import AlignmentCommand
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Float64, Int32

import riptide_controllers.msg

import math
import time

def diff(a):
    if a > 20:
        return True

class AlignAction(object):

    cam_width = 644
    cam_height = 482

    bbox_x = 30
    bbox_y = 30
    bbox_z = 30

    found = False

    def __init__(self):
        self.alignPub = rospy.Publisher("/command/alignment", AlignmentCommand, queue_size=1)
        self._as = actionlib.SimpleActionServer("align", riptide_controllers.msg.AlignAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("Aligning to object %s", goal.object)

        self.alignPub.publish(goal.object, goal.width_ratio)  

        while diff(self.bbox_x) or diff(self.bbox_y) or diff(self.bbox_z):
            bbox_msg = rospy.wait_for_message("/state/bboxes", BoundingBoxes)
            for bbox in bbox_msg.bounding_box:
                if bbox.Class == goal.object:
                    self.bbox_x = (bbox.xmin + bbox.xmax) / 2 - self.cam_width / 2
                    self.bbox_y = (bbox.ymin + bbox.ymax) / 2 - self.cam_height / 2
                    self.bbox_z = (bbox.xmax - bbox.xmin) - self.cam_width * goal.width_ratio
    
        rospy.loginfo("Alignment succeed")
        self._as.set_succeeded()

if __name__ == '__main__':
    rospy.init_node('align')
    server = AlignAction()
    rospy.spin()