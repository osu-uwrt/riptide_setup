#! /usr/bin/env python
import rospy
import actionlib

from riptide_msgs.msg import Dvl, LinearCommand
from geometry_msgs.msg import Vector3
from darknet_ros_msgs.msg import BoundingBoxes
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge, CvBridgeError
import riptide_controllers.msg

import math
import numpy as np
import cv2

bridge = CvBridge()

class GetDistance(object):

    _result = riptide_controllers.msg.GetDistanceResult()

    def __init__(self):
        self._as = actionlib.SimpleActionServer("get_distance", riptide_controllers.msg.GetDistanceAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

      
    def execute_cb(self, goal):
        rospy.loginfo("Finding distance to " + goal.object)
        self.image = None
        rospy.Subscriber("/stereo/disparity", DisparityImage, self.imgCB)

        readings = []
        while len(readings) < 5:
            bboxes = rospy.wait_for_message("/state/bboxes", BoundingBoxes).bounding_boxes
            while len([x for x in bboxes if x.Class == goal.object]) == 0 or self.image is None:
                bboxes = rospy.wait_for_message("/state/bboxes", BoundingBoxes).bounding_boxes

            bbox = [x for x in bboxes if x.Class == goal.object][0]

            sample_region = self.image[bbox.ymin:bbox.ymax, bbox.xmin:bbox.xmax].reshape((-1,1))
            
            # Define criteria = ( type, max_iter = 10 , epsilon = 1.0 )
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

            # Set flags (Just to avoid line break in the code)
            flags = cv2.KMEANS_RANDOM_CENTERS

            # Apply KMeans
            _,labels,centers = cv2.kmeans(sample_region,4,None,criteria,10,flags)

            labels = [l[0] for l in labels]
            maxLabel = max(set(labels), key=labels.count)
            disparity = centers[maxLabel][0]
            if disparity < 0:
                labels = [l for l in labels if l != maxLabel]
                maxLabel = max(set(labels), key=labels.count)
                disparity = centers[maxLabel][0]

            readings.append(self.f * self.T / disparity)

        self._result.distance =  np.median(readings)
        rospy.loginfo("Distance: %f"%self._result.distance)
        self._as.set_succeeded(self._result)

    def imgCB(self, msg):
        self.f = msg.f
        self.T = msg.T
        try:
            self.image = bridge.imgmsg_to_cv2(msg.image)
        except CvBridgeError as e:
            print(e)
        
        
if __name__ == '__main__':
    rospy.init_node('get_distance')
    server = GetDistance()
    rospy.spin()