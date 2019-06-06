#! /usr/bin/env python

import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2

from task_processor import TaskProcessor
from riptide_msgs.msg import PoleData, BoundingBox
from riptide_vision import RiptideVision
from geometry_msgs.msg import Point
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image



def imgCB(msg):
    global bridge


    try:
        cv_image = bridge.imgmsg_to_cv2(msg.image)
    except CvBridgeError as e:
        print(e)

    (rows,cols) = cv_image.shape
        

    x = 0
    maxScore = 0
    for c in range(cols):
        score = 0
        for r in range(rows):
            score += cv_image[r,c]
        if (score > maxScore):
            maxScore = score
            x = c

    cv2.line(cv_image, (x, rows), (x, 0), 255)

    try:
        pub.publish(bridge.cv2_to_imgmsg(cv_image))
    except CvBridgeError as e:
        print(e)

rospy.init_node("pole_processor")
rospy.Subscriber("/stereo/disparity", DisparityImage, imgCB)
pub = rospy.Publisher("/debug/pole", Image, queue_size=5)
bridge = CvBridge()
rospy.spin()

