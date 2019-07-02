#! /usr/bin/env python

import rospy

from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import numpy as np
import math

from task_processor import TaskProcessor
from riptide_msgs.msg import PoleData, BoundingBox, Object
from riptide_vision import RiptideVision
from geometry_msgs.msg import Point
from stereo_msgs.msg import DisparityImage
from sensor_msgs.msg import Image
from std_msgs.msg import Header



def imgCB(msg):
    global bridge

    start = time.time()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg.image)
    except CvBridgeError as e:
        print(e)

    (rows,cols) = cv_image.shape
        
    integral = cv2.integral(cv_image)


    score_img = np.zeros((1,cols,1), np.float32)
    x = 0
    maxScore = 0
    for c in range(cols):
        score = (integral[rows, c+1] - integral[rows,c])/rows
        if (score > maxScore):
            maxScore = score
            x = c
        elif score < 0:
            score = 0
        score_img[0,c] = score

    [mean, std] = cv2.meanStdDev(score_img)
    _, thresh = cv2.threshold(score_img, mean+2.5*std, 500000, cv2.THRESH_TOZERO)

    kernel = np.ones((1,15),np.float32)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    thresh = thresh / thresh.max() * 255
    thresh = thresh.astype(np.uint8)

    im2, contours,hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


    img2.publish(msg.image)

    if len(contours) != 0:
        #find the biggest area
        c = max(contours, key = cv2.contourArea)

        x,y,w,h = cv2.boundingRect(c)
        
        if w > 15:

            sample_region = cv_image[rows*1/4:rows*3/4, x:x+w].reshape((-1,1))
            
            # Define criteria = ( type, max_iter = 10 , epsilon = 1.0 )
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

            # Set flags (Just to avoid line break in the code)
            flags = cv2.KMEANS_RANDOM_CENTERS

            # Apply KMeans
            _,labels,centers = cv2.kmeans(sample_region,4,None,criteria,10,flags)

            labels = [x[0] for x in labels]
            maxLabel = max(set(labels), key=labels.count)
            disparity = centers[maxLabel][0]
            if disparity < 0:
                labels = [x for x in labels if x != maxLabel]
                maxLabel = max(set(labels), key=labels.count)
                disparity = centers[maxLabel][0]

            depth = msg.f * msg.T / disparity

            img.publish(bridge.cv2_to_imgmsg(thresh))
            head = Header()
            head.stamp = rospy.Time.now()
            center = Point(depth, (x+w/2) - cols/2, 0)
            pub.publish(head, "pole", w, rows, center)
            return
    
    # else
    img.publish(bridge.cv2_to_imgmsg(score_img))

    


    

rospy.init_node("pole_processor")
rospy.Subscriber("/stereo/disparity", DisparityImage, imgCB)
pub = rospy.Publisher("/state/object", Object, queue_size=5)
img = rospy.Publisher("/debug/pole", Image, queue_size=5)
img2 = rospy.Publisher("/debug/pole2", Image, queue_size=5)
bridge = CvBridge()
rospy.spin()

