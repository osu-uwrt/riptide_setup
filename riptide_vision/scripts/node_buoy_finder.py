#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys, time

# numpy and scipy
import numpy as np
#from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import robosub

# Ros Messages
from sensor_msgs.msg import CompressedImage
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False

lower_green = np.array([40,45,45])
upper_green = np.array([90,255,255])

lower_red = np.array([170,70,70])
upper_red = np.array([179,255,255])


class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish

        # subscribed Topic
        self.subscriber = rospy.Subscriber("/stereo/left/image_color/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        
        self.image_pub = rospy.Publisher("/output/image_buoy_processed/compressed",
            CompressedImage)
        if VERBOSE :
            print "subscribed to /stereo/left/image_color"
        startTime = time.time()

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format
        print "Received image"
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        image_np = cv2.flip(image_np,0)

        overlay = image_np.copy()
        robosub.findColorBuoy(image_np.copy(), lower_green, upper_green, "Green",overlay)
        robosub.findColorBuoy(image_np.copy(), lower_red, upper_red, "Red",overlay)

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', overlay)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    print "created node"
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
