#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import sys



def main():
    FILE = "/home/tsender/rosbags/roulette_7-7-18.avi"
    TOPIC = "/forward/image_undistorted"
    loop = False

    if len(sys.argv) >= 2:
        FILE = sys.argv[1]

    if len(sys.argv) == 3 and sys.argv[2] == '-l':
        loop = True

    rospy.init_node("publish_video")
    print "Opening " + FILE
    cap = cv2.VideoCapture(FILE)

    if not cap.isOpened():
        print "Could not open file. Exiting..."
        exit(0)

    print "Opened file. Publishing frames on " + TOPIC
    pub = rospy.Publisher(TOPIC, Image, queue_size=1)
    rate = rospy.Rate(30)
    bridge = CvBridge()
    while not rospy.is_shutdown():
        rval, frame = cap.read()
        if rval:
            msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(msg)
        elif loop:
            cap.release()
            cap = cv2.VideoCapture(FILE)
        else:
            rospy.signal_shutdown("End of video.")
        rate.sleep()


    cap.release()

if __name__ == "__main__":
    main()
