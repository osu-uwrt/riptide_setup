#!/usr/bin/env python
# qualification_vision.py
# Subscribes to camera output, publishes what it sees.

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from riptide_vision import RiptideVision
from riptide_msgs.msg import GateData
from geometry_msgs.msg import Vector3

class ImageProcessor:
    def __init__(self):
        self.image_pub = rospy.Publisher("/forward/processed/compressed", CompressedImage, queue_size=1)
        self.data_pub = rospy.Publisher("/state/vision/gate", GateData, queue_size=1)
        self.fwd_sub = rospy.Subscriber("/forward/image_raw", Image, self.image_callback, queue_size=1)
        self.bridge = CvBridge()

    def image_callback(self, data):
        good = False
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            good = True
        except CvBridgeError as e:
            print(e)

        if good:
            response = RiptideVision().detect_gate(cv_image)
            hud_img = RiptideVision().detect_gate_vis(cv_image, response)
            img = CompressedImage()
            img.header.stamp = rospy.Time.now()
            img.format = ".jpeg"
            img.data = RiptideVision().compressed_img_msg_data(".jpeg", hud_img)
            self.image_pub.publish(img)

            msg = GateData()
            msg.object_data.header.stamp = rospy.Time.now()
            msg.object_data.visible = False
            if (len(response) > 0):
                pos = Vector3()
                pos.x = 0
                pos.y = response[3]
                pos.z = response[4]
                msg.left_pole_visible = response[1]
                msg.right_pole_visible = response[2]
                msg.object_data.visible = True
                msg.object_data.relative_position = pos

            self.data_pub.publish(msg)


def main():
    rospy.init_node('qualification_vision')
    ip = ImageProcessor()
    rospy.spin()

if __name__ == "__main__":
    main()
