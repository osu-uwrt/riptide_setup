#!/usr/bin/env python
# qualification_vision.py
# Subscribes to camera output, publishes what it sees.

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from riptide_vision import RiptideVision
from riptide_msgs.msg import GateData, ObjectData
from geometry_msgs.msg import Vector3

class ImageProcessor:
    def __init__(self):
        self.image_pub = rospy.Publisher("/forward/processed/compressed", CompressedImage, queue_size=2)
        self.gate_pub = rospy.Publisher("/task/gate", GateData, queue_size=1)
        #self.gate_pub = rospy.Publisher("/state/vision/pole", PoleData, queue_size=1)
        self.obj_pub = rospy.Publisher("/task/gate/object_data", ObjectData, queue_size=1)
        self.fwd_sub = rospy.Subscriber("/forward/image_raw", Image, self.image_callback, queue_size=1)
        self.bridge = CvBridge()
        self.prev_pos = list();

    def image_callback(self, data):
        self.process_gate(data)

    def pos_is_valid(self, new_pos):
        threshold = 20 #px
        x = abs(self.prev_pos[0].x - new_pos.x) < threshold
        y = abs(self.prev_pos[0].y - new_pos.y) < threshold
        z = abs(self.prev_pos[0].z - new_pos.z) < threshold
        return x and y and z

    def process_gate(self, data):
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

            gate_msg = GateData()
            obj_msg = ObjectData()
            obj_msg.header.stamp = rospy.Time.now()
            obj_msg.visible = False
            if (len(response) > 0):
                pos = Vector3()
                pos.x = 0
                pos.y = response[2]
                pos.z = response[3]

                # Averaging stuff
                if (len(self.prev_pos) == 0 or pos_is_valid(pos)):
                    self.prev_pos.insert(0, pos)

                    if (len(self.prev_pos) == 5):
                        self.prev_pos.pop()

                    xt = 0
                    yt = 0
                    zt = 0
                    for p in self.prev_pos:
                        xt += p.x
                        yt += p.y
                        zt += p.z

                    avg_pos.x = xt / len(prev_pos)
                    avg_pos.y = yt / len(prev_pos)
                    avg_pos.z = zt / len(prev_pos)

                    # end averaging stuff

                    gate_msg.left_pole_visible = response[1]
                    gate_msg.right_pole_visible = response[2]
                    obj_msg.visible = True
                    obj_msg.rel_pos = avg_pos
            else:
                del self.prev_pos[:]

            if (len(self.prev_pos) == 5):
                self.gate_pub.publish(gate_msg)
                self.obj_pub.publish(obj_msg)


def main():
    rospy.init_node('qualification_vision')
    ip = ImageProcessor()
    rospy.spin()

if __name__ == "__main__":
    main()
