#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image

currentCam = 0

def cameraCb(msg):
    cameraPub.publish(msg)

def cameraSelectionCb(msg):
    global cameraSub
    global currentCam

    if msg.data != currentCam:
        cameraSub.unregister()
        if msg.data == 0:
            cameraSub = rospy.Subscriber("/stereo/left/image_rect_color", Image, cameraCb)
        else:
            cameraSub = rospy.Subscriber("/downward/image_rect_color", Image, cameraCb)
        currentCam = msg.data

def bboxCb(msg):
    objects = {}
    for b in msg.bounding_boxes:
        if not objects.has_key(b.Class) or b.probability > objects[b.Class].probability:
            objects[b.Class] = b
    msg.bounding_boxes = objects.values()
    bboxPub.publish(msg)


if __name__ == '__main__':

    rospy.init_node("darknet_processor")

    # Set subscribers
    rospy.Subscriber("/command/camera", Int8, cameraSelectionCb)
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, bboxCb)
    cameraPub = rospy.Publisher("/darknet_ros/input_image", Image, queue_size=1)
    bboxPub = rospy.Publisher("/state/bboxes", BoundingBoxes, queue_size=1)
    cameraSub = rospy.Subscriber("/stereo/left/image_rect_color", Image, cameraCb)

    rospy.spin()
