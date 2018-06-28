#!/usr/bin/env python
# oculus.py
# Subscribes to camera output, publishes data about what it sees.
# Determines what to look for based on what is being subscribed to.

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from riptide_vision import RiptideVision
from gate_processor import GateProcessor
from pole_processor import PoleProcessor
from riptide_msgs.msg import TaskAlignment, BoundingBox
from geometry_msgs.msg import Point
import time

class Oculus:
    # Class constants
    SHAKE_THRESHOLD = 20    # Allowable amount of difference between positions
    MAX_SAMPLES = 5         # Number of previous positions to store for averaging
    DEBUG = True           # Setting to true will publish processed images on debug topic
    MODE_NONE = -1
    MODE_GATE = 0           # Detect gate mode
    MODE_POLE = 1           # Detect pole mode

    def __init__(self):
        self.image_pub = rospy.Publisher("/forward/processed/compressed", CompressedImage, queue_size=1)
        self.alignment_pub = rospy.Publisher("/task/gate/alignment", TaskAlignment, queue_size=1)
        self.fwd_sub = rospy.Subscriber("/forward/image_raw", Image, self.image_callback, queue_size=1)
        self.bridge = CvBridge()
        self.prev_pos = list()
        self.mode = self.MODE_NONE
        self.gate_processor = GateProcessor()
        self.pole_processor = PoleProcessor()

    def update_mode(self, mode, topic=None):
        self.alignment_pub.unregister()
        if (mode is not self.MODE_NONE):
            self.alignment_pub = rospy.Publisher(topic, TaskAlignment, queue_size=1)
            print "Publishing on " + topic + "."
        self.mode = mode
        print "Switched to mode " + str(mode)

    # Called whenever a camera frame is availale.
    def image_callback(self, data):
        # Convert image message to something OpenCV can deal with
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        pos = None
        bbox = None

        # Process the image based on which topic is being subscribed to
        # Set the object data pub to publish on the correct topic
        # Use mode to avoid creating a new publisher each time
        if (self.gate_processor.IsConnected()):
            if (self.mode != self.MODE_GATE):
                self.update_mode(self.MODE_GATE, "task/gate/alignment")
            t = time.time()
            pos, bbox = self.gate_processor.Process(cv_image, self.image_pub)
        elif (self.pole_processor.IsConnected()):
            if (self.mode != self.MODE_POLE):
                self.update_mode(self.MODE_POLE, "task/pole/alignment")
            pos, bbox = self.pole_processor.Process(cv_image, self.image_pub)
        else:
            if self.mode is not self.MODE_NONE:
                self.update_mode(self.MODE_NONE)
                self.reset_processor()

        if self.mode is not self.MODE_NONE:
            self.process_alignment_data(pos, bbox)

    # Function: reset_processor
    # Parameters:
    #   self
    # Description:
    #   Deletes any stored information in the processor to allow it to switch
    #   into a different processing mode with a clean slate
    def reset_processor(self):
        del self.prev_pos[:]

    # Function: pos_is_valid
    # Parameters:
    #   self
    #   pos: Position to check
    # Description:
    #   Returns whether or not a position is within the given
    #   SHAKE_THRESHOLD. Prevents sporatic false positives from skewing
    #   the average position.
    def pos_is_valid(self, pos):
        x = True
        y = True
        z = True
        if (len(self.prev_pos) > 0):
            x = abs(self.prev_pos[0].x - pos.x) < self.SHAKE_THRESHOLD
            y = abs(self.prev_pos[0].y - pos.y) < self.SHAKE_THRESHOLD
            z = abs(self.prev_pos[0].z - pos.z) < self.SHAKE_THRESHOLD
        return x and y and z

    # Function: get_new_average_pos
    # Parameters:
    #   self
    #   new_pos: Position to be added to the average
    # Description:
    #   Returns an average position of *new_pos* and the previous
    #   *MAX_SAMPLES* positions
    def get_new_average_pos(self, new_pos):
        avg_pos = Point()
        length = len(self.prev_pos)

        if (length == self.MAX_SAMPLES):
            self.prev_pos.pop()

        self.prev_pos.insert(0, new_pos)
        length += 1

        xt = 0
        yt = 0
        zt = 0
        for p in self.prev_pos:
            xt += p.x
            yt += p.y
            zt += p.z

        avg_pos.x = xt / length
        avg_pos.y = yt / length
        avg_pos.z = zt / length

        return avg_pos

    # Function: process_object_data
    # Parameters:
    #   self
    #   pos: Position of the object
    # Description:
    #   Publishes an object data message using the *alignment_pub*.
    #   *pos* is used to generate a new average position that is added to
    #   the message.
    def process_alignment_data(self, pos, bbox):
        align_msg = TaskAlignment()
        align_msg.header.stamp = rospy.Time.now() # Timestamp

        # Check if we saw the object
        # If yes, add the new position to the average and publish
        # If no, set visible to false and publish
        if (pos is not None):
            align_msg.visible = True
            if (self.pos_is_valid(pos)):
                align_msg.relative_pos = self.get_new_average_pos(pos)
                if bbox is not None:
                    align_msg.bbox = bbox
            else:
                self.reset_processor()
        else:
            align_msg.visible = False

        self.alignment_pub.publish(align_msg)

def main():
    rospy.init_node('oculus')
    oc = Oculus()
    rospy.spin()

if __name__ == "__main__":
    main()
