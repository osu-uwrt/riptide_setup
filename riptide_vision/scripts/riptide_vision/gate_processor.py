import rospy
from task_processor import TaskProcessor
from riptide_msgs.msg import GateData, BoundingBox
from riptide_vision import RiptideVision
from geometry_msgs.msg import Point
import time
import lib

# Class GateProcessor
# Inherits from TaskProcessor
class GateProcessor(TaskProcessor):
    def __init__(self):
        gate_pub = rospy.Publisher("/task/gate", GateData, queue_size=1)
        TaskProcessor.__init__(self, gate_pub)

    # Normal Processing
    def Process(self, image, debug_pub=None):
        gate_msg = GateData()
        gate_msg.left_pole_visible = False
        gate_msg.right_pole_visible = False
        pos = None
        bbox = None

        # Process the image
        t = time.time()
        response, _ = lib.find_gate(image)
        print "'New:' Found gate in %2.2f" % (time.time() - t)


        # Package data (if there is any)
        if (len(response) > 0):
            x_min = response[3]
            y_min = response[4]
            x_max = response[5]
            y_max = response[6]
            x_center = response[9]
            y_center = response[10]
            cam_center_x = response[11]
            cam_center_y = response[12]

            # Adjust all X and Y positions to be relative to the center of the camera frame
            # Also maintain the axes convention in here (X is pos. to the right, Y is pos. down)
            x_min = x_min - cam_center_x
            x_max = x_max - cam_center_x
            x_center = x_center - cam_center_x

            y_min = y_min - cam_center_y
            y_max = y_max - cam_center_y
            y_center = y_center - cam_center_y

            # pos = the middle of the gate in vehicle coordinate frame
            pos = Point()
            pos.x = 0
            pos.y = -x_center # Cam frame x negated
            pos.z = -y_center # Cam frame y negated

            bbox = BoundingBox()
            bbox.top_left = Point(0, -x_min, -y_min)
            bbox.bottom_right = Point(0, -x_max, -y_max)

            gate_msg.left_pole_visible = response[0]
            gate_msg.right_pole_visible = response[1]

        if debug_pub is not None:
            # Compress debug image
            debug_img = RiptideVision().detect_gate_vis(image, response)
            self.publish_debug_image(debug_img, debug_pub)

        # Publish data
        self.task_pub.publish(gate_msg)

        return pos, bbox
