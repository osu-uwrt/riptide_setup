import rospy
from task_processor import TaskProcessor
from riptide_msgs.msg import PoleData, BoundingBox
from riptide_vision import RiptideVision
from geometry_msgs.msg import Point

# Class PoleProcessor
# Inherits from TaskProcessor
class PoleProcessor(TaskProcessor):
    def __init__(self):
        pole_pub = rospy.Publisher("/task/pole", PoleData, queue_size=1)
        TaskProcessor.__init__(self, pole_pub)

    # Normal Processing
    def Process(self, image, debug_pub=None):
        pole_msg = PoleData()
        pole_msg.roll_correction = 0
        pole_msg.beam_thickness = 0
        pos = None
        bbox = None

        # Process the image
        response = RiptideVision().detect_pole(image)

        # Package data (if there is any)
        if (len(response) > 0):
            x_min = response[2]
            y_min = response[3]
            x_max = response[4]
            y_max = response[5]
            x_mid = response[6]
            y_mid = response[7]
            cam_center_x = response[8]
            cam_center_y = response[9]

            # Adjust all X and Y positions to be relative to the center of the camera frame
            # Also maintain the axes convention in here (X is pos. to the right, Y is pos. down)
            x_min = x_min - cam_center_x
            x_max = x_max - cam_center_x
            x_mid = x_mid - cam_center_x

            y_min = y_min - cam_center_y
            y_max = y_max - cam_center_y
            y_mid = y_mid - cam_center_y

            # pos = the middle of the pole in vehicle coordinate frame
            pos = Point()
            pos.x = 0
            pos.y = -x_mid # Cam frame x negated
            pos.z = -y_mid # Cam frame y negated

            bbox = BoundingBox()
            bbox.top_left = Point(0, -x_min, -y_min)
            bbox.bottom_right = Point(0, -x_max, -y_max)

            pole_msg.roll_correction = response[0]
            pole_msg.beam_thickness = response[1]

        if debug_pub is not None:
            # Compress debug image
            debug_img = RiptideVision().detect_pole_vis(image, response)
            self.publish_debug_image(debug_img, debug_pub)

        # Publish data
        self.task_pub.publish(pole_msg)

        return pos, bbox
