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
            pos = Point()
            pos.x = 0
            pos.y = response[2]
            pos.z = response[3]

            bbox = BoundingBox()
            bbox.top_left = Point(0, response[4], response[6])
            bbox.bottom_right = Point(0, response[5], response[7])

            pole_msg.roll_correction = response[0]
            pole_msg.beam_thickness = response[1]

        if debug_pub is not None:
            # Compress debug image
            debug_img = RiptideVision().detect_pole_vis(image, response)
            self.publish_debug_image(debug_img, debug_pub)

        # Publish data
        self.task_pub.publish(pole_msg)

        return pos, bbox
