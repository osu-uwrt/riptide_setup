import rospy
from task_processor import TaskProcessor
from riptide_msgs.msg import GateData, BoundingBox
from riptide_vision import RiptideVision
from geometry_msgs.msg import Point

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
        response = RiptideVision().detect_gate(image)

        # Package data (if there is any)
        if (len(response) > 0):
            # pos = the middle of the gate in vehicle coordinate frame
            pos = Point()
            pos.x = 0
            pos.y = -response[7] # Cam frame x negated
            pos.z = -response[8] # Cam frame y negated

            bbox = BoundingBox()
            bbox.top_left = Point(0, -response[3], -response[4])
            bbox.bottom_right = Point(0, -response[5], -response[6])

            gate_msg.left_pole_visible = response[0]
            gate_msg.right_pole_visible = response[1]

        if debug_pub is not None:
            # Compress debug image
            debug_img = RiptideVision().detect_gate_vis(image, response)
            self.publish_debug_image(debug_img, debug_pub)

        # Publish data
        self.task_pub.publish(gate_msg)

        return pos, bbox
