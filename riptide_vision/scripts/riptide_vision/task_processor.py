import rospy
from riptide_vision import RiptideVision
from sensor_msgs.msg import CompressedImage

class TaskProcessor:
    def IsConnected(self):
        return self.task_pub.get_num_connections() > 0

    def __init__(self, task_pub):
        self.task_pub = task_pub

    def publish_debug_image(self, img, pub):
        img_msg = CompressedImage()
        img_msg.header.stamp = rospy.Time.now()
        img_msg.format = ".jpeg"
        img_msg.data = RiptideVision().compressed_img_msg_data(".jpeg", img)

        pub.publish(img_msg)
