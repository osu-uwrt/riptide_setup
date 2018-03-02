# qualification_vision.py
# Subscribes to camera output, publishes what it sees.

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from riptide_vision import RiptideVision

class ImageProcessor:
    def __init__(self):
        self.image_pub = rospy.Publisher("/forward/processed/compressed", CompressedImage, queue_size=1)
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
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = ".jpeg"
            msg.data = RiptideVision().compressed_img_msg_data(".jpeg", hud_img)
            self.image_pub.publish(msg)
def main():
    rospy.init_node('qualification_vision')
    ip = ImageProcessor()
    rospy.spin()

if __name__ == "__main__":
    main()
