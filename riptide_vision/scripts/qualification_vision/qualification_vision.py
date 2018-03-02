# qualification_vision.py
# Subscribes to camera output, publishes what it sees.

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from riptide_vision import RiptideVision

class ImageProcessor:
    def __init__(self):
        self.image_pub = rospy.Publisher("/state/vision/gate", CompressedImage)
        self.fwd_sub = rospy.Subscriber("/forward/image_raw", Image, self.image_callback, queue_size=1)
        self.bridge = CvBridge()
        self.vision = RiptideVision()

    def image_callback(data):
        good = False
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            good = True
        except CvBridgeError as e:
            print(e)

        if good:
            response = self.vision.detect_gate(cv_image)
            hud_img = self.vision.detect_gate_vis(cv_image, response)
            msg = self.bridge.cv2_to_imgmsg(hud_img)
            self.image_pub
def main():
    rospy.init_node('qualification_vision')
    ip = ImageProcessor()
    rospy.spin()

if __name__ == "__main__":
    main()
