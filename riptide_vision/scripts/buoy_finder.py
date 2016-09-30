"""A prototype buoy detector."""

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # , CvBridgeError

red_lower = (-15, 152, 152)
red_upper = (25, 255, 255)
green_lower = (45, 75, 75)
green_upper = (85, 230, 230)

bridge = CvBridge()
pub = rospy.Publisher('stereo/left/image_buoy', Image, queue_size=1)


def find_buoy(image, lower, upper):

    x = -1
    y = -1
    radius = -1
    center = -1

    # Remove non-targeted colors:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    threshold = cv2.inRange(hsv, lower, upper)

    # Remove noise:
    kernel = np.ones((5, 5), np.uint8)
    opening = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    # Identify contours
    contours, hierarchy = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:

        # Determine largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        center = None
        # Find its bounding cirlce
        (x, y), radius = cv2.minEnclosingCircle(largest_contour)
        if not radius < 10:
            # Draw its bounding circle
            cv2.circle(image, (int(x), int(y)),
                       int(radius), (255, 255, 255), 2)

            # Calculate its moments
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:

                # Calculate its center of mass
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # Draw its center of mass
                cv2.circle(image, center, 5, (0, 0, 0), -1)

    return ((x, y), radius), center


def callback(image_message_in):
    cv_image = bridge.imgmsg_to_cv2(image_message_in, desired_encoding="bgr8")
    # (rows, cols, chans) = cv_image.shape
    # rospy.loginfo()

    bounds, target = find_buoy(cv_image, green_lower, green_upper)

    bounds, target = find_buoy(cv_image, red_lower, red_upper)

    image_message_out = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
    pub.publish(image_message_out)


def buoy_finder():
    rospy.init_node('buoy_finder', anonymous=True)
    rospy.Subscriber('stereo/left/image_color', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        buoy_finder()
    except rospy.ROSInterruptException:
        pass
