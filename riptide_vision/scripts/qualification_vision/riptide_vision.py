import numpy as np
import cv2
import math

"""
Helper functions
"""

# img: an OpenCV BGR image
# Returns an image with values of 0 or 255
def hsv_select_hue(img, thresh=(0, 255)):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    h_channel = hsv[:, :, 0]

    binary_output = np.zeros_like(h_channel)
    binary_output[(h_channel > thresh[0]) & (h_channel <= thresh[1])] = 255
    return binary_output


# img: an OpenCV BGR image
# h thresholds: values from 0 to 179
# l and s thresholds: values from 0 to 255
# Returns an image with values of 0 or 255
def hls_select_multiple(img, h_lower, h_upper, l_lower, l_upper, s_lower, s_upper):  # NOQA
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    thresh_min = np.array([h_lower, l_lower, s_lower], np.uint8)
    thresh_max = np.array([h_upper, l_upper, s_upper], np.uint8)
    return cv2.inRange(hls, thresh_min, thresh_max)


# img: an OpenCV image
# size: a tuple, (width, height)
def resize_img(img, size):
    return cv2.resize(img, size)


# img: an OpenCV BGR image
# Returns an image with its brightness adjusted
def histogram_equalization(img):
    # Y component determines brightness of the color
    yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    yuv[:, :, 0] = cv2.equalizeHist(yuv[:, :, 0])
    return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)


# img: an OpenCV image
# angle: float representing degrees
# center: a tuple
# Returns a rotated image
def rotate_img(img, angle, center=None):
    (rows, cols) = img.shape[:2]

    if center is None:
        center = (cols / 2, rows / 2)

    # Perform the rotation
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(img, M, (cols, rows))
    return rotated

class RiptideVision:
    # Function that returns a packet of information
    def detect_gate(sefl, img):
        left_pole_visible = False
        right_pole_visible = False
        displacement_x = None
        displacement_y = None
        roll_correction = None
        x_mid = None
        y_mid = None
        x_center = None
        y_center = None
        cam_center_x = None
        cam_center_y = None

        # Blur the image a bit to reduce detail and get better thresholding results
        blur = cv2.GaussianBlur(img, (5, 5), 3)

        # Binarize the image by thresholding the hls colorspace
        # These values target orange
        hls = hls_select_multiple(blur, 5, 35, 5, 255, 50, 255)

        # Hough Transform parameters used to detect line segments
        # Distance resolution of the accumulator in pixels
        rho = 1

        # Angle resolution of the accumulator in radians
        theta = np.pi / 180

        # Accumulator threshold param, only lines with enough votes get returned
        thresh = 200

        # Minimum line length, line segments shorter than that are rejected
        min_line_length = 200

        # Maximum allowed gap between points on the same line to link them
        max_line_gap = 10

        # Allowed line slope values will be 0 +/- slope_thresh
        slope_thresh = 2

        # (x1, y1): represent the left endpoint of the gate upper beam
        # (x2, y2): the opposite
        x1_avg = None
        y1_avg = None
        x2_avg = None
        y2_avg = None

        # Temporary values used for keep track of max/min values
        x_max = np.float64(-9999)
        x_min = np.float64(9999)
        y_max = np.float64(-9999)
        y_min = np.float64(9999)

        # Flag that keeps track of found line segment matches
        no_hits = True

        # Return line segments that meet the requirements of the paremeters above
        lines = cv2.HoughLinesP(hls, rho=rho, theta=theta, threshold=thresh, minLineLength=min_line_length, maxLineGap=max_line_gap)  # NOQA

        if lines is None:
            return []

        print(len(lines[0]))

        # Average valid line segments to get a slope_avg
        for line in lines[0]:
            x1 = line[0]
            y1 = line[1]
            x2 = line[2]
            y2 = line[3]

            # Hacky solution to avoid divide by zero
            if x1 == x2:
                x1 += 1

            # The slope of the current line segment
            slope = (np.float64(y2) - np.float64(y1)) / (np.float64(x2) - np.float64(x1))  # NOQA

            if (slope <= slope_thresh) and (slope >= -slope_thresh):
                if no_hits:
                    x1_avg = np.float64(x1)
                    y1_avg = np.float64(y1)
                    x2_avg = np.float64(x2)
                    y2_avg = np.float64(y2)

                    no_hits = False
                else:
                    x1_avg = (x1 + x1_avg) / 2
                    y1_avg = (y1 + y1_avg) / 2
                    x2_avg = (x2 + x2_avg) / 2
                    y2_avg = (y2 + y2_avg) / 2

                # Update max values found
                if x_max < x2:
                    x_max = x2
                if x_max < x1:
                    x_max = x1
                if x_min > x2:
                    x_min = x2
                if x_min > x1:
                    x_min = x1

                if y_max < y2:
                    y_max = y2
                if y_max < y1:
                    y_max = y1
                if y_min > y2:
                    y_min = y2
                if y_min > y1:
                    y_min = y1

        if no_hits:
            return []

        # If valid line segments found
        if no_hits is False:
            slope_avg = (y2_avg - y1_avg) / (x2_avg - x1_avg)

            # The point slope equation can be used to extend the averaged line
            # And any point on the average line can be used to find the boundary
            # y - y0 = m (x - x0)
            y_min = slope_avg * (x_min - x1_avg) + y1_avg
            y_max = slope_avg * (x_max - x1_avg) + y1_avg
            y_min = y_min.astype(int)
            y_max = y_max.astype(int)

            # 57.2958 is used to convert radians to degrees
            offset_angle = (57.2958 * (math.acos(abs(x_max - x_min) / (math.sqrt(math.pow((x_max - x_min), 2) + math.pow((y_max - y_min), 2))))))  # NOQA

            roll_correction = round(offset_angle, 4)

            if slope_avg < 0:
                roll_correction = -roll_correction

        # image x axis boundaries
        x_limit_min = 0
        x_limit_max = img.shape[1]

        # Constant that determines what the allowed pixel error for beam
        # endpoint detection
        endpoint_error = 3

        if (x_min >= x_limit_min + endpoint_error):
            left_pole_visible = True

        if (x_max <= x_limit_max - endpoint_error):
            right_pole_visible = True

        if left_pole_visible and right_pole_visible:
            x_mid = (x_max + x_min) / 2
            y_mid = (y_max + y_min) / 2
            x_mid = x_mid.astype(int)
            y_mid = y_mid.astype(int)

            # The upper beam should be 2 meters long
            upper_beam_pixel_magnitude = math.sqrt(math.pow((x_max - x_min), 2) + math.pow((y_max - y_min), 2))  # NOQA

            # Improve hacky solution for avoiding divide by zero, current
            # solution takes advantage of numpy's float64
            if x2_avg == x1_avg:
                x1_avg += 0.00001
            if y2_avg == y1_avg:
                y1_avg += 0.00001
            slope_avg_reciprocal = (np.float64(1.0) / ((y2_avg - y1_avg) / (x2_avg - x1_avg)))  # NOQA

            # The y_center will be 0.75 meters below the detected midpoint
            y_center = 1.0 * (y_mid + ((upper_beam_pixel_magnitude / 2) * 0.75))
            y_center = y_center.astype(int)

            x_center = 1.0 * (((y_center - y_mid) / (-slope_avg_reciprocal)) + x_mid)  # NOQA
            x_center = x_center.astype(int)

            cam_center_y = int(img.shape[0] / 2)
            cam_center_x = int(img.shape[1] / 2)

            # 2.0 is a constant for meters, displacement is in meters
            displacement_x = round(((x_center - cam_center_x) * 2.0) / upper_beam_pixel_magnitude, 4)  # NOQA
            displacement_y = round(((cam_center_y - y_center) * 2.0) / upper_beam_pixel_magnitude, 4)  # NOQA

        packet = []
        packet.append(left_pole_visible)
        packet.append(right_pole_visible)
        packet.append(displacement_x)
        packet.append(displacement_y)
        packet.append(roll_correction)
        packet.append(x_mid)
        packet.append(y_mid)
        packet.append(x_center)
        packet.append(y_center)
        packet.append(cam_center_x)
        packet.append(cam_center_y)

        if None in [x_mid, y_mid, x_center, y_center, cam_center_x, cam_center_y]:
            return []

        return packet


    # This function draws on an image with information from the detect_gate packet,
    # this can be improved but it is not a priority
    def detect_gate_vis(self, img, packet):
        if len(packet) != 0:
            cv2.line(img, (packet[5], packet[6]), (packet[7], packet[8]), (0, 0, 255), 5)  # NOQA
        return img

    def compressed_img_msg_data(self, type, image):
        return np.array(cv2.imencode(type, image)[1]).tostring()

    # Attempts to detect a vertical pole, returns a list with information
    def detect_pole(self, img):
        roll_correction = None
        beam_thickness = None

        img_shrink_factor = 3
        img = resize_img(img, (img.shape[1] / img_shrink_factor, img.shape[0] / img_shrink_factor))  # NOQA

        # Blur the image a bit to reduce detail and get better thresholding results
        blur = cv2.GaussianBlur(img, (5, 5), 3)

        # Binarize the image by thresholding the hls colorspace
        # These values target orange
        hls = hls_select_multiple(blur, 5, 35, 5, 255, 50, 255)

        # Hough Transform parameters used to detect line segments
        # Distance resolution of the accumulator in pixels
        rho = 1

        # Angle resolution of the accumulator in radians
        theta = np.pi / 180

        # Accumulator threshold param, only lines with enough votes get returned
        thresh = 200

        # Minimum line length, line segments shorter than that are rejected
        min_line_length = 200

        # Maximum allowed gap between points on the same line to link them
        max_line_gap = 2

        # Allowed line slope values will be 0 +/- slope_thresh
        slope_thresh = 3

        # (x1, y1): represent the left endpoint of the gate upper beam
        # (x2, y2): the opposite
        x1_avg = None
        y1_avg = None
        x2_avg = None
        y2_avg = None

        # Temporary values used for keep track of max/min values
        x_max = np.float64(-9999)
        x_min = np.float64(9999)
        y_max = np.float64(-9999)
        y_min = np.float64(9999)

        # Flag that keeps track of found line segment matches
        no_hits = True

        # Return line segments that meet the requirements of the paremeters above
        lines = cv2.HoughLinesP(hls, rho=rho, theta=theta, threshold=thresh, minLineLength=min_line_length, maxLineGap=max_line_gap)  # NOQA

        if lines is None:
            return []

        # Average valid line segments to get a slope_avg
        for line in lines[0]:
            x1 = line[0]
            y1 = line[1]
            x2 = line[2]
            y2 = line[3]

            # Hacky solution to avoid divide by zero
            if x1 == x2:
                x1 += 1

            # The slope of the current line segment
            slope = (np.float64(y2) - np.float64(y1)) / (np.float64(x2) - np.float64(x1))  # NOQA

            if not((slope <= slope_thresh) and (slope >= -slope_thresh)):
                if no_hits:
                    x1_avg = np.float64(x1)
                    y1_avg = np.float64(y1)
                    x2_avg = np.float64(x2)
                    y2_avg = np.float64(y2)

                    no_hits = False
                else:
                    x1_avg = (x1 + x1_avg) / 2
                    y1_avg = (y1 + y1_avg) / 2
                    x2_avg = (x2 + x2_avg) / 2
                    y2_avg = (y2 + y2_avg) / 2

                # Update max values found
                if x_max < x2:
                    x_max = x2
                if x_max < x1:
                    x_max = x1
                if x_min > x2:
                    x_min = x2
                if x_min > x1:
                    x_min = x1

                if y_max < y2:
                    y_max = y2
                if y_max < y1:
                    y_max = y1
                if y_min > y2:
                    y_min = y2
                if y_min > y1:
                    y_min = y1

        if no_hits:
            return []

        # If valid line segments found
        if no_hits is False:
            slope_avg = (y2_avg - y1_avg) / (x2_avg - x1_avg)

            # The point slope equation can be used to extend the averaged line
            # And any point on the average line can be used to find the boundary
            # y - y0 = m (x - x0)
            y_min = slope_avg * (x_min - x1_avg) + y1_avg
            y_max = slope_avg * (x_max - x1_avg) + y1_avg
            y_min = y_min.astype(int)
            y_max = y_max.astype(int)

            x_mid = int((x1_avg + x2_avg) / 2)
            y_mid = int((y1_avg + y2_avg) / 2)

            if slope_avg < 0:
                # 57.2958 is used to convert radians to degrees
                offset_angle = (57.2958 * (math.acos(abs(x_max - x_min) / (math.sqrt(math.pow((x_max - x_min), 2) + math.pow((y_max - y_min), 2))))))  # NOQA
                offset_angle = abs(offset_angle - 90.0)
                roll_correction = round(offset_angle, 4)
                roll_correction = roll_correction
            else:
                # 57.2958 is used to convert radians to degrees
                offset_angle = (57.2958 * (math.acos(abs(x_max - x_min) / (math.sqrt(math.pow((x_max - x_min), 2) + math.pow((y_max - y_min), 2))))))  # NOQA
                offset_angle = 90 - offset_angle
                roll_correction = round(offset_angle, 4)
                roll_correction = -roll_correction

            sum_rows = np.sum(hls, axis=1)
            beam_thickness = sum_rows[y_mid] / 255

        return [roll_correction, beam_thickness, x_mid, y_mid, x_min, x_max, y_min, y_max, slope_avg]  # NOQA


    def detect_pole_vis(self, img, packet):
        img = resize_img(img, (img.shape[1] / 3, img.shape[0] / 3))

        if packet != []:
            cv2.line(img, (packet[4], packet[6]), (packet[5], packet[7]), (0, 255, 0), 3)  # NOQA

        return img
