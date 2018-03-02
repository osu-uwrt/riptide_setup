import cv2
import riptide_vision as rv

"""
Code for thresholding the hls colorspace
"""

# Initial threshold values, targets orange
h_lower = 5
h_upper = 35
l_lower = 5
l_upper = 255
s_lower = 50
s_upper = 255

cap = cv2.VideoCapture('../video_here.avi')

cv2.namedWindow('binarized')
cv2.namedWindow('trackbars')


def update_trackbars(callback):
    global h_lower
    global h_upper
    global l_lower
    global l_upper
    global s_lower
    global s_upper

    h_lower = cv2.getTrackbarPos('h_lower', 'trackbars')
    h_upper = cv2.getTrackbarPos('h_upper', 'trackbars')
    l_lower = cv2.getTrackbarPos('l_lower', 'trackbars')
    l_upper = cv2.getTrackbarPos('l_upper', 'trackbars')
    s_lower = cv2.getTrackbarPos('s_lower', 'trackbars')
    s_upper = cv2.getTrackbarPos('s_upper', 'trackbars')

cv2.createTrackbar('h_lower', 'trackbars', h_lower, 179, update_trackbars)
cv2.createTrackbar('h_upper', 'trackbars', h_upper, 179, update_trackbars)
cv2.createTrackbar('l_lower', 'trackbars', l_lower, 255, update_trackbars)
cv2.createTrackbar('l_upper', 'trackbars', l_upper, 255, update_trackbars)
cv2.createTrackbar('s_lower', 'trackbars', s_lower, 255, update_trackbars)
cv2.createTrackbar('s_upper', 'trackbars', s_upper, 255, update_trackbars)

while cap.isOpened():
    ret, frame = cap.read()

    frame = rv.resize_img(frame, (frame.shape[1] / 2, frame.shape[0] / 2))

    frame = cv2.GaussianBlur(frame, (5, 5), 3)

    hls = rv.hls_select_multiple(frame, h_lower, h_upper, l_lower, l_upper, s_lower, s_upper)  # NOQA

    cv2.imshow('binarized', hls)
    cv2.imshow('RGB', frame)

    cv2.waitKey()

cap.release()
cv2.destroyAllWindows()
