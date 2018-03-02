# Code to test run pole detection on video
import cv2
import riptide_vision as rv

cap = cv2.VideoCapture('./data/vid/pooltest_recent.avi')

while cap.isOpened():
    # Grab frame from video
    ret, frame = cap.read()

    # Detect Gate
    # packet = rv.detect_gate(frame)
    # print(packet)
    # vis = rv.detect_gate_vis(frame, packet)

    # Detect Pole
    packet = rv.detect_pole(frame)
    if packet != []:
        print(packet[0], packet[1])
    vis = rv.detect_pole_vis(frame, packet)

    cv2.imshow('vis', vis)

    if cv2.waitKey(3) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
