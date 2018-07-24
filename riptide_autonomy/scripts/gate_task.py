#!/usr/bin/env python
# gate_align.py
# Align to the orange qualiification gate and drive through.

import rospy
from riptide_msgs.msg import AlignmentCommand, BoundingBox, GateData, DepthCommand

class GateTask:
    TARGET_BBOX_A = 50
    TARGET_BBOX_B = 100
    TARGET_BBOX_C = 200

    def __init__(self):
        self.cmd_pub = rospy.Publisher("/command/depth", DepthCommand)
        self.task_sub = rospy.Subscriber("/task/gate")
        self.alignment_cmd = AlignmentCommand()

    def loop(self):




def main():
    rospy.init_node("gate_task")
    gt = GateTask()
    while not rospy.is_shutdown():
        gt.loop()

if __name__ == '__main__':
    main()
