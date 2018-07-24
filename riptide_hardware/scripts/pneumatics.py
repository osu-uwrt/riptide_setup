#!/usr/bin/env python

import serial
import rospy
from riptide_msgs.msg import Pneumatics
#set up the symlink
COM_PORT = '/dev/ttyS0'
ser = serial.Serial(COM_PORT, baudrate=9600, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0.1)
def pneuCB(pneuMsg):
    pneuStart = "++++"
    pneuEnd = "@@@@\n"
    #blank nothing messages
    pin1 = "!nnnn"
    pin2 = "!nnnn"
    pin3 = "!nnnn"
    pin4 = "!nnnn"
    #conditionals to send to
    if pneuMsg.torpedo_stbd:
        pin2 = "!{:0>4d}".format(pneuMsg.duration)
    if pneuMsg.torpedo_port:
        pin3 = "!{:0>4d}".format(pneuMsg.duration)
    if pneuMsg.markerdropper:
        pin1 = "!{:0>4d}".format(pneuMsg.duration)
    if pneuMsg.manipulator:
        pin4 = "!{:0>4d}".format(pneuMsg.duration)
    final_pneu = pneuStart + pin1 + pin2 + pin3 + pin4 + pneuEnd
    #change this write to match what erika is looking for
    print "sent: ", final_pneu
    ser.write(final_pneu)

def main():
    print "attempting to start node"
    rospy.init_node('pneumatics')
    print "starting node"
    pneuPub = rospy.Subscriber("/command/pneumatics", Pneumatics, pneuCB, queue_size=1)
    rate = rospy.Rate(0.5)
    pneuMsg = Pneumatics()
    while not rospy.is_shutdown():
        if ser.inWaiting() != 0:
            ser.readline()
        else:
            rospy.loginfo("No pneumatics message")
        rate.sleep()

if __name__ == "__main__": main()
