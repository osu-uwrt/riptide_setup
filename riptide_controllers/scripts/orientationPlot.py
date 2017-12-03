#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import rospy
import geometry_msgs
from riptide_msgs.msg import Imu
#Need to learn how to import ros msgs.

def update_cmd(msg):
    time = msg.header.stamp.secs
    cmd_orientation = msg.euler_rpy.x;
    plot_x(time, cmd_orientation, '*')
    
def update_state(msg):
    time = msg.header.stamp.secs
    state_orientation = msg.euler_rpy.x;
    plot_x(time, state_orientation, 'x')

def plot_x(time, depth, symbol):

    plt.plot(time, depth, symbol)
    plt.draw()
    plt.pause(.1)
    
    
if __name__ == '__main__':
    
    plt.autoscale(False, 'y', None)
    axes = plt.axes()
    axes.set_ylim(-3,3)
    rospy.init_node("plotter")
    rospy.Subscriber("command/orientation", Imu, update_cmd)
    
    rospy.Subscriber("state/imu", Imu, update_state)
    plt.show()
    rospy.spin()