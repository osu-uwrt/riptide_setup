#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
import rospy
from riptide_msgs.msg import Depth
#Need to learn how to import ros msgs.

def update_cmd(msg):
    time = msg.header.stamp.secs
    cmd_depth = msg.depth
    plot_x(time, cmd_depth, '*')
    
def update_state(msg):
    time = msg.header.stamp.secs
    state_depth = msg.depth
    plot_x(time, state_depth, 'x')

def plot_x(time, depth, symbol):

    plt.plot(time, depth, symbol)
    plt.draw()
    plt.pause(.1)
    
    
if __name__ == '__main__':
    
    plt.autoscale(False, 'y', None)
    axes = plt.axes()
    axes.set_ylim(-3,3)
    rospy.init_node("plotter")
    rospy.Subscriber("command/depth", Depth, update_cmd)
    
    rospy.Subscriber("state/depth", Depth, update_state)
    plt.show()
    rospy.spin()