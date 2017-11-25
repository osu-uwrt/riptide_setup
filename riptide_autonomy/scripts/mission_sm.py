#!/usr/bin/env python

import rospy
import smach
import smach_ros

def main():
    mission_sm = smach.StateMachine(outcomes=['mission_completed', 'mission_failed'])
