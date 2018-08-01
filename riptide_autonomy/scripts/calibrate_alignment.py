#!/usr/bin/env python

import rospy
from riptide_msgs.msg import CalibrateAlignment, DepthCommand, AlignmentCommand, TaskInfo, AttitudeCommand
from geometry_msgs.msg import Point, Vector3

import yaml

task_name = ""
object_name = ""

activate_attitude = False
heading = 0.0

activate_depth = False
depth = 0.0

activate_surge = False
activate_sway = False
activate_heave = False
align_plane = 0
task_id = 0

bbox_dim = 0
bbox_control = 0

target_pos = Point()

task_info = {}

found = False

depth_pub = rospy.Publisher('/command/depth', DepthCommand, queue_size=1)
align_pub = rospy.Publisher('/command/alignment', AlignmentCommand, queue_size=1)
info_pub = rospy.Publisher('/task/info', TaskInfo, queue_size=1)
attitude_pub = rospy.Publisher('/commmand/attitude', AttitudeCommand, queue_size=1)

def CalibrateAlignmentCB(msg):
    global task_name
    global object_name

    global activate_attitude
    global heading

    global activate_depth
    global depth

    global activate_surge
    global activate_sway
    global activate_heave
    global align_plane
    global task_id

    global bbox_dim
    global bbox_control

    global target_pos

    global task_info

    global found

    global depth_pub
    global align_pub
    global info_pub
    global attitude_pub

    found = False

    for task in task_info['tasks']:
        if task['name'] == msg.task_name:
            task_id = task['id']
            align_plane = task['plane']
            found = True

    if not found:
        print 'Invalid task name'
    else:

        if not task_name == msg.task_name:
            t = TaskInfo()
            t.task_name = msg.task_name
            t.task_id = task_id
            t.alignment_plane = align_plane
            print 'Update task info'
            info_pub.publish(t)

        if not activate_depth == msg.depth_active or not depth == msg.depth:
            if msg.depth_active and not msg.heave_active:
                print 'Update depth command'
                d = DepthCommand()
                d.active = True
                d.depth = msg.depth
                depth_pub.publish(d)
            else:
                print 'Update depth command'
                d = DepthCommand()
                d.active = False
                d.depth = 0.0
                depth_pub.publish(d)

        # Alignment management
        align_change = False

        align_change = not (activate_surge == msg.surge_active)
        align_change = not (activate_sway == msg.sway_active) or align_change
        align_change = not (activate_heave == msg.heave_active) or align_change
        align_change = not (object_name == msg.object_name) or align_change
        align_change = not (bbox_dim == int(msg.bbox_dim)) or align_change
        align_change = not (bbox_control == int(msg.bbox_control)) or align_change
        align_change = not (target_pos == msg.target_pos) or align_change
        align_msg = AlignmentCommand()

        if align_change:
            print 'Update align command'
            align_msg.surge_active = msg.surge_active
            align_msg.sway_active = msg.sway_active
            align_msg.heave_active = msg.heave_active
            align_msg.object_name = msg.object_name
            align_msg.alignment_plane = align_plane
            align_msg.bbox_dim = msg.bbox_dim
            align_msg.bbox_control = msg.bbox_control
            align_pub.publish(align_msg)

        if not activate_attitude == msg.heading_active or not heading == msg.heading:
            print 'Update heading command'
            if msg.heading_active:
                a = AttitudeCommand()
                a.roll_active = True
                a.pitch_active = True
                a.yaw_active = True
                rpy = Vector3()
                rpy.x = 0.0
                rpy.y = 0.0
                rpy.z = msg.heading
                a.euler_rpy = rpy
                attitude_pub.publish(a)
            else:
                a = AttitudeCommand()
                a.roll_active = False
                a.pitch_active = False
                a.yaw_active = False
                a.euler_rpy = Vector3()
                attitude_pub.publish(a)

    task_name = msg.task_name
    object_name = msg.object_name

    activate_attitude = msg.heading_active
    heading = msg.heading

    activate_depth = msg.depth_active
    depth = msg.depth

    activate_surge = msg.surge_active
    activate_sway = msg.sway_active
    activate_heave = msg.heave_active

    bbox_dim = msg.bbox_dim
    bbox_control = msg.bbox_control

    target_pos = msg.target_pos


if __name__ == "__main__":
    with open("../osu-uwrt/riptide_software/src/riptide_autonomy/cfg/tasks.yaml", 'r') as stream:
        try:
            task_info = yaml.load(stream)
            print 'Loaded task info'
            rospy.init_node("calibrate_alignment")
            rospy.Subscriber("/command/calibrate_alignment", CalibrateAlignment, CalibrateAlignmentCB)
            rospy.spin()
        except yaml.YAMLError as exc:
            print 'Failed to load task info.. exiting ' + exc
