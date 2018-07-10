cmake_minimum_required(VERSION 2.8.3)
project(riptide_msgs)

find_package(catkin REQUIRED
    COMPONENTS
    message_generation
    std_msgs
    geometry_msgs
    darknet_ros_msgs
)

add_message_files(
    FILES
    Depth.msg
    Thrust.msg
    ThrustStamped.msg
    Pwm.msg
    PwmStamped.msg
    Imu.msg
    ImuVerbose.msg
    SwitchState.msg
    TaskAlignment.msg
    GateData.msg
    PoleData.msg
    BoundingBox.msg
    ResetControls.msg
    ControlStatus.msg
    ControlStatusAngular.msg
    ControlStatusLinear.msg
    AlignmentCommand.msg
    DepthCommand.msg
    Acoustics.msg
    Pneumatics.msg
    Constants.msg
    ThrusterResiduals.msg
    TaskInfo.msg
    Alignment.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs darknet_ros_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})
