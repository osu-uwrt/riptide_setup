cmake_minimum_required(VERSION 2.8.3)
project(riptide_msgs)

find_package(catkin REQUIRED
    COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
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
    ObjectData.msg
    GateData.msg
    ResetControls.msg
    ControlStatus.msg
    ControlStatusAngular.msg
    ControlStatusLinear.msg
    MassVol.msg
    RotationOut.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})
