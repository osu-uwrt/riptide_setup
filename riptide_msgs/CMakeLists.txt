cmake_minimum_required(VERSION 2.8.3)
project(riptide_msgs)
add_compile_options(-std=c++11)

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
    AttitudeCommand.msg
    AcousticsStatus.msg
    AcousticsCommand.msg
    Pneumatics.msg
    Constants.msg
    TaskInfo.msg
    Object.msg
    MissionState.msg
    StatusLight.msg
    CalibrateAlignment.msg
    NetLoad.msg
    LinearCommand.msg
    Dvl.msg
)

generate_messages(
    DEPENDENCIES 
    std_msgs 
    geometry_msgs 
    darknet_ros_msgs

)

catkin_package(
    CATKIN_DEPENDS 
    message_runtime
    
)

include_directories(${catkin_INCLUDE_DIRS})
