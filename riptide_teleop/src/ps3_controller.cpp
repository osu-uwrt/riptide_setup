#include "riptide_teleop/ps3_controller.h"

#define GRAVITY 9.81 // [m/s^2]
#define WATER_DENSITY 1000 // [kg/m^3]

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ps3_controller");
  PS3Controller ps3;
  ps3.Loop();
}

PS3Controller::PS3Controller() : nh("ps3_controller") {
  joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &PS3Controller::JoyCB, this);
  depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &PS3Controller::DepthCB, this);
  attitude_pub = nh.advertise<geometry_msgs::Vector3>("/command/manual/attitude", 1);
  lin_accel_pub = nh.advertise<geometry_msgs::Vector3>("/command/manual/accel/linear", 1);
  depth_pub = nh.advertise<riptide_msgs::DepthCommand>("/command/manual/depth", 1);
  reset_pub = nh.advertise<riptide_msgs::ResetControls>("/controls/reset", 1);
  plane_pub = nh.advertise<riptide_msgs::PS3Plane>("/command/ps3_plane", 1);

  PS3Controller::LoadParam<bool>("is_depth_working", isDepthWorking); // Is depth sensor working?
  PS3Controller::LoadParam<double>("rate", rt); // [Hz]
  PS3Controller::LoadParam<double>("buoyancy_depth_thresh", buoyancy_depth_thresh); // [m]
  PS3Controller::LoadParam<double>("max_roll_limit", MAX_ROLL); // [m/s^2]
  PS3Controller::LoadParam<double>("max_pitch_limit", MAX_PITCH); // [m/s^2]
  PS3Controller::LoadParam<double>("max_x_accel", MAX_XY_ACCEL); // [m/s^2]
  PS3Controller::LoadParam<double>("max_z_accel", MAX_Z_ACCEL); // [m/s^2]
  PS3Controller::LoadParam<double>("max_depth", MAX_DEPTH); // [m]
  PS3Controller::LoadParam<double>("cmd_roll_rate", CMD_ROLL_RATE); // [deg/s]
  PS3Controller::LoadParam<double>("cmd_pitch_rate", CMD_PITCH_RATE); // [deg/s]
  PS3Controller::LoadParam<double>("cmd_yaw_rate", CMD_YAW_RATE); // [deg/s]
  PS3Controller::LoadParam<double>("cmd_depth_rate", CMD_DEPTH_RATE); // [deg/s]
  PS3Controller::LoadParam<double>("boost", boost); // Factor to multiply pressed button/axis for faster rate
  // When using the boost factor, you must subtract one from its value for it
  // to have the desired effect. See below for implementation

  isReset = true;
  isStarted = false;
  isInit = false;
  isR2Init = false;
  isL2Init = false;
  isDepthInit = false;
  current_depth = 0;
  euler_rpy.setZero();
  alignment_plane = (bool)riptide_msgs::Constants::PLANE_YZ;

  roll_factor = CMD_ROLL_RATE/rt;
  pitch_factor = CMD_PITCH_RATE/rt;
  yaw_factor = CMD_YAW_RATE/rt;
  depth_factor = CMD_DEPTH_RATE/rt;

  PS3Controller::InitMsgs();
}

void PS3Controller::InitMsgs() {
  reset_msg.reset_surge = true;
  reset_msg.reset_sway = true;
  reset_msg.reset_heave = true;
  PS3Controller::ResetControllers();
}

// Load parameter from namespace
template <typename T>
void PS3Controller::LoadParam(string param, T &var)
{
  try
  {
    if (!nh.getParam(param, var))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    string ns = nh.getNamespace();
    ROS_ERROR("PS3 Controller Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void PS3Controller::DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg) {
  current_depth = depth_msg->depth;
}

// Create rotation matrix from IMU orientation
void PS3Controller::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg) {
  vector3MsgToTF(imu_msg->euler_rpy, euler_rpy);
}

void PS3Controller::JoyCB(const sensor_msgs::Joy::ConstPtr& joy) {
  if(joy->buttons[BUTTON_SHAPE_X]) { // Reset Vehicle (The "X" button)
    isReset = true;
    isStarted = false;
    isInit = false;
    isDepthInit = false;
    PS3Controller::ResetControllers();
  }
  else if(isReset) { // If reset, must wait for Start button to be pressed
    if(joy->buttons[BUTTON_START]) {
      isStarted = true;
      isReset = false;
      PS3Controller::EnableControllers();
    }
  }
  else if(isStarted) {
    if(!isInit) { // Initialize to roll and pitch of 0 [deg], and set yaw to current heading
      isInit = true;
      cmd_attitude.z = round(euler_rpy.z());

      if(isDepthWorking)
        cmd_depth.absolute = current_depth;
      else
        cmd_accel.z = 0;
    }
    else if(isInit) {
      // Update Roll and Pitch
      if(joy->buttons[BUTTON_SHAPE_CIRCLE]) { // Set both roll and pitch to 0 [deg]
        cmd_attitude.x = 0;
        cmd_attitude.y = 0;
        delta_attitude.x = 0;
        delta_attitude.y = 0;
      }
      else {
        if(joy->buttons[BUTTON_CROSS_RIGHT])
          delta_attitude.x = roll_factor * (1 + (boost-1)*joy->buttons[BUTTON_SHAPE_SQUARE]); // Right -> inc roll
        else if(joy->buttons[BUTTON_CROSS_LEFT])
          delta_attitude.x = -roll_factor * (1+ (boost-1)*joy->buttons[BUTTON_SHAPE_SQUARE]); // Left -> dec roll
        else
          delta_attitude.x = 0;

        if(joy->buttons[BUTTON_CROSS_UP])
          delta_attitude.y = -pitch_factor * (1+ (boost-1)*joy->buttons[BUTTON_SHAPE_SQUARE]); // Up -> dec pitch (Front points upward)
        else if(joy->buttons[BUTTON_CROSS_DOWN])
            delta_attitude.y = pitch_factor * (1+ (boost-1)*joy->buttons[BUTTON_SHAPE_SQUARE]); //Down -> inc pitch (Front points downward)
        else
          delta_attitude.y = 0;
      }

      // Update Yaw
      if(abs(joy->axes[AXES_STICK_LEFT_UD]) < 0.7) // Try to avoid yaw and depth simultaneously
        delta_attitude.z = joy->axes[AXES_STICK_LEFT_LR]*yaw_factor * (1+ (boost-1)*joy->buttons[BUTTON_SHAPE_SQUARE]);
      else
        delta_attitude.z = 0;

      // Update Depth/Z-accel
      if(isDepthWorking) { // Depth sensor working properly
        if(joy->buttons[BUTTON_SHAPE_TRIANGLE]) { // Enable depth controller
          isDepthInit = true;
          reset_msg.reset_depth = false;
        }
        else if(isDepthInit && abs(joy->axes[AXES_STICK_LEFT_LR]) < 0.7) // Try to avoid yaw and depth simultaneously
          delta_depth = -joy->axes[AXES_STICK_LEFT_UD]*depth_factor * (1+ (boost-1)*joy->buttons[BUTTON_SHAPE_SQUARE]); // Up -> dec depth, Down -> inc depth
        else
          delta_depth = 0;
        }
      else { // Depth sensor not working properly
        // For some reason, R2 and L2 are initialized to 0
        if(!isR2Init && (joy->axes[AXES_REAR_R2] != 0))
          isR2Init = true;
        if(!isL2Init && (joy->axes[AXES_REAR_L2] != 0))
          isL2Init = true;

        if(isR2Init && (1 - joy->axes[AXES_REAR_R2] != 0)) // If pressed at all, inc z-accel
          cmd_accel.z = 0.5*(1 - joy->axes[AXES_REAR_R2])*MAX_Z_ACCEL; // Multiplied by 0.5 to scale axes value from 0 to 1
        else if(isL2Init && (1 - joy->axes[AXES_REAR_L2] != 0)) // If pressed at all, dec z-accel
          cmd_accel.z = 0.5*(1 - joy->axes[AXES_REAR_L2])*MAX_Z_ACCEL; // Multiplied by 0.5 to scale axes value from 0 to 1
      }

      // Update Linear XY Accel
      cmd_accel.x = joy->axes[AXES_STICK_RIGHT_UD]*MAX_XY_ACCEL; // Surge pos. forward
      cmd_accel.y = joy->axes[AXES_STICK_RIGHT_LR]*MAX_XY_ACCEL; // Sway pos. left

      if(joy->buttons[BUTTON_SELECT])
        alignment_plane = !alignment_plane;
    }
  }
}

// Run when Reset button is pressed
void PS3Controller::ResetControllers() {
  reset_msg.reset_depth = true;
  reset_msg.reset_roll = true;
  reset_msg.reset_pitch = true;
  reset_msg.reset_yaw = true;
  reset_msg.reset_pwm = true;

  cmd_attitude.x = 0;
  cmd_attitude.y = 0;
  cmd_attitude.z = 0;
  delta_attitude.x = 0;
  delta_attitude.y = 0;
  delta_attitude.z = 0;

  cmd_accel.x = 0;
  cmd_accel.y = 0;
  cmd_accel.z = 0;

  cmd_depth.isManual = true;
  cmd_depth.absolute = 0;
  cmd_depth.relative = 0;
  delta_depth = 0;
}

// Run when Start button is pressed
void PS3Controller::EnableControllers() {
  reset_msg.reset_roll = false;
  reset_msg.reset_pitch = false;
  reset_msg.reset_yaw = false;
  reset_msg.reset_pwm = false;
}

double PS3Controller::Constrain(double current, double max) {
  if(current > max)
    return max;
  else if(current < -1*max)
    return -1*max;
  return current;
}

void PS3Controller::UpdateCommands() {
  cmd_attitude.x += delta_attitude.x;
  cmd_attitude.y += delta_attitude.y;
  cmd_attitude.z += delta_attitude.z;

  cmd_attitude.x = PS3Controller::Constrain(cmd_attitude.x, MAX_ROLL);
  cmd_attitude.y = PS3Controller::Constrain(cmd_attitude.y, MAX_PITCH);

  if(cmd_attitude.z > 180)
    cmd_attitude.z -= 360;
  if(cmd_attitude.z < -180)
    cmd_attitude.z += 360;

  cmd_depth.relative = delta_depth;
  cmd_depth.absolute += cmd_depth.relative;
  cmd_depth.absolute = PS3Controller::Constrain(cmd_depth.absolute, MAX_DEPTH);
  if(cmd_depth.absolute < 0)
    cmd_depth.absolute = 0;

  plane_msg.alignment_plane = (int)alignment_plane;
}

void PS3Controller::PublishCommands() {
  reset_pub.publish(reset_msg);
  attitude_pub.publish(cmd_attitude);
  lin_accel_pub.publish(cmd_accel);
  if(isDepthWorking)
    depth_pub.publish(cmd_depth);
  plane_pub.publish(plane_msg);
}

void PS3Controller::Loop()
{
  ros::Rate rate(rt); // MUST have this rate in here, since all factors are based on it
  while(ros::ok())
  {
    PS3Controller::UpdateCommands();
    PS3Controller::PublishCommands();
    ros::spinOnce();
    rate.sleep();
  }
}
