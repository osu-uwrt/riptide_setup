/*
Start: start the robot, roll and pitch are hold to zero, depth is set to half meter. 

While driving:
Right stick, forward backward left right in the robot frame.
Left stick, up down deep shallow, LR yaw
Cross buttons: up down: pitch lef right: roll

Square: boost, go faster, not for x and y
Circle; zero roll and pith
X: cut thrusters
 */

#include "riptide_teleop/ps3_controller.h"

#define GRAVITY 9.81       // [m/s^2]
#define WATER_DENSITY 1000 // [kg/m^3]

bool IS_ATTITUDE_RESET = false;
bool IS_DEPTH_RESET = false;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ps3_controller");
  PS3Controller ps3;
  ps3.Loop();
}

PS3Controller::PS3Controller() : nh("ps3_controller")
{
  joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, &PS3Controller::JoyCB, this);
  depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &PS3Controller::DepthCB, this);
  imu_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &PS3Controller::ImuCB, this);
  roll_pub = nh.advertise<riptide_msgs::AttitudeCommand>("/command/roll", 1);
  pitch_pub = nh.advertise<riptide_msgs::AttitudeCommand>("/command/pitch", 1);
  yaw_pub = nh.advertise<riptide_msgs::AttitudeCommand>("/command/yaw", 1);
  x_pub = nh.advertise<riptide_msgs::LinearCommand>("/command/x", 1);
  y_pub = nh.advertise<riptide_msgs::LinearCommand>("/command/y", 1);
  depth_pub = nh.advertise<riptide_msgs::DepthCommand>("/command/depth", 1);
  reset_pub = nh.advertise<riptide_msgs::ResetControls>("/controls/reset", 1);
  camera_pub = nh.advertise<std_msgs::Int8>("/command/camera", 1);

  PS3Controller::LoadParam<double>("rate", rt);                       // [Hz]
  PS3Controller::LoadParam<double>("max_x_force", MAX_X_FORCE);       // [m/s^2]
  PS3Controller::LoadParam<double>("max_y_force", MAX_Y_FORCE);       // [m/s^2]
  PS3Controller::LoadParam<double>("cmd_roll_rate", CMD_ROLL_RATE);   // [deg/s]
  PS3Controller::LoadParam<double>("cmd_pitch_rate", CMD_PITCH_RATE); // [deg/s]
  PS3Controller::LoadParam<double>("cmd_yaw_rate", CMD_YAW_RATE);     // [deg/s]
  PS3Controller::LoadParam<double>("cmd_depth_rate", CMD_DEPTH_RATE); // [deg/s]
  PS3Controller::LoadParam<double>("boost", boost);                   // Factor to multiply pressed button/axis for faster rate
  // When using the boost factor, you must subtract one from its value for it
  // to have the desired effect. See below for implementation

  isReset = true;
  isR2Init = false;
  isL2Init = false;

  current_depth = 0;
  euler_rpy.x = 0;
  euler_rpy.y = 0;
  euler_rpy.z = 0;
  camera = 0;

  roll_factor = CMD_ROLL_RATE / rt;
  pitch_factor = CMD_PITCH_RATE / rt;
  yaw_factor = CMD_YAW_RATE / rt;
  depth_factor = CMD_DEPTH_RATE / rt;

  axes_rear_R2 = 0;
  axes_rear_L2 = 0;

  PS3Controller::InitMsgs();

  ROS_INFO("PS3 Controller Reset. Press Start to begin.");
}

void PS3Controller::InitMsgs()
{
  reset_msg.reset_pwm = true;

  euler_rpy.x = 0;
  euler_rpy.y = 0;
  euler_rpy.z = 0;

  PS3Controller::DisableControllers();
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
  catch (int e)
  {
    string ns = nh.getNamespace();
    ROS_ERROR("PS3 Controller Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void PS3Controller::DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg)
{
  current_depth = depth_msg->depth;
}

// Create rotation matrix from IMU orientation
void PS3Controller::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg)
{
  euler_rpy = imu_msg->rpy_deg;
}

void PS3Controller::JoyCB(const sensor_msgs::Joy::ConstPtr &joy)
{

  // For some reason, R2 and L2 are initialized to 0
  if (!isR2Init && (joy->axes[AXES_REAR_R2] != 0))
    isR2Init = true;
  if (!isL2Init && (joy->axes[AXES_REAR_L2] != 0))
    isL2Init = true;

  if (isR2Init)
  {
    axes_rear_R2 = 0.5 * (1 - joy->axes[AXES_REAR_R2]);
  }
  else
  {
    axes_rear_R2 = 0;
  }

  if (isL2Init)
  {
    axes_rear_L2 = 0.5 * (1 - joy->axes[AXES_REAR_L2]);
  }
  else
  {
    axes_rear_L2 = 0;
  }

  if (!isReset && joy->buttons[BUTTON_SHAPE_X])
  { // Reset Vehicle (The "X" button)
    isReset = true;
    PS3Controller::DisableControllers();
    ROS_INFO("PS3 Controller Reset. Press Start to begin.");
  }
  else if (isReset)
  { // If reset, must wait for Start button to be pressed
    if (joy->buttons[BUTTON_START])
    {
      isReset = false;
      reset_msg.reset_pwm = false;
      reset_pub.publish(reset_msg);
      cmd_euler_rpy.z = (int)euler_rpy.z;
      cmd_depth.depth = 0.5;
    }
  }
  else
  {
    // Update Roll and Pitch
    if (joy->buttons[BUTTON_SHAPE_CIRCLE])
    { // Set both roll and pitch to 0 [deg]
      cmd_euler_rpy.x = 0;
      cmd_euler_rpy.y = 0;
      delta_attitude.x = 0;
      delta_attitude.y = 0;
    }
    else
    { // Update roll/pitch angles with CROSS
      // Roll
      if (joy->buttons[BUTTON_CROSS_RIGHT])
        delta_attitude.x = roll_factor * (1 + (boost - 1) * joy->buttons[BUTTON_SHAPE_SQUARE]); // Right -> inc roll
      else if (joy->buttons[BUTTON_CROSS_LEFT])
        delta_attitude.x = -roll_factor * (1 + (boost - 1) * joy->buttons[BUTTON_SHAPE_SQUARE]); // Left -> dec roll
      else
      {
        delta_attitude.x = 0;
        // ROS_INFO("FALSE FROM ENABLE");
      }

      // Pitch
      if (joy->buttons[BUTTON_CROSS_UP])
        delta_attitude.y = -pitch_factor * (1 + (boost - 1) * joy->buttons[BUTTON_SHAPE_SQUARE]); // Up -> inc pitch (Nose points upward)
      else if (joy->buttons[BUTTON_CROSS_DOWN])
        delta_attitude.y = pitch_factor * (1 + (boost - 1) * joy->buttons[BUTTON_SHAPE_SQUARE]); //Down -> dec pitch (Nose points downward)
      else
        delta_attitude.y = 0;

      // Yaw
      delta_attitude.z = -joy->axes[AXES_STICK_LEFT_LR] * yaw_factor * (1 + (boost - 1) * joy->buttons[BUTTON_SHAPE_SQUARE]);
    }

    delta_depth = -joy->axes[AXES_STICK_LEFT_UD] * depth_factor * (1 + (boost - 1) * joy->buttons[BUTTON_SHAPE_SQUARE]); // Up -> dec depth, Down -> inc depth

    /*// Code for using R2 and L2
      if (isR2Init && (1 - joy->axes[AXES_REAR_R2] != 0))                     // If pressed at all, inc z-accel
        cmd_force_z.data = 0.5 * (1 - joy->axes[AXES_REAR_R2]) * MAX_Z_FORCE; // Multiplied by 0.5 to scale axes value from 0 to 1
      else if (isL2Init && (1 - joy->axes[AXES_REAR_L2] != 0))                // If pressed at all, dec z-accel
        cmd_force_z.data = 0.5 * (1 - joy->axes[AXES_REAR_L2]) * MAX_Z_FORCE; // Multiplied by 0.5 to scale axes value from 0 to 1*/
  }

  // Update Linear XY Accel
  cmd_x.value = joy->axes[AXES_STICK_RIGHT_UD] * MAX_X_FORCE;  // Surge (X) positive forward
  cmd_y.value = -joy->axes[AXES_STICK_RIGHT_LR] * MAX_Y_FORCE; // Sway (Y) positive left

  if (joy->buttons[BUTTON_SELECT])
    camera = (camera + 1) % 2;
}

// Run when Reset button is pressed
void PS3Controller::DisableControllers()
{
  reset_msg.reset_pwm = true;

  delta_attitude.x = 0;
  delta_attitude.y = 0;
  delta_attitude.z = 0;

  delta_depth = 0;

  cmd_x.value = 0;
  cmd_y.value = 0;

  reset_pub.publish(reset_msg);
}

double PS3Controller::Constrain(double current, double max)
{
  if (current > max)
    return max;
  else if (current < -1 * max)
    return -1 * max;
  return current;
}

void PS3Controller::UpdateCommands()
{
  cmd_euler_rpy.x += delta_attitude.x;
  cmd_euler_rpy.y += delta_attitude.y;
  cmd_euler_rpy.z += delta_attitude.z;

  if (cmd_euler_rpy.z > 180)
    cmd_euler_rpy.z -= 360;
  if (cmd_euler_rpy.z < -180)
    cmd_euler_rpy.z += 360;

  cmd_depth.active = true;
  cmd_depth.depth += delta_depth;
  if (cmd_depth.depth < 0)
    cmd_depth.depth = 0;

  camera_msg.data = camera;
}

void PS3Controller::PublishCommands()
{
  riptide_msgs::AttitudeCommand roll_cmd;
  riptide_msgs::AttitudeCommand pitch_cmd;
  riptide_msgs::AttitudeCommand yaw_cmd;

  roll_cmd.value = cmd_euler_rpy.x;
  pitch_cmd.value = cmd_euler_rpy.y;
  yaw_cmd.value = cmd_euler_rpy.z;

  roll_cmd.mode = roll_cmd.POSITION;
  pitch_cmd.mode = pitch_cmd.POSITION;
  yaw_cmd.mode = yaw_cmd.POSITION;

  roll_pub.publish(roll_cmd);
  pitch_pub.publish(pitch_cmd);
  yaw_pub.publish(yaw_cmd);

  cmd_x.mode = riptide_msgs::LinearCommand::FORCE;
  cmd_y.mode = riptide_msgs::LinearCommand::FORCE;
  x_pub.publish(cmd_x);
  y_pub.publish(cmd_y);

  depth_pub.publish(cmd_depth);

  camera_pub.publish(camera_msg);
}

// This loop function is critical because it allows for different command rates
void PS3Controller::Loop()
{
  ros::Rate rate(rt); // MUST have this rate in here, since all factors are based on it
  while (ros::ok())
  {
    if (!isReset)
    {
      PS3Controller::UpdateCommands();
      PS3Controller::PublishCommands();
    }
    ros::spinOnce();
    rate.sleep();
  }
}