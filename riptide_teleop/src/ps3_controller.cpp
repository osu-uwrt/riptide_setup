#include "riptide_teleop/ps3_controller.h"

#define GRAVITY 9.81 // [m/s^2]
#define WATER_DENSITY 1000 // [kg/m^3]

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ps3_control");
  PS3Controller ps3;
  ps3.Loop();
}

PS3Controller::PS3Controller()
{
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, &PS3Controller::JoyCB, this);
  attitude_pub = nh.advertise<geometry_msgs::Vector3>("command/manual/attitude", 1);
  lin_accel_pub = nh.advertise<geometry_msgs::Vector3>("command/manual/accel/linear", 1);
  depth_pub = nh.advertise<riptide_msgs::DepthCommand>("command/manual/depth", 1);
  reset_pub = nh.advertise<riptide_msgs::ResetControls>("controls/reset", 1);

  nh.param<int>("/ps3_controller/rate", rt, 100); // [Hz]
  nh.param<bool>("/ps3_controller/depth_status", isDepthWorking, true); // Is depth sensor working?
  PS3Controller::LoadProperty("Mass", mass); // [kg]
  PS3Controller::LoadProperty("Volume", volume); // [m^3]
  PS3Controller::LoadProperty("max_roll_limit", MAX_ROLL); // [m/s^2]
  PS3Controller::LoadProperty("max_pitch_limit", MAX_PITCH); // [m/s^2]
  PS3Controller::LoadProperty("max_x_accel", MAX_XY_ACCEL); // [m/s^2]
  PS3Controller::LoadProperty("max_z_accel", MAX_Z_ACCEL); // [m/s^2]
  PS3Controller::LoadProperty("max_depth", MAX_DEPTH); // [m]
  PS3Controller::LoadProperty("cmd_roll_rate", CMD_ROLL_RATE); // [deg/s]
  PS3Controller::LoadProperty("cmd_pitch_rate", CMD_PITCH_RATE); // [deg/s]
  PS3Controller::LoadProperty("cmd_yaw_rate", CMD_YAW_RATE); // [deg/s]
  PS3Controller::LoadProperty("cmd_depth_rate", CMD_DEPTH_RATE); // [deg/s]

  isReset = false;
  isStarted = false;
  isInit = false;

  roll_factor = CMD_ROLL_RATE/rt;
  pitch_factor = CMD_PITCH_RATE/rt;
  yaw_factor = CMD_YAW_RATE/rt;
  depth_factor = CMD_DEPTH_RATE/rt;

  stable_z_accel = (mass - volume*WATER_DENSITY) * GRAVITY / mass;

  PS3Controller::InitMsgs();
}

void PS3Controller::InitMsgs() {
  reset_msg.reset_surge = true;
  reset_msg.reset_sway = true;
  reset_msg.reset_heave = true;
  reset_msg.reset_roll = true;
  reset_msg.reset_pitch = true;
  reset_msg.reset_yaw = true;
  reset_msg.reset_depth = true;
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

// Load property from namespace
void PS3Controller::LoadProperty(std::string name, double &param)
{
  try
  {
    if (!nh.getParam("/ps3_controller/" + name, param))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    ROS_ERROR("Critical! PS3 Controller has no property set for %s. Shutting down...", name.c_str());
    ros::shutdown();
  }
}

void PS3Controller::JoyCB(const sensor_msgs::Joy::ConstPtr& joy) {
  if(joy->buttons[BUTTON_SHAPE_X]) { // Reset button (The "X") pressed
    isReset = true;
    isStarted = false;
    reset_msg.reset_depth = true;
    reset_msg.reset_roll = true;
    reset_msg.reset_pitch = true;
    reset_msg.reset_yaw = true;
    reset_msg.reset_pwm = true;
  }
  else if(isReset) { // If reset, must wait for Start button to be pressed
    if(joy->axes[BUTTON_START])
      isStarted = true;
      isReset = false;
      reset_msg.reset_depth = false;
      reset_msg.reset_roll = false;
      reset_msg.reset_pitch = false;
      reset_msg.reset_yaw = false;
      reset_msg.reset_pwm = false;

      delta_attitude.x = 0;
      delta_attitude.y = 0;
      delta_attitude.z = 0;
      delta_depth = 0;
  }
  else if(isStarted) {
    if(!isInit) { // Initialize to roll and pitch of 0 [deg], and set yaw to current heading
      isInit = true;
      cmd_attitude.x = 0;
      cmd_attitude.y = 0;
      cmd_attitude.z = tf.z();
      cmd_depth.absolute = 0.2;
      cmd_accel.x = 0;
      cmd_accel.y = 0;
      cmd_accel.z = 0;
    }
    else if(isInit) {
      delta_attitude.x = -joy->axes[AXES_STICK_LEFT_LR]*roll_factor; // Right -> inc roll, Left -> dec roll
      delta_attitude.y = joy->axes[AXES_STICK_LEFT_UD]*pitch_factor; // Up -> inc pitch, Down -> dec pitch

      if(joy->buttons[BUTTON_CROSS_RIGHT])
        delta_attitude.z = -joy->buttons[BUTTON_CROSS_RIGHT]*yaw_factor; // Yaw CW rotation
      else if(joy->buttons[BUTTON_CROSS_LEFT])
        delta_attitude.z = joy->buttons[BUTTON_CROSS_LEFT]*yaw_factor; // Yaw CCW rotation

      if(isDepthWorking) { // Depth sensor working properly
        if(joy->buttons[BUTTON_CROSS_UP]) // Up -> dec depth (moves upwards)
          delta_depth = -depth_factor;
        else if(joy->buttons[BUTTON_CROSS_DOWN]) // Down -> inc depth (moves downward)
          delta_depth = depth_factor;
        }
      else { // Depth sensor not working properly
        cmd_accel.z = stable_z_accel;
        if(1 - joy->axes[AXES_REAR_R2] != 0) // If pressed at all, inc accel
          cmd_accel.z += 0.5*(1 - joy->axes[AXES_REAR_R2])*MAX_Z_ACCEL; // Multiplied by 0.5 to scale to from axis 0 to 1
        else if(1 - joy->axes[AXES_REAR_L2] != 0) // If pressed at all, dec accel
          cmd_accel.z -= 0.5*(1 - joy->axes[AXES_REAR_L2])*MAX_Z_ACCEL; // Multiplied by 0.5 to scale to from axis 0 to 1
      }

      cmd_accel.x = joy->axes[AXES_STICK_RIGHT_UD]*MAX_XY_ACCEL; // Surge pos. forward
      cmd_accel.y = joy->axes[AXES_STICK_RIGHT_LR]*MAX_XY_ACCEL; // Sway pos. left
    }
  }

}

double PS3Controller::Constrain(double current, double max) {
  if(current > max)
    return max;
  else if(current < -1*max)
    return -1*max;
  return current;
}

// Create rotation matrix from IMU orientation
void PS3Controller::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg) {
  vector3MsgToTF(imu_msg->euler_rpy, tf);
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

  cmd_depth.absolute += delta_depth;
  if(cmd_depth.absolute < 0.1)
    cmd_depth.absolute = 0.1;
  else if(cmd_depth.absolute > MAX_DEPTH)
    cmd_depth.absolute = MAX_DEPTH;

}

void PS3Controller::PublishCommands() {
  reset_pub.publish(reset_msg);
  attitude_pub.publish(cmd_attitude);
  lin_accel_pub.publish(cmd_accel);
  if(isDepthWorking)
    depth_pub.publish(cmd_depth);
}

void PS3Controller::Loop()
{
  ros::Rate rate(rt);
  while (ros::ok())
  {
    PS3Controller::UpdateCommands();
    PS3Controller::PublishCommands();
    ros::spinOnce();
    rate.sleep();
  }
}
