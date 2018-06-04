#include "riptide_teleop/ps3_controller.h"

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
  //depth_pub = nh.advertise<riptide_msgs::DepthCommand>("command/manual/depth", 1);
  reset_pub = nh.advertise<riptide_msgs::ResetControls>("controls/reset", 1);

  nh.param("/ps3_controller/rate", rt, 100); // [Hz]
  PS3Controller::LoadProperty("max_roll_limit", MAX_ROLL); // [m/s^2]
  PS3Controller::LoadProperty("max_pitch_limit", MAX_PITCH); // [m/s^2]
  PS3Controller::LoadProperty("max_x_accel", MAX_XY_ACCEL); // [m/s^2]
  PS3Controller::LoadProperty("max_z_accel", MAX_Z_ACCEL); // [m/s^2]
  PS3Controller::LoadProperty("max_depth", MAX_DEPTH); // [m/s^2]

  isReset = false;
  isStarted = false;
  isInit = false;

  roll_factor = 0.25*MAX_ROLL*rt;
  pitch_factor = 0.25*MAX_PITCH*rt;
  yaw_factor = 0.5* MAX_ROLL*rt; // Use max_roll since it's about 30 deg
  depth_factor = 0.5*MAX_DEPTH*rt;

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
  cmd_att_total.x = 0;
  cmd_att_total.y = 0;
  cmd_att_total.z = 0;

  cmd_accel.x = 0;
  cmd_accel.y = 0;
  cmd_accel.z = 0;

  cmd_depth.isManual = true;
  cmd_depth.absolute = 0;
  cmd_depth.relative = 0;
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
  if(joy->axes[14]) { // Reset button (The "X") pressed
    isReset = true;
    isStarted = false;
    reset_msg.reset_depth = true;
    reset_msg.reset_roll = true;
    reset_msg.reset_pitch = true;
    reset_msg.reset_yaw = true;
    reset_msg.reset_pwm = true;
  }
  else if(isReset) { // If reset, must wait for Start button to be pressed
    if(joy->axes[3])
      isStarted = true;
      reset_msg.reset_depth = false;
      reset_msg.reset_roll = false;
      reset_msg.reset_pitch = false;
      reset_msg.reset_yaw = false;
      reset_msg.reset_pwm = false;
  }
  else if(isStarted) {
    if(!isInit) { // Initialize to roll and pitch of 0 [deg], and set
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
      cmd_attitude.x += -1*joy->axes[0]*roll_factor; // Left Stick: Right -> inc roll, Left -> dec roll
      cmd_attitude.y += joy->axes[1]*pitch_factor; // Left Stick: Up -> inc pitch, Down -> dec pitch
      cmd_attitude.z += -1*joy->buttons[5]*yaw_factor; // Cross Right -> CW rotation
      cmd_attitude.z += joy->buttons[7]*yaw_factor; // Cross Left -> CCW rotation

      //cmd_depth += joy->buttons[4]*depth_factor; // Cross Up -> inc depth
      //cmd_depth += -1*joy->buttons[6]*depth_factor; // Cross Down -> dec depth

      /*cmd_accel.x = joy->axes[3]*MAX_ACCEL;
      cmd_accel.y = joy->axes[2]*MAX_ACCEL;
      cmd_accel.z = joy->buttons[4]*MAX_ACCEL + 0.5;
      cmd_accel.z = -1*joy->buttons[6]*MAX_ACCEL + 0.5;

      cmd_attitude.x = PS3Controller::Constrain(cmd_attitude.x, MAX_ROLL);
      cmd_attitude.y = PS3Controller::Constrain(cmd_attitude.y, MAX_PITCH);
      if(cmd_attitude.z > 180)
        cmd_attitude.z -= 360;
      if(cmd_attitude.z < -180)
        cmd_attitude.z += 360;
      //cmd_depth = PS3Controller::Constrain(cmd_depth, MAX_DEPTH);*/

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

}

void PS3Controller::PublishCommands() {
  reset_pub.publish(reset_msg);
  attitude_pub.publish(cmd_attitude);
  lin_accel_pub.publish(cmd_accel);
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
