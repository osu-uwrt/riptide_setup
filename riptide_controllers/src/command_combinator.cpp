#include "riptide_controllers/command_combinator.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "command_combinator");
  CommandCombinator cc;
  cc.Loop();
}

CommandCombinator::CommandCombinator() : nh("command_combinator") {
  auto_linear_sub = nh.subscribe<geometry_msgs::Vector3>("/command/auto/accel/linear", 1, &CommandCombinator::AutoLinearCB, this);
  manual_linear_sub = nh.subscribe<geometry_msgs::Vector3>("/command/manual/accel/linear", 1, &CommandCombinator::ManualLinearCB, this);
  depth_sub = nh.subscribe<geometry_msgs::Vector3>("/command/auto/accel/depth", 1, &CommandCombinator::DepthCB, this);
  auto_angular_sub = nh.subscribe<geometry_msgs::Vector3>("/command/auto/accel/angular", 1, &CommandCombinator::AutoAngularCB, this);
  manual_angular_sub = nh.subscribe<geometry_msgs::Vector3>("/command/manual/accel/angular", 1, &CommandCombinator::ManualAngularCB, this);

  cmd_pub = nh.advertise<geometry_msgs::Accel>("/command/accel", 1); // The final accel output

  CommandCombinator::LoadProperty("max_x_accel", MAX_X_ACCEL);
  CommandCombinator::LoadProperty("max_y_accel", MAX_Y_ACCEL);
  CommandCombinator::LoadProperty("max_z_accel", MAX_Z_ACCEL);
  CommandCombinator::LoadProperty("max_roll_accel", MAX_ROLL_ACCEL);
  CommandCombinator::LoadProperty("max_pitch_accel", MAX_PITCH_ACCEL);
  CommandCombinator::LoadProperty("max_yaw_accel", MAX_YAW_ACCEL);

  CommandCombinator::InitMsgs();
}

// Load property from namespace
void CommandCombinator::LoadProperty(std::string name, double &param)
{
  try
  {
    if(!nh.getParam(name, param))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    ROS_ERROR("Critical! Command Combinator has no property set for %s. Shutting down...", name.c_str());
    ros::shutdown();
  }
}

void CommandCombinator::InitMsgs() {
  cmd_accel.linear.x = 0;
  cmd_accel.linear.y = 0;
  cmd_accel.linear.z = 0;
  cmd_accel.angular.x = 0;
  cmd_accel.angular.y = 0;
  cmd_accel.angular.z = 0;

  auto_accel.linear.x = 0;
  auto_accel.linear.y = 0;
  auto_accel.linear.z = 0;
  auto_accel.angular.x = 0;
  auto_accel.angular.y = 0;
  auto_accel.angular.z = 0;

  manual_accel.linear.x = 0;
  manual_accel.linear.y = 0;
  manual_accel.linear.z = 0;
  manual_accel.angular.x = 0;
  manual_accel.angular.y = 0;
  manual_accel.angular.z = 0;

  depth_accel.x = 0;
  depth_accel.y = 0;
  depth_accel.z = 0;
}

void CommandCombinator::AutoLinearCB(const geometry_msgs::Vector3::ConstPtr &lin_accel) {
  // NOTE: There is no auto_accel.linear.z b/c that goes straight to the depth controller
  auto_accel.linear.x = lin_accel->x;
  auto_accel.linear.y = lin_accel->y;
}

void CommandCombinator::ManualLinearCB(const geometry_msgs::Vector3::ConstPtr &lin_accel) {
  manual_accel.linear.x = lin_accel->x;
  manual_accel.linear.y = lin_accel->y;
  manual_accel.linear.z = lin_accel->z;
}

void CommandCombinator::DepthCB(const geometry_msgs::Vector3::ConstPtr &d_accel) {
  depth_accel.x = d_accel->x;
  depth_accel.y = d_accel->y;
  depth_accel.z = d_accel->z;
}

void CommandCombinator::AutoAngularCB(const geometry_msgs::Vector3::ConstPtr &ang_accel) {
  auto_accel.angular.x = ang_accel->x;
  auto_accel.angular.y = ang_accel->y;
  auto_accel.angular.z = ang_accel->z;
}

void CommandCombinator::ManualAngularCB(const geometry_msgs::Vector3::ConstPtr &ang_accel) {
  manual_accel.angular.x = ang_accel->x;
  manual_accel.angular.y = ang_accel->y;
  manual_accel.angular.z = ang_accel->z;
}

void CommandCombinator::Combine() {
  cmd_accel.linear.x = auto_accel.linear.x + manual_accel.linear.x + depth_accel.x;
  cmd_accel.linear.y = auto_accel.linear.y + manual_accel.linear.y + depth_accel.y;
  cmd_accel.linear.z = auto_accel.linear.z + manual_accel.linear.z + depth_accel.z;
  cmd_accel.angular.x = auto_accel.angular.x + manual_accel.angular.x;
  cmd_accel.angular.y = auto_accel.angular.y + manual_accel.angular.y;
  cmd_accel.angular.z = auto_accel.angular.z + manual_accel.angular.z;

  cmd_accel.linear.x = CommandCombinator::Constrain(cmd_accel.linear.x, MAX_X_ACCEL);
  cmd_accel.linear.y = CommandCombinator::Constrain(cmd_accel.linear.y, MAX_Y_ACCEL);
  cmd_accel.linear.z = CommandCombinator::Constrain(cmd_accel.linear.z, MAX_Z_ACCEL);
  cmd_accel.angular.x = CommandCombinator::Constrain(cmd_accel.angular.x, MAX_ROLL_ACCEL);
  cmd_accel.angular.y = CommandCombinator::Constrain(cmd_accel.angular.y, MAX_PITCH_ACCEL);
  cmd_accel.angular.z = CommandCombinator::Constrain(cmd_accel.angular.z, MAX_YAW_ACCEL);
}

double CommandCombinator::Constrain(double current, double max) {
  if(current > max)
    return max;
  else if(current < -1*max)
    return -1*max;
  return current;
}

void CommandCombinator::Loop() {
  while(!ros::isShuttingDown()) {
    CommandCombinator::Combine();
    cmd_pub.publish(cmd_accel); // ALWAYS publish a message, regardless of circumstance
    ros::spinOnce();
  }
}
