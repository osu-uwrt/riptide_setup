#include "riptide_controllers/command_combinator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "command_combinator");
  CommandCombinator cc;
  try
  {
    ros::spin();
  }
  catch (exception &e)
  {
    ROS_ERROR("Comm Comb Error: %s", e.what());
    ROS_ERROR("Comm Comb: Shutting Down");
  }
}

CommandCombinator::CommandCombinator() : nh("command_combinator")
{
  x_sub = nh.subscribe<std_msgs::Float64>("/command/accel_x", 1, &CommandCombinator::XCB, this);
  y_sub = nh.subscribe<std_msgs::Float64>("/command/accel_y", 1, &CommandCombinator::YCB, this);
  z_sub = nh.subscribe<std_msgs::Float64>("/command/accel_z", 1, &CommandCombinator::ZCB, this);
  depth_sub = nh.subscribe<geometry_msgs::Vector3>("/command/accel_depth", 1, &CommandCombinator::DepthCB, this);
  angular_sub = nh.subscribe<geometry_msgs::Vector3>("/command/accel_angular", 1, &CommandCombinator::AngularCB, this);

  cmd_pub = nh.advertise<geometry_msgs::Accel>("/command/accel", 1); // The final accel output

  CommandCombinator::LoadParam<double>("max_x_accel", MAX_X_ACCEL);
  CommandCombinator::LoadParam<double>("max_y_accel", MAX_Y_ACCEL);
  CommandCombinator::LoadParam<double>("max_z_accel", MAX_Z_ACCEL);
  CommandCombinator::LoadParam<double>("max_roll_accel", MAX_ROLL_ACCEL);
  CommandCombinator::LoadParam<double>("max_pitch_accel", MAX_PITCH_ACCEL);
  CommandCombinator::LoadParam<double>("max_yaw_accel", MAX_YAW_ACCEL);

  CommandCombinator::InitMsgs();
}

// Load parameter from namespace
template <typename T>
void CommandCombinator::LoadParam(string param, T &var)
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
    ROS_ERROR("Command Combinator Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void CommandCombinator::InitMsgs()
{
  cmd_accel.linear.x = 0;
  cmd_accel.linear.y = 0;
  cmd_accel.linear.z = 0;
  cmd_accel.angular.x = 0;
  cmd_accel.angular.y = 0;
  cmd_accel.angular.z = 0;

  accel.linear.x = 0;
  accel.linear.y = 0;
  accel.linear.z = 0;
  accel.angular.x = 0;
  accel.angular.y = 0;
  accel.angular.z = 0;

  depth_accel.x = 0;
  depth_accel.y = 0;
  depth_accel.z = 0;
}

void CommandCombinator::XCB(const std_msgs::Float64::ConstPtr &x_accel)
{
  accel.linear.x = x_accel->data;

  CommandCombinator::Combine();
  cmd_pub.publish(cmd_accel);
}

void CommandCombinator::YCB(const std_msgs::Float64::ConstPtr &y_accel)
{
  accel.linear.y = y_accel->data;

  CommandCombinator::Combine();
  cmd_pub.publish(cmd_accel);
}

void CommandCombinator::ZCB(const std_msgs::Float64::ConstPtr &z_accel)
{
  accel.linear.z = z_accel->data;

  CommandCombinator::Combine();
  cmd_pub.publish(cmd_accel);
}

void CommandCombinator::DepthCB(const geometry_msgs::Vector3::ConstPtr &d_accel)
{
  depth_accel.x = d_accel->x;
  depth_accel.y = d_accel->y;
  depth_accel.z = d_accel->z;

  CommandCombinator::Combine();
  cmd_pub.publish(cmd_accel);
}

void CommandCombinator::AngularCB(const geometry_msgs::Vector3::ConstPtr &ang_accel)
{
  accel.angular.x = ang_accel->x;
  accel.angular.y = ang_accel->y;
  accel.angular.z = ang_accel->z;

  CommandCombinator::Combine();
  cmd_pub.publish(cmd_accel);
}

void CommandCombinator::Combine()
{
  cmd_accel.linear.x = accel.linear.x + depth_accel.x;
  cmd_accel.linear.y = accel.linear.y + depth_accel.y;
  cmd_accel.linear.z = accel.linear.z + depth_accel.z;
  cmd_accel.angular.x = accel.angular.x;
  cmd_accel.angular.y = accel.angular.y;
  cmd_accel.angular.z = accel.angular.z;

  cmd_accel.linear.x = CommandCombinator::Constrain(cmd_accel.linear.x, MAX_X_ACCEL);
  cmd_accel.linear.y = CommandCombinator::Constrain(cmd_accel.linear.y, MAX_Y_ACCEL);
  cmd_accel.linear.z = CommandCombinator::Constrain(cmd_accel.linear.z, MAX_Z_ACCEL);
  cmd_accel.angular.x = CommandCombinator::Constrain(cmd_accel.angular.x, MAX_ROLL_ACCEL);
  cmd_accel.angular.y = CommandCombinator::Constrain(cmd_accel.angular.y, MAX_PITCH_ACCEL);
  cmd_accel.angular.z = CommandCombinator::Constrain(cmd_accel.angular.z, MAX_YAW_ACCEL);
}

double CommandCombinator::Constrain(double current, double max)
{
  if (current > max)
    return max;
  else if (current < -1 * max)
    return -1 * max;
  return current;
}
