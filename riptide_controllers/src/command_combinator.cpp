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
  x_sub = nh.subscribe<std_msgs::Float64>("/command/force_x", 1, &CommandCombinator::XCB, this);
  y_sub = nh.subscribe<std_msgs::Float64>("/command/force_y", 1, &CommandCombinator::YCB, this);
  z_sub = nh.subscribe<std_msgs::Float64>("/command/force_z", 1, &CommandCombinator::ZCB, this);
  depth_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/command/force_depth", 1, &CommandCombinator::DepthCB, this);
  moment_sub = nh.subscribe<geometry_msgs::Vector3Stamped>("/command/moment", 1, &CommandCombinator::MomentCB, this);

  cmd_pub = nh.advertise<riptide_msgs::NetLoad>("/command/net_load", 1); // The final net load

  CommandCombinator::LoadParam<double>("max_x_force", MAX_X_FORCE);
  CommandCombinator::LoadParam<double>("max_y_force", MAX_Y_FORCE);
  CommandCombinator::LoadParam<double>("max_z_force", MAX_Z_FORCE);
  CommandCombinator::LoadParam<double>("max_x_moment", MAX_X_MOMENT);
  CommandCombinator::LoadParam<double>("max_y_moment", MAX_Y_MOMENT);
  CommandCombinator::LoadParam<double>("max_z_moment", MAX_Z_MOMENT);

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
  force.x = 0;
  force.y = 0;
  force.z = 0;
  force_depth.x = 0;
  force_depth.y = 0;
  force_depth.z = 0;

  cmd_load.force.x = 0;
  cmd_load.force.y = 0;
  cmd_load.force.z = 0;
  cmd_load.moment.x = 0;
  cmd_load.moment.y = 0;
  cmd_load.moment.z = 0;
}

void CommandCombinator::XCB(const std_msgs::Float64::ConstPtr &force_x)
{
  force.x = force_x->data;
  cmd_load.force.x = force.x + force_depth.x;
  cmd_load.force.x = CommandCombinator::Constrain(cmd_load.force.x, MAX_X_FORCE);
  cmd_load.header.stamp = ros::Time::now();
  cmd_pub.publish(cmd_load);
}

void CommandCombinator::YCB(const std_msgs::Float64::ConstPtr &force_y)
{
  force.y = force_y->data;
  cmd_load.force.y = force.y + force_depth.y;
  cmd_load.force.y = CommandCombinator::Constrain(cmd_load.force.y, MAX_Y_FORCE);
  cmd_load.header.stamp = ros::Time::now();
  cmd_pub.publish(cmd_load);
}

void CommandCombinator::ZCB(const std_msgs::Float64::ConstPtr &force_z)
{
  force.z = force_z->data;
  cmd_load.force.z = force.z + force_depth.z;
  cmd_load.force.z = CommandCombinator::Constrain(cmd_load.force.z, MAX_Z_FORCE);
  cmd_load.header.stamp = ros::Time::now();
  cmd_pub.publish(cmd_load);
}

void CommandCombinator::DepthCB(const geometry_msgs::Vector3Stamped::ConstPtr &force_msg)
{
  force_depth.x = force_msg->vector.x;
  force_depth.y = force_msg->vector.y;
  force_depth.z = force_msg->vector.z;

  cmd_load.force.x = force.x + force_depth.x;
  cmd_load.force.y = force.y + force_depth.y;
  cmd_load.force.z = force.z + force_depth.z;

  cmd_load.force.x = CommandCombinator::Constrain(cmd_load.force.x, MAX_X_FORCE);
  cmd_load.force.y = CommandCombinator::Constrain(cmd_load.force.y, MAX_Y_FORCE);
  cmd_load.force.z = CommandCombinator::Constrain(cmd_load.force.z, MAX_Z_FORCE);
  cmd_load.header.stamp = ros::Time::now();
  cmd_pub.publish(cmd_load);
}

void CommandCombinator::MomentCB(const geometry_msgs::Vector3Stamped::ConstPtr &moment_msg)
{
  cmd_load.moment.x = moment_msg->vector.x;
  cmd_load.moment.y = moment_msg->vector.y;
  cmd_load.moment.z = moment_msg->vector.z;

  cmd_load.moment.x = CommandCombinator::Constrain(cmd_load.moment.x, MAX_X_MOMENT);
  cmd_load.moment.y = CommandCombinator::Constrain(cmd_load.moment.y, MAX_Y_MOMENT);
  cmd_load.moment.z = CommandCombinator::Constrain(cmd_load.moment.z, MAX_Z_MOMENT);
  cmd_load.header.stamp = ros::Time::now();
  cmd_pub.publish(cmd_load);
}

double CommandCombinator::Constrain(double current, double max)
{
  if (current > max)
    return max;
  else if (current < -1 * max)
    return -1 * max;
  return current;
}
