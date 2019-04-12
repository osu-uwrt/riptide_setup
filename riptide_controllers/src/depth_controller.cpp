#include "riptide_controllers/depth_controller.h"

#undef debug
#undef report
#undef progress

#define PI 3.141592653
#define MIN_DEPTH 0
#define RESET_ID 0
#define DISABLE_ID 1

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_controller");
  DepthController dc;
  try
  {
    ros::spin();
  }
  catch (exception &e)
  {
    ROS_ERROR("Depth Error: %s", e.what());
    ROS_ERROR("Depth: Shutting Down");
  }
}

DepthController::DepthController() : nh("~")
{
  ros::NodeHandle dcpid("depth_controller");
  R_b2w.setIdentity();
  R_w2b.setIdentity();
  tf.setValue(0, 0, 0);

  depth_controller_pid.init(dcpid, false);

  cmd_sub = nh.subscribe<riptide_msgs::DepthCommand>("/command/depth", 1, &DepthController::CommandCB, this);
  depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &DepthController::DepthCB, this);
  imu_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &DepthController::ImuCB, this);

  cmd_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/command/force_depth", 1);
  status_pub = nh.advertise<riptide_msgs::ControlStatus>("/status/controls/depth", 1);

  DepthController::LoadParam<double>("max_depth", MAX_DEPTH);
  DepthController::LoadParam<double>("max_depth_error", MAX_DEPTH_ERROR);
  DepthController::LoadParam<double>("PID_IIR_LPF_bandwidth", PID_IIR_LPF_bandwidth);
  DepthController::LoadParam<double>("sensor_rate", sensor_rate);

  pid_depth_active = false;
  current_depth = 0;

  // IIR LPF Variables
  double fc = PID_IIR_LPF_bandwidth; // Shorthand variable for IIR bandwidth
  dt_iir = 1.0 / sensor_rate;
  alpha = 2 * PI * dt_iir * fc / (2 * PI * dt_iir * fc + 1); // Multiplier

  sample_start = ros::Time::now();

  status_msg.current = current_depth;
  DepthController::ResetDepth();
}

// Load parameter from namespace
template <typename T>
void DepthController::LoadParam(string param, T &var)
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
    ROS_ERROR("Depth Controller Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void DepthController::UpdateError()
{
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  if (pid_depth_active)
  {
    depth_error = depth_cmd - current_depth;
    depth_error = DepthController::Constrain(depth_error, MAX_DEPTH_ERROR);
    depth_error_dot = (depth_error - last_error) / dt;
    depth_error_dot = DepthController::SmoothErrorIIR(depth_error_dot, last_error_dot);
    last_error = depth_error;
    last_error_dot = depth_error_dot;
    status_msg.error = depth_error;

    output = depth_controller_pid.computeCommand(depth_error, depth_error_dot, sample_duration);

    cmd_force.header.stamp = ros::Time::now();
    cmd_force.vector.x = -output * sin(theta);
    cmd_force.vector.y = output * sin(phi) * cos(theta);
    cmd_force.vector.z = output * cos(phi) * cos(theta);
    cmd_pub.publish(cmd_force);

    status_msg.header.stamp = ros::Time::now();
    status_pub.publish(status_msg);
  }
  sample_start = ros::Time::now();
}

// Constrain physical error. This acts as a way to break up the motion into
// smaller segments. Otherwise, if error is too large, the vehicle would move
// too quickly in the water and exhibit large overshoot
double DepthController::Constrain(double current, double max)
{
  if (current > max)
    return max;
  else if (current < -1 * max)
    return -1 * max;
  return current;
}

// Apply IIR LPF to depth error_dot
double DepthController::SmoothErrorIIR(double input, double prev)
{
  return (alpha * input + (1 - alpha) * prev);
}

// Subscribe to manual depth command
void DepthController::CommandCB(const riptide_msgs::DepthCommand::ConstPtr &cmd)
{
  pid_depth_active = cmd->active;
  if (pid_depth_active)
  {
    depth_cmd = cmd->depth;
    depth_cmd = DepthController::Constrain(depth_cmd, MAX_DEPTH);
    if (depth_cmd < 0) // Min. depth is zero
      depth_cmd = 0;
    status_msg.reference = depth_cmd;
  }
  else
  {
    DepthController::ResetDepth(); // Should not execute consecutive times
  }
}

// Subscribe to state/depth
void DepthController::DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg)
{
  current_depth = depth_msg->depth;
  status_msg.current = current_depth;
  DepthController::UpdateError();
}

// Create rotation matrix from IMU orientation
void DepthController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg)
{
  phi = imu_msg->rpy_deg.x * PI / 180;
  theta = imu_msg->rpy_deg.y * PI / 180;
  DepthController::UpdateError();
}

// Should not execute consecutive times
void DepthController::ResetDepth()
{
  depth_controller_pid.reset();
  depth_cmd = 0;
  depth_error = 0;
  depth_error_dot = 0;
  last_error = 0;
  last_error_dot = 0;

  status_msg.reference = 0;
  status_msg.error = 0;
  status_msg.header.stamp = ros::Time::now();
  status_pub.publish(status_msg);

  output = 0;
  cmd_force.header.stamp = ros::Time::now();
  cmd_force.vector.x = 0;
  cmd_force.vector.y = 0;
  cmd_force.vector.z = 0;
  cmd_pub.publish(cmd_force);
}
