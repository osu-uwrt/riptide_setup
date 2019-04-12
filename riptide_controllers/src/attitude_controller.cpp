#include "riptide_controllers/attitude_controller.h"

#undef debug
#undef report
#undef progress

#define PI 3.141592653
#define RESET_ID 0
#define DISABLE_ID 1

float round(float d)
{
  return floor(d + 0.5);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "attitude_controller");
  AttitudeController ac;
  try
  {
    ros::spin();
  }
  catch (exception &e)
  {
    ROS_ERROR("Attitude Error: %s", e.what());
    ROS_ERROR("Attitude: Shutting Down");
  }
}

AttitudeController::AttitudeController() : nh("~")
{
  ros::NodeHandle rcpid("roll_controller");
  ros::NodeHandle ycpid("yaw_controller");
  ros::NodeHandle pcpid("pitch_controller");

  roll_controller_pid.init(rcpid, false);
  yaw_controller_pid.init(ycpid, false);
  pitch_controller_pid.init(pcpid, false);

  cmd_sub = nh.subscribe<riptide_msgs::AttitudeCommand>("/command/attitude", 1, &AttitudeController::CommandCB, this);
  imu_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &AttitudeController::ImuCB, this);

  cmd_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/command/moment", 1);
  status_pub = nh.advertise<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1);

  AttitudeController::LoadParam<double>("max_roll_error", MAX_ROLL_ERROR);
  AttitudeController::LoadParam<double>("max_pitch_error", MAX_PITCH_ERROR);
  AttitudeController::LoadParam<double>("max_yaw_error", MAX_YAW_ERROR);
  AttitudeController::LoadParam<double>("max_roll_limit", MAX_ROLL_LIMIT);
  AttitudeController::LoadParam<double>("max_pitch_limit", MAX_PITCH_LIMIT);
  AttitudeController::LoadParam<double>("PID_IIR_LPF_bandwidth", PID_IIR_LPF_bandwidth);
  AttitudeController::LoadParam<double>("imu_filter_rate", imu_filter_rate);

  pid_roll_active = false;
  pid_pitch_active = false;
  pid_yaw_active = false;
  pid_attitude_active = false;

  // IIR LPF Variables
  double fc = PID_IIR_LPF_bandwidth; // Shorthand variable for IIR bandwidth
  dt_iir = 1.0 / imu_filter_rate;
  alpha = 2 * PI * dt_iir * fc / (2 * PI * dt_iir * fc + 1); // Multiplier

  sample_start = ros::Time::now();

  //AttitudeController::InitMsgs();
  status_msg.roll.current = 0;
  status_msg.pitch.current = 0;
  status_msg.yaw.current = 0;
  AttitudeController::ResetRoll();
  AttitudeController::ResetPitch();
  AttitudeController::ResetYaw();
}

// Load parameter from namespace
template <typename T>
void AttitudeController::LoadParam(string param, T &var)
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
    ROS_ERROR("Attitude Controller Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void AttitudeController::UpdateError()
{
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  // Roll error
  if (pid_roll_active)
  {
    roll_error = roll_cmd - current_attitude.x;
    roll_error = AttitudeController::Constrain(roll_error, MAX_ROLL_ERROR);
    roll_error_dot = -ang_vel.x; // Negative sign is necesary to account for correct sign of error_dot
    last_error.x = roll_error;
    status_msg.roll.error = roll_error;

    cmd_moment.vector.x = roll_controller_pid.computeCommand(roll_error, roll_error_dot, sample_duration);
  }

  // Pitch error
  if (pid_pitch_active)
  {
    pitch_error = pitch_cmd - current_attitude.y;
    pitch_error = AttitudeController::Constrain(pitch_error, MAX_PITCH_ERROR);
    pitch_error_dot = -ang_vel.y; // Negative sign is necesary to account for correct sign of error_dot
    last_error.y = pitch_error;
    status_msg.pitch.error = pitch_error;

    cmd_moment.vector.y = pitch_controller_pid.computeCommand(pitch_error, pitch_error_dot, sample_duration);
  }

  // Yaw error
  if (pid_yaw_active)
  {
    // Always take shortest path to setpoint
    yaw_error = yaw_cmd - current_attitude.z;
    if (yaw_error > 180)
      yaw_error -= 360;
    else if (yaw_error < -180)
      yaw_error += 360;
    yaw_error = AttitudeController::Constrain(yaw_error, MAX_YAW_ERROR);

    yaw_error_dot = -ang_vel.z; // Negative sign is necesary to account for correct sign of error_dot
    last_error.z = yaw_error;
    status_msg.yaw.error = yaw_error;

    cmd_moment.vector.z = yaw_controller_pid.computeCommand(yaw_error, yaw_error_dot, sample_duration);
  }

  if (pid_attitude_active)
  {
    cmd_moment.header.stamp = ros::Time::now();
    cmd_pub.publish(cmd_moment);
    status_msg.header.stamp = sample_start;
    status_pub.publish(status_msg);
  }

  sample_start = ros::Time::now();
}

// Constrain physical error. This acts as a way to break up the motion into
// smaller segments. Otherwise, if error is too large, the vehicle would move
// too quickly in the water in exhibit large overshoot
double AttitudeController::Constrain(double current, double max)
{
  if (current > max)
    return max;
  else if (current < -1 * max)
    return -1 * max;
  return current;
}

// Apply IIR LPF to error_dot
double AttitudeController::SmoothErrorIIR(double input, double prev)
{
  return (alpha * input + (1 - alpha) * prev);
}

void AttitudeController::CommandCB(const riptide_msgs::AttitudeCommand::ConstPtr &cmd)
{
  pid_roll_active = cmd->roll_active;
  pid_pitch_active = cmd->pitch_active;
  pid_yaw_active = cmd->yaw_active;

  if (pid_roll_active)
  {
    roll_cmd = cmd->euler_rpy.x;
    roll_cmd = AttitudeController::Constrain(roll_cmd, MAX_ROLL_LIMIT);
    status_msg.roll.reference = roll_cmd;
  }
  else
  {
    AttitudeController::ResetRoll(); // Should not execute consecutive times
  }

  if (pid_pitch_active)
  {
    pitch_cmd = cmd->euler_rpy.y;
    pitch_cmd = AttitudeController::Constrain(pitch_cmd, MAX_PITCH_LIMIT);
    status_msg.pitch.reference = pitch_cmd;
  }
  else
  {
    AttitudeController::ResetPitch(); // Should not execute consecutive times
  }

  if (pid_yaw_active)
  {
    yaw_cmd = cmd->euler_rpy.z;
    status_msg.yaw.reference = yaw_cmd;
  }
  else
  {
    AttitudeController::ResetYaw(); // Should not execute consecutive times
  }

  if (pid_roll_active || pid_pitch_active || pid_yaw_active)
    pid_attitude_active = true; // Only need one to be active
  else
    pid_attitude_active = false;
}

// Subscribe to state/imu
void AttitudeController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg)
{
  current_attitude = imu_msg->rpy_deg;
  status_msg.roll.current = current_attitude.x;
  status_msg.pitch.current = current_attitude.y;
  status_msg.yaw.current = current_attitude.z;

  //Get angular velocity (leave in [deg/s])
  ang_vel = imu_msg->ang_vel_deg;
  AttitudeController::UpdateError();
}

// Should not execute consecutive times
void AttitudeController::ResetRoll()
{
  roll_controller_pid.reset();
  roll_cmd = 0;
  roll_error = 0;
  roll_error_dot = 0;
  last_error.x = 0;

  status_msg.roll.reference = 0;
  status_msg.roll.error = 0;
  status_msg.header.stamp = ros::Time::now();
  status_pub.publish(status_msg);

  cmd_moment.header.stamp = ros::Time::now();
  cmd_moment.vector.x = 0;
  cmd_pub.publish(cmd_moment);
}

// Should not execute consecutive times
void AttitudeController::ResetPitch()
{
  pitch_controller_pid.reset();
  pitch_cmd = 0;
  pitch_error = 0;
  pitch_error_dot = 0;
  last_error.y = 0;

  status_msg.pitch.reference = 0;
  status_msg.pitch.error = 0;
  status_msg.header.stamp = ros::Time::now();
  status_pub.publish(status_msg);

  cmd_moment.header.stamp = ros::Time::now();
  cmd_moment.vector.y = 0;
  cmd_pub.publish(cmd_moment);
}

// Should not execute consecutive times
void AttitudeController::ResetYaw()
{
  yaw_controller_pid.reset();
  yaw_cmd = 0;
  yaw_error = 0;
  yaw_error_dot = 0;
  last_error.z = 0;

  status_msg.yaw.reference = 0;
  status_msg.yaw.error = 0;
  status_msg.header.stamp = ros::Time::now();
  status_pub.publish(status_msg);

  cmd_moment.header.stamp = ros::Time::now();
  cmd_moment.vector.z = 0;
  cmd_pub.publish(cmd_moment);
}
