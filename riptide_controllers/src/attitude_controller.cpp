#include "riptide_controllers/attitude_controller.h"

#undef debug
#undef report
#undef progress

#define PI 3.141592653

float round(float d) {
  return floor(d + 0.5);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "attitude_controller");
  AttitudeController ac;
  ros::spin();
}

AttitudeController::AttitudeController() : nh("attitude_controller") {
    ros::NodeHandle rcpid("roll_controller");
    ros::NodeHandle ycpid("yaw_controller");
    ros::NodeHandle pcpid("pitch_controller");

    roll_controller_pid.init(rcpid, false);
    yaw_controller_pid.init(ycpid, false);
    pitch_controller_pid.init(pcpid, false);

    cmd_sub = nh.subscribe<riptide_msgs::AttitudeCommand>("/command/attitude", 1, &AttitudeController::CommandCB, this);
    imu_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &AttitudeController::ImuCB, this);
    reset_sub = nh.subscribe<riptide_msgs::ResetControls>("/controls/reset", 1, &AttitudeController::ResetController, this);

    cmd_pub = nh.advertise<geometry_msgs::Vector3>("/command/accel_angular", 1);
    status_pub = nh.advertise<riptide_msgs::ControlStatusAngular>("/status/controls/angular", 1);

    AttitudeController::LoadParam<double>("max_roll_error", MAX_ROLL_ERROR);
    AttitudeController::LoadParam<double>("max_pitch_error", MAX_PITCH_ERROR);
    AttitudeController::LoadParam<double>("max_yaw_error", MAX_YAW_ERROR);
    AttitudeController::LoadParam<double>("max_roll_limit", MAX_ROLL_LIMIT);
    AttitudeController::LoadParam<double>("max_pitch_limit", MAX_PITCH_LIMIT);
    AttitudeController::LoadParam<double>("PID_IIR_LPF_bandwidth", PID_IIR_LPF_bandwidth);
    AttitudeController::LoadParam<double>("imu_filter_rate", imu_filter_rate);

    pid_roll_reset = true;
    pid_pitch_reset = true;
    pid_yaw_reset = true;
    pid_attitude_reset = true;

    pid_roll_active = false;
    pid_pitch_active = false;
    pid_yaw_active = false;
    pid_attitude_active = false;

    reset_angX_sent = true;
    reset_angY_sent = true;
    reset_angZ_sent = true;
    inactive_angX_sent = true;
    inactive_angY_sent = true;
    inactive_angZ_sent = true;

    R_b2w.setIdentity();
    R_w2b.setIdentity();
    tf.setValue(0, 0, 0);
    ang_vel.setValue(0, 0, 0);

    // IIR LPF Variables
    double fc = PID_IIR_LPF_bandwidth; // Shorthand variable for IIR bandwidth
    dt_iir = 1.0/imu_filter_rate;
    alpha = 2*PI*dt_iir*fc / (2*PI*dt_iir*fc + 1); // Multiplier

    sample_start = ros::Time::now();

    AttitudeController::InitMsgs();
    AttitudeController::ResetRoll();
    AttitudeController::ResetPitch();
    AttitudeController::ResetYaw();
}

void AttitudeController::InitMsgs() {
  status_msg.roll.reference = 0;
  status_msg.roll.current = 0;
  status_msg.roll.error = 0;

  status_msg.pitch.reference = 0;
  status_msg.pitch.current = 0;
  status_msg.pitch.error = 0;

  status_msg.yaw.reference = 0;
  status_msg.yaw.current = 0;
  status_msg.yaw.error = 0;

  ang_accel_cmd.x = 0;
  ang_accel_cmd.y = 0;
  ang_accel_cmd.z = 0;
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
  catch(int e)
  {
    string ns = nh.getNamespace();
    ROS_ERROR("Attitude Controller Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void AttitudeController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  // Roll error
  if(!pid_roll_reset && pid_roll_active) {
    roll_error = roll_cmd - current_attitude.x;
    roll_error = AttitudeController::Constrain(roll_error, MAX_ROLL_ERROR);
    roll_error_dot = -ang_vel.x(); // Negative sign is necesary to account for correct sign of error_dot
    last_error.x = roll_error;
    last_error_dot.x = roll_error_dot;
    status_msg.roll.error = roll_error;

    ang_accel_cmd.x = roll_controller_pid.computeCommand(roll_error, roll_error_dot, sample_duration);
  }

  // Pitch error
  if(!pid_pitch_reset && pid_pitch_active) {
    pitch_error = pitch_cmd - current_attitude.y;
    pitch_error = AttitudeController::Constrain(pitch_error, MAX_PITCH_ERROR);
    pitch_error_dot = -ang_vel.y(); // Negative sign is necesary to account for correct sign of error_dot
    last_error.y = pitch_error;
    last_error_dot.y = pitch_error_dot;
    status_msg.pitch.error = pitch_error;

    ang_accel_cmd.y = pitch_controller_pid.computeCommand(pitch_error, pitch_error_dot, sample_duration);
  }

  // Yaw error
  if(!pid_yaw_reset && pid_yaw_active) {
    // Always take shortest path to setpoint
    yaw_error = yaw_cmd - current_attitude.z;
    if (yaw_error > 180)
        yaw_error -= 360;
    else if (yaw_error < -180)
        yaw_error += 360;
    yaw_error = AttitudeController::Constrain(yaw_error, MAX_YAW_ERROR);

    yaw_error_dot = -ang_vel.z(); // Negative sign is necesary to account for correct sign of error_dot
    last_error.z = yaw_error;
    last_error_dot.z = yaw_error_dot;
    status_msg.yaw.error = yaw_error;

    ang_accel_cmd.z = yaw_controller_pid.computeCommand(yaw_error, yaw_error_dot, sample_duration);
  }

  // ALWAYS Publish status and command messages
  cmd_pub.publish(ang_accel_cmd);
  status_msg.header.stamp = sample_start;
  status_pub.publish(status_msg);

  sample_start = ros::Time::now();
}

// Constrain physical error. This acts as a way to break up the motion into
// smaller segments. Otherwise, if error is too large, the vehicle would move
// too quickly in the water in exhibit large overshoot
double AttitudeController::Constrain(double current, double max) {
  if(current > max)
    return max;
  else if(current < -1*max)
    return -1*max;
  return current;
}

// Apply IIR LPF to error
double AttitudeController::SmoothErrorIIR(double input, double prev) {
  return (alpha*input + (1-alpha)*prev);
}

// Subscribe to state/imu
void AttitudeController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg) {
  current_attitude = imu_msg->euler_rpy;
  status_msg.roll.current = current_attitude.x;
  status_msg.pitch.current = current_attitude.y;
  status_msg.yaw.current = current_attitude.z;

  //Get angular velocity (leave in [deg/s])
  vector3MsgToTF(imu_msg->ang_vel, ang_vel);
  AttitudeController::UpdateError();
}

void AttitudeController::CommandCB(const riptide_msgs::AttitudeCommand::ConstPtr &cmd) {
  pid_roll_active = cmd->roll_active;
  pid_pitch_active = cmd->pitch_active;
  pid_yaw_active = cmd->yaw_active;

  if(!pid_roll_reset && pid_roll_active) {
    roll_cmd = cmd->euler_rpy.x;
    roll_cmd = AttitudeController::Constrain(roll_cmd, MAX_ROLL_LIMIT);
    status_msg.roll.reference = roll_cmd;
    inactive_angX_sent = false;
  }
  else {
    AttitudeController::ResetRoll();
  }

  if(!pid_pitch_reset && pid_pitch_active) {
    pitch_cmd = cmd->euler_rpy.y;
    pitch_cmd = AttitudeController::Constrain(pitch_cmd, MAX_PITCH_LIMIT);
    status_msg.pitch.reference = pitch_cmd;
    inactive_angY_sent = false;
  }
  else {
    AttitudeController::ResetPitch();
  }

  if(!pid_yaw_reset && pid_yaw_active) {
    yaw_cmd = cmd->euler_rpy.z;
    status_msg.yaw.reference = yaw_cmd;
    inactive_angZ_sent = false;
  }
  else {
    AttitudeController::ResetYaw();
  }

  if(pid_roll_active || pid_pitch_active || pid_yaw_active)
    pid_attitude_active = true; // Only need one to be active
  else
    pid_attitude_active = false;

  AttitudeController::UpdateError();
}

void AttitudeController::ResetController(const riptide_msgs::ResetControls::ConstPtr& reset_msg) {
  if(reset_msg->reset_roll) {
    pid_roll_reset = true;
    AttitudeController::ResetRoll();
  }
  else {
    pid_roll_reset = false;
    reset_angX_sent = false;
  }

  if(reset_msg->reset_pitch) {
    pid_pitch_reset = true;
    AttitudeController::ResetPitch();
  }
  else {
    pid_pitch_reset = false;
    reset_angY_sent = false;
  }

  if(reset_msg->reset_yaw) {
    pid_yaw_reset = true;
    AttitudeController::ResetYaw();
  }
  else {
    pid_yaw_reset = false;
    reset_angZ_sent = false;
  }

  if(pid_roll_reset && pid_pitch_reset && pid_yaw_reset)
    pid_attitude_reset = true;
  else
    pid_attitude_reset = false;
}

void AttitudeController::ResetRoll() {
  if(!reset_angX_sent && !inactive_angX_sent) {
    roll_controller_pid.reset();
    roll_cmd = 0;
    roll_error = 0;
    roll_error_dot = 0;
    last_error.x = 0;
    last_error_dot.x = 0;

    status_msg.roll.reference = 0;
    status_msg.roll.error = 0;

    ang_accel_cmd.x = 0;

    AttitudeController::UpdateError();

    if(!reset_angX_sent)
      reset_angX_sent = true;
    if(!inactive_angX_sent)
      inactive_angX_sent = true;
  }
}

void AttitudeController::ResetPitch() {
  if(!reset_angY_sent && !inactive_angY_sent) {
    pitch_controller_pid.reset();
    pitch_cmd = 0;
    pitch_error = 0;
    pitch_error_dot = 0;
    last_error.y = 0;
    last_error_dot.y = 0;

    status_msg.pitch.reference = 0;
    status_msg.pitch.error = 0;

    ang_accel_cmd.y = 0;

    AttitudeController::UpdateError();

    if(!reset_angY_sent)
      reset_angY_sent = true;
    if(!inactive_angY_sent)
      inactive_angY_sent = true;
  }
}

void AttitudeController::ResetYaw() {
  if(!reset_angZ_sent && !inactive_angZ_sent) {
    yaw_controller_pid.reset();
    yaw_cmd = 0;
    yaw_error = 0;
    yaw_error_dot = 0;
    last_error.z = 0;
    last_error_dot.z = 0;

    status_msg.yaw.reference = 0;
    status_msg.yaw.error = 0;

    ang_accel_cmd.z = 0;

    AttitudeController::UpdateError();

    if(!reset_angZ_sent)
      reset_angZ_sent = true;
    if(!inactive_angZ_sent)
      inactive_angZ_sent = true;
  }
}
