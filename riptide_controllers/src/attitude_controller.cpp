#include "riptide_controllers/attitude_controller.h"

#undef debug
#undef report
#undef progress

#define MAX_ROLL_ERROR 10
#define MAX_PITCH_ERROR 10
#define MAX_YAW_ERROR 45

float round(float d) {
  return floor(d + 0.5);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "attitude_controller");
  AttitudeController ac;
  ac.Loop();
}

AttitudeController::AttitudeController() {
    ros::NodeHandle rcpid("roll_controller");
    ros::NodeHandle ycpid("yaw_controller");
    ros::NodeHandle pcpid("pitch_controller");

    pid_roll_init = false;
    pid_pitch_init = false;
    pid_yaw_init = false;

    roll_controller_pid.init(rcpid, false);
    yaw_controller_pid.init(ycpid, false);
    pitch_controller_pid.init(pcpid, false);

    cmd_sub = nh.subscribe<geometry_msgs::Vector3>("command/attitude", 1, &AttitudeController::CommandCB, this);
    imu_sub = nh.subscribe<riptide_msgs::Imu>("state/imu", 1, &AttitudeController::ImuCB, this);
    reset_sub = nh.subscribe<riptide_msgs::ResetControls>("controls/reset", 1, &AttitudeController::ResetController, this);

    cmd_pub = nh.advertise<geometry_msgs::Vector3>("command/accel/angular", 1);
    status_pub = nh.advertise<riptide_msgs::ControlStatusAngular>("controls/status/angular", 1);

    sample_start_roll = ros::Time::now();
    sample_start_pitch = sample_start_roll;
    sample_start_yaw = sample_start_roll;

    AttitudeController::InitPubMsg();
}

void AttitudeController::InitPubMsg() {
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

void AttitudeController::UpdateError() {
  // Roll error
  if(pid_roll_init) {
    sample_duration_roll = ros::Time::now() - sample_start_roll;
    dt_roll = sample_duration_roll.toSec();

    roll_error = roll_cmd - round(current_attitude.x);
    roll_error = AttitudeController::ConstrainError(roll_error, MAX_ROLL_ERROR);
    roll_error_dot = (roll_error - last_error.x) / dt_roll;
    last_error.x = roll_error;
    status_msg.roll.error = roll_error;

    ang_accel_cmd.x = roll_controller_pid.computeCommand(roll_error, roll_error_dot, sample_duration_roll);
  }

  // Pitch error
  if(pid_pitch_init) {
    sample_duration_pitch = ros::Time::now() - sample_start_pitch;
    dt_pitch = sample_duration_pitch.toSec();

    pitch_error = pitch_cmd - round(current_attitude.y);
    pitch_error = AttitudeController::ConstrainError(pitch_error, MAX_PITCH_ERROR);
    pitch_error_dot = (pitch_error - last_error.y) / dt_pitch;
    last_error.y = pitch_error;
    status_msg.pitch.error = pitch_error;

    ang_accel_cmd.y = pitch_controller_pid.computeCommand(pitch_error, pitch_error_dot, sample_duration_pitch);
  }

  // Yaw error
  if(pid_yaw_init) {
    sample_duration_yaw = ros::Time::now() - sample_start_yaw;
    dt_yaw = sample_duration_yaw.toSec();

    // Always take shortest path to setpoint
    yaw_error = yaw_cmd - round(current_attitude.z);
    if (yaw_error > 180)
        yaw_error -= 360;
    else if (yaw_error < -180)
        yaw_error += 360;
    yaw_error = AttitudeController::ConstrainError(yaw_error, MAX_YAW_ERROR);

    yaw_error_dot = (yaw_error - last_error.z) / dt_yaw;
    last_error.z = yaw_error;
    status_msg.yaw.error = yaw_error;

    ang_accel_cmd.z = yaw_controller_pid.computeCommand(yaw_error, yaw_error_dot, sample_duration_yaw);
  }

  // ALWAYS Publish status and command messages
  status_msg.header.stamp = ros::Time::now();
  status_pub.publish(status_msg);
  cmd_pub.publish(ang_accel_cmd);

  sample_start_roll = ros::Time::now();
  sample_start_pitch = ros::Time::now();
  sample_start_yaw = ros::Time::now();
}

// Constrain physical error. This acts as a way to break up the motion into
// smaller segments. Otherwise, if error is too large, the vehicle would move
// too quickly in the water in exhibit large overshoot
double AttitudeController::ConstrainError(double error, double max) {
  if(error > max)
    return max;
  else if(error < -1*max)
    return -1*max;
  return error;
}

// Subscribe to state/imu
void AttitudeController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu) {
  current_attitude = imu->euler_rpy;
  status_msg.roll.current = current_attitude.x;
  status_msg.pitch.current = current_attitude.y;
  status_msg.yaw.current = current_attitude.z;
}

// Subscribe to command/orientation
// set the MAX_ROLL and MAX_PITCH value in the header
void AttitudeController::CommandCB(const geometry_msgs::Vector3::ConstPtr &cmd) {
  roll_cmd = round(cmd->x);
  pitch_cmd = round(cmd->y);
  yaw_cmd = round(cmd->z);
  status_msg.roll.reference = roll_cmd;
  status_msg.pitch.reference = pitch_cmd;
  status_msg.yaw.reference = yaw_cmd;
}

void AttitudeController::ResetController(const riptide_msgs::ResetControls::ConstPtr& reset_msg) {
  //Reset any controllers if required from incoming message
  if(reset_msg->reset_roll)
    AttitudeController::ResetRoll();
  else pid_roll_init = true;

  if(reset_msg->reset_pitch)
    AttitudeController::ResetPitch();
  else pid_pitch_init = true;

  if(reset_msg->reset_yaw)
    AttitudeController::ResetYaw();
  else pid_yaw_init = true;
}

void AttitudeController::ResetRoll() {
  roll_cmd = 0;
  roll_error = 0;
  roll_error_dot = 0;
  roll_controller_pid.reset();
  current_attitude.x = 0;
  last_error.x = 0;

  sample_start_roll = ros::Time::now();
  sample_duration_roll = ros::Duration(0);
  dt_roll = 0;

  status_msg.roll.reference = 0;
  status_msg.roll.error = 0;

  pid_roll_init = false;
  ang_accel_cmd.x = 0;
}

void AttitudeController::ResetPitch() {
  pitch_cmd = 0;
  pitch_error = 0;
  pitch_error_dot = 0;
  pitch_controller_pid.reset();
  current_attitude.y = 0;
  last_error.y = 0;

  sample_start_pitch = ros::Time::now();
  sample_duration_pitch = ros::Duration(0);
  dt_pitch = 0;

  status_msg.pitch.reference = 0;
  status_msg.pitch.error = 0;

  pid_pitch_init = false;
  ang_accel_cmd.y = 0;
}

void AttitudeController::ResetYaw() {
  yaw_cmd = 0;
  yaw_error = 0;
  yaw_error_dot = 0;
  yaw_controller_pid.reset();
  current_attitude.z = 0;
  last_error.z = 0;

  sample_start_yaw = ros::Time::now();
  sample_duration_yaw = ros::Duration(0);
  dt_yaw = 0;

  status_msg.yaw.reference = 0;
  status_msg.yaw.error = 0;

  pid_yaw_init = false;
  ang_accel_cmd.z = 0;
}

void AttitudeController::Loop() {
  while(!ros::isShuttingDown()) {
    AttitudeController::UpdateError(); // ALWAYS update error, regardless of circumstance
    ros::spinOnce();
  }
}
