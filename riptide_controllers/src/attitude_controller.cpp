#include "riptide_controllers/attitude_controller.h"

#undef debug
#undef report
#undef progress

int main(int argc, char **argv) {
  ros::init(argc, argv, "attitude_controller");
  AttitudeController ac;
  ros::spin();
}

void AttitudeController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  // Roll error
  roll_error = roll_cmd - current_attitude.x;
  roll_error_dot = (roll_error - last_error.x) / dt;
  last_error.x = roll_error;

  // Pitch error
  pitch_error = pitch_cmd - current_attitude.y;
  pitch_error_dot = (pitch_error - last_error.y) / dt;
  last_error.y = pitch_error;

  // Yaw error
  // Always take shortest path to setpoint
  yaw_error = yaw_cmd - current_attitude.z;
  if (yaw_error > 180)
      yaw_error -= 360;
  else if (yaw_error < -180)
      yaw_error += 360;

  yaw_error_dot = (yaw_error - last_error.z) / dt;
  last_error.z = yaw_error;

  accel_cmd.x = roll_controller_pid.computeCommand(roll_error, roll_error_dot, sample_duration);
  accel_cmd.y = pitch_controller_pid.computeCommand(pitch_error, pitch_error_dot, sample_duration);
  accel_cmd.z = yaw_controller_pid.computeCommand(yaw_error, yaw_error_dot, sample_duration);

  cmd_pub.publish(accel_cmd);
  sample_start = ros::Time::now();
}

AttitudeController::AttitudeController() {
    ros::NodeHandle rcpid("roll_controller");
    ros::NodeHandle ycpid("yaw_controller");
    ros::NodeHandle pcpid("pitch_controller");

    pid_initialized = false;

    cmd_sub = nh.subscribe<geometry_msgs::Vector3>("command/orientation", 1000, &AttitudeController::CommandCB, this);
    imu_sub = nh.subscribe<riptide_msgs::Imu>("state/imu", 1000, &AttitudeController::ImuCB, this);

    roll_controller_pid.init(rcpid, false);
    yaw_controller_pid.init(ycpid, false);
    pitch_controller_pid.init(pcpid, false);

    cmd_pub = nh.advertise<geometry_msgs::Vector3>("command/accel/angular", 1);
    sample_start = ros::Time::now();
}

// Subscribe to state/imu
void AttitudeController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu) {
  current_attitude = imu->euler_rpy;
  if (pid_initialized) {
    AttitudeController::UpdateError();
  }
}

// Subscribe to command/orientation
// set the MAX_ROLL and MAX_PITCH value in the header
void AttitudeController::CommandCB(const geometry_msgs::Vector3::ConstPtr &cmd) {
  roll_cmd = cmd->x;
  pitch_cmd = cmd->y;
  yaw_cmd = cmd->z;

  // Constrain pitch
  if(roll_cmd > MAX_ROLL)
    roll_cmd = MAX_ROLL;
  else if (roll_cmd < -MAX_ROLL)
    roll_cmd = -MAX_ROLL;

  // constrain roll
  if (pitch_cmd > MAX_PITCH)
      pitch_cmd = MAX_PITCH;
  else if (pitch_cmd < -MAX_PITCH)
    pitch_cmd = -MAX_PITCH;

  if (!pid_initialized)
    pid_initialized = true;

  AttitudeController::UpdateError();
}
