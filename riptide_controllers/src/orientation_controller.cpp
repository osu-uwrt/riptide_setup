#include "riptide_controllers/orientation_controller.h"

#undef debug
#undef report
#undef progress

int main(int argc, char **argv) {
  ros::init(argc, argv, "depth_controller");
  OrientationController oc;
  ros::spin();
}

void OrientationController::UpdateError() {
  sample_duration = ros::Time::now() - sample_start;
  dt = sample_duration.toSec();

  // Roll error
  roll_error = roll_cmd - current_orientation.x;
  roll_error_dot = (roll_error - last_error.x) / dt;
  last_error.x = roll_error;

  // Pitch error
  pitch_error = pitch_cmd - current_orientation.y;
  pitch_error_dot = (pitch_error - last_error.y) / dt;
  last_error.y = pitch_error;

  // Yaw error
  // Always take shortest path to setpoint
  yaw_error = yaw_cmd - current_orientation.z;
  if (yaw_error > 180)
      yaw_error -= 180;
  else if (yaw_error < -180)
      yaw_error += 180;

  yaw_error_dot = (yaw_error - last_error.z) / dt;
  last_error.z = yaw_error;

  angular_accel_cmd.x = roll_controller_pid.computeCommand(roll_error, roll_error_dot, sample_duration);
  angular_accel_cmd.y = pitch_controller_pid.computeCommand(pitch_error, pitch_error_dot, sample_duration);
  angular_accel_cmd.z = yaw_controller_pid.computeCommand(yaw_error, yaw_error_dot, sample_duration);

  cmd_pub.publish(angular_accel_cmd);
  sample_start = ros::Time::now();
}


OrientationController::OrientationController() {
    ros::NodeHandle rcpid("roll_controller");
    ros::NodeHandle ycpid("yaw_controller");
    ros::NodeHandle pcpid("pitch_controller");


    cmd_sub = nh.subscribe<geometry_msgs::Vector3>("command/orientation", 1000, &OrientationController::CommandCB, this);
    imu_sub = nh.subscribe<riptide_msgs::Imu>("state/imu", 1000, &OrientationController::ImuCB, this);

    rcpid.setParam("p", 0.0);
    ycpid.setParam("p", 0.0);
    pcpid.setParam("p", 0.0);


    roll_controller_pid.init(rcpid, false);
    yaw_controller_pid.init(ycpid, false);
    pitch_controller_pid.init(pcpid, false);

    cmd_pub = nh.advertise<geometry_msgs::Vector3>("command/accel/angular", 1);
    sample_start = ros::Time::now();
}

// Subscribe to state/imu
void OrientationController::ImuCB(const riptide_msgs::Imu::ConstPtr &imu) {
  current_orientation = imu->euler_rpy;
  if (!pid_initialized) {
    roll_cmd = current_orientation.x;
    pitch_cmd = current_orientation.y;
    yaw_cmd = current_orientation.z;
  }

  OrientationController::UpdateError();
}

// Subscribe to command/orientation
// set the MAX_ROLL and MAX_PITCH value in the header
void OrientationController::CommandCB(const geometry_msgs::Vector3::ConstPtr &cmd) {
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

  OrientationController::UpdateError();
}
