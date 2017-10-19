#include "riptide_controllers/command_combinator.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "command_combinator");
  CommandCombinator cc;
  ros::spin();
}

void CommandCombinator::linearXCB(const std_msgs::Float64::ConstPtr &accel) {
  current_accel.linear.x = accel->data;
  cmd_pub.publish(current_accel);
}

void CommandCombinator::linearYCB(const std_msgs::Float64::ConstPtr &accel) {
  current_accel.linear.y = accel->data;
  cmd_pub.publish(current_accel);
}

void CommandCombinator::linearZCB(const std_msgs::Float64::ConstPtr &accel) {
  current_accel.linear.z = accel->data;
  cmd_pub.publish(current_accel);
}

void CommandCombinator::angularCB(const geometry_msgs::Vector3::ConstPtr &accel) {
  current_accel.angular.x = accel->x;
  current_accel.angular.y = accel->y;
  current_accel.angular.z = accel->z;
  cmd_pub.publish(current_accel);
}

CommandCombinator::CommandCombinator() {
    linear_x_sub = nh.subscribe<std_msgs::Float64>("command/accel/linear/x", 10, &CommandCombinator::linearXCB, this);
    linear_y_sub = nh.subscribe<std_msgs::Float64>("command/accel/linear/y", 10, &CommandCombinator::linearYCB, this);
    linear_z_sub = nh.subscribe<std_msgs::Float64>("command/accel/linear/z", 10, &CommandCombinator::linearZCB, this);

    angular_sub = nh.subscribe<geometry_msgs::Vector3>("command/accel/angular", 10, &CommandCombinator::angularCB, this);

    cmd_pub = nh.advertise<geometry_msgs::Accel>("command/accel", 10);

    current_accel.linear.x = 0;
    current_accel.linear.y = 0;
    current_accel.linear.z = 0;
    current_accel.angular.x = 0;
    current_accel.angular.y = 0;
    current_accel.angular.z = 0;
}
