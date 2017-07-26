#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "std_msgs/Empty.h"

bool heartBeating = false;
ros::Time missionStartTime;
ros::Duration delayStartDuration;
ros::Duration submergeDuration;
ros::Duration goStraightDuration;

ros::Time lastHeartBeat;
ros::Duration deadTimeThreshold;

void heartbeatCB(const std_msgs::Empty::ConstPtr &msg) {
  lastHeartBeat = ros::Time::now();
  if (!heartBeating) {
    heartBeating = true;
    missionStartTime = lastHeartBeat;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "go_straight");
  ros::NodeHandle nh;

  ros::Publisher accel_pub = nh.advertise<geometry_msgs::Accel>("command/accel", 1000);
  ros::Subscriber kill_sub = nh.subscribe<std_msgs::Empty>("state/kill", 100, heartbeatCB);

  geometry_msgs::Accel accel;

  submergeDuration.fromSec(5.0);
  goStraightDuration.fromSec(10.0);
  delayStartDuration.fromSec(10.0);
  deadTimeThreshold.fromSec(2.0);

  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();

    if (ros::Time::now() - lastHeartBeat > deadTimeThreshold) {
      heartBeating = false;
    }

    if (heartBeating) {
      ros::Time currentTime = ros::Time::now();
      if (currentTime - missionStartTime < delayStartDuration) {
        accel.linear.x = 0;
        accel.linear.y = 0;
        accel.linear.z = 0;
        accel.angular.x = 0;
        accel.angular.y = 0;
        accel.angular.z = 0;
      } else if (currentTime - missionStartTime - delayStartDuration < submergeDuration) {
        accel.linear.x =0.2;
        accel.linear.y = 0;
        accel.linear.z = -0.5;
        accel.angular.x = 0;
        accel.angular.y = 0;
        accel.angular.z = 0;
      } else if (currentTime - missionStartTime - submergeDuration - delayStartDuration < goStraightDuration) {
        accel.linear.x = 0.5;
        accel.linear.y = 0;
        accel.linear.z = -0.1;
        accel.angular.x = 0;
        accel.angular.y = 0;
        accel.angular.z = 0;
      } else {
        accel.linear.x = 0;
        accel.linear.y = 0;
        accel.linear.z = 0;
        accel.angular.x = 0;
        accel.angular.y = 0;
        accel.angular.z = 0;
      }
    } else {
      accel.linear.x = 0;
      accel.linear.y = 0;
      accel.linear.z = 0;
      accel.angular.x = 0;
      accel.angular.y = 0;
      accel.angular.z = 0;
    }

    accel_pub.publish(accel);

    loop_rate.sleep();
  }
}
