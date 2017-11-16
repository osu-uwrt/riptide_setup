#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include "ros/ros.h"
//#include "message_filters/subscriber.h"
//#include "message_filters/synchronizer.h"
//#include "message_filters/sync_policies/approximate_time.h"
//#include "tf/transform_broadcaster.h"
#include "riptide_msgs/Imu.h"
#include "std_msgs/Header.h"
#include "imu_3dm_gx4/FilterOutput.h"
#include "math.h"

class IMUProcessor
{
private:
  ros::NodeHandle nh;
  ros::Subscriber imu_filter_sub;
  ros::Publisher imu_state_pub;
  int cycles;

  //0 = current state, 1 = one state ago, 2 = two states ago, etc.
  //Only velocities and accelerations will be smoothed
  riptide_msgs::Imu raw_state[7];
  riptide_msgs::Imu smoothed_state[7];
  float zero_ang_vel_thresh;
public:
  IMUProcessor(char **argv);
  void callback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter_msg);
  void smoothData();
  void loop();
};

#endif
