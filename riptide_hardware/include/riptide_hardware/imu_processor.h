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

class IMUProcessor
{
private:
  ros::NodeHandle nh;
  ros::Subscriber imu_filter_sub;
  ros::Publisher imu_state_pub;

  //0= current state, 1 = one state ago, 2 = two states ago
  riptide_msgs::Imu state[3];

  //Change in time between the three states
  //dt[0] = time b/w state[0] and state[1], dt[1] = time b/w state[1] and state[2]
  float dt[2];
public:
  IMUProcessor(char **argv);
  void callback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter_msg);
  void loop();
};

#endif
