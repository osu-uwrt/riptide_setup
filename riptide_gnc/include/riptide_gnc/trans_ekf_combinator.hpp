#ifndef TRANS_EKF_COMBINATOR
#define TRANS_EKF_COMBINATOR

#include "ros/ros.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/Depth.h"
#include "nortek_dvl/Dvl.h"
#include "auv_msgs/SixDoF.h"
#include <algorithm>

namespace riptide_gnc
{
class TransEKFCombinator
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber depth_sub_, imu_sub_, dvl_sub_;
  ros::Publisher six_dof_pub_;
  auv_msgs::SixDoF six_dof_msg_;
  int cbCounter_; // Counts the number of callbacks entered within each spin

public:
  TransEKFCombinator(ros::NodeHandle nh);
  int getCBCounter();
  void publishMsg();

  void depthCB(const riptide_msgs::Depth::ConstPtr &depth);
  void imuCB(const riptide_msgs::Imu::ConstPtr &imu);
  void dvlCB(const nortek_dvl::Dvl::ConstPtr &dvl);
};
} // namespace riptide_gnc

#endif
