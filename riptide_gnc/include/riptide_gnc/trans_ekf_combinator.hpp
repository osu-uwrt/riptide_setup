#ifndef TRANS_EKF_COMBINATOR
#define TRANS_EKF_COMBINATOR

#include "ros/ros.h"
#include "riptide_msgs/Imu.h"
#include "riptide_msgs/Depth.h"
#include "nortek_dvl/Dvl.h"
#include "auv_msgs/SixDoF.h"

namespace riptide_gnc
{
class TransEKFCombinator
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber septh_sub_, imu_sub_, dvl_sub_;
  ros::Publisher six_dof_pub_;

public:
  TransEKFCombinator(ros::NodeHandle nh);
  template <typename T>
  void loadParam(std::string param, T &var);
  void initMsgs();
  void depthCB(const riptide_msgs::Depth::ConstPtr &depth);
  void imuCB(const riptide_msgs::Imu::ConstPtr &imu);
  void dvlCB(const nortek_dvl::Dvl::ConstPtr &dvl);
};
} // namespace riptide_gnc

#endif
