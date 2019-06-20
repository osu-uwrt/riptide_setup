#include "riptide_gnc/trans_ekf_combinator.hpp"

namespace riptide_gnc
{
TransEKFCombinator::TransEKFCombinator(ros::NodeHandle nh)
{
   nh_ = nh;

   TransEKFCombinator::initMsgs();
}

// Load parameter from namespace
template <typename T>
void TransEKFCombinator::loadParam(std::string param, T &var)
{
   try
   {
      if (!nh_.getParam(param, var))
      {
         throw 0;
      }
   }
   catch (int e)
   {
      std::string ns = nh_.getNamespace();
      ROS_ERROR("Command Combinator Namespace: %s", ns.c_str());
      ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. Shutting down.", ns.c_str(), param.c_str());
      ros::shutdown();
   }
}

void TransEKFCombinator::initMsgs()
{
}

void TransEKFCombinator::depthCB(const riptide_msgs::Depth::ConstPtr &depth)
{
}

void TransEKFCombinator::imuCB(const riptide_msgs::Imu::ConstPtr &imu)
{
}

void TransEKFCombinator::dvlCB(const nortek_dvl::Dvl::ConstPtr &dvl)
{
}
} // namespace riptide_gnc
