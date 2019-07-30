#include "riptide_gnc/trans_ekf_combinator.hpp"

namespace riptide_gnc
{
TransEKFCombinator::TransEKFCombinator(ros::NodeHandle nh)
{
   nh_ = nh;

   std::string trans_ekf_sub_topic;
   nh_.param<std::string>("/auv_gnc/trans_ekf/subscriber_topic", trans_ekf_sub_topic, std::string("/auv_gnc/trans_ekf/input_data"));

   depth_sub_ = nh_.subscribe<riptide_msgs::Depth>("/state/depth", 1, &TransEKFCombinator::depthCB, this);
   imu_sub_ = nh_.subscribe<riptide_msgs::Imu>("/state/imu", 1, &TransEKFCombinator::imuCB, this);
   dvl_sub_ = nh_.subscribe<nortek_dvl::Dvl>("/state/dvl", 1, &TransEKFCombinator::dvlCB, this);

   six_dof_pub_ = nh_.advertise<auv_msgs::SixDoF>(trans_ekf_sub_topic, 1);
}

int TransEKFCombinator::getCBCounter()
{
   return cb_counter_;
}

void TransEKFCombinator::publishMsg()
{
   six_dof_pub_.publish(six_dof_msg_);
   cb_counter_ = 0;
}

void TransEKFCombinator::depthCB(const riptide_msgs::Depth::ConstPtr &depth)
{
   double time1 = six_dof_msg_.header.stamp.toSec(); // Current stamp
   double time2 = depth->header.stamp.toSec();       // Depth stamp
   if (time2 > time1)
      six_dof_msg_.header.stamp = depth->header.stamp;

   six_dof_msg_.pose.position.z = depth->depth;
   cb_counter_++;
}

void TransEKFCombinator::imuCB(const riptide_msgs::Imu::ConstPtr &imu)
{
   double time1 = six_dof_msg_.header.stamp.toSec(); // Current stamp
   double time2 = imu->header.stamp.toSec();         // IMU stamp
   if (time2 > time1)
      six_dof_msg_.header.stamp = imu->header.stamp;

   six_dof_msg_.pose.orientation.x = imu->quaternion.x;
   six_dof_msg_.pose.orientation.y = imu->quaternion.y;
   six_dof_msg_.pose.orientation.z = imu->quaternion.z;
   six_dof_msg_.pose.orientation.w = imu->quaternion.w;

   six_dof_msg_.velocity.angular.x = imu->ang_vel_rad.x;
   six_dof_msg_.velocity.angular.y = imu->ang_vel_rad.y;
   six_dof_msg_.velocity.angular.z = imu->ang_vel_rad.z;

   six_dof_msg_.linear_accel.x = imu->linear_accel.x;
   six_dof_msg_.linear_accel.y = imu->linear_accel.y;
   six_dof_msg_.linear_accel.z = imu->linear_accel.z;
   cb_counter_++;
}

void TransEKFCombinator::dvlCB(const nortek_dvl::Dvl::ConstPtr &dvl)
{
   double time1 = six_dof_msg_.header.stamp.toSec(); // Current stamp
   double time2 = dvl->header.stamp.toSec();         // DVL stamp
   if (time2 > time1)
      six_dof_msg_.header.stamp = dvl->header.stamp;

   if (!isnan(dvl->velocity.x)) // There is no easy way to handle the DVL outputting nan
   {
      six_dof_msg_.velocity.linear.x = dvl->velocity.x;
      six_dof_msg_.velocity.linear.y = dvl->velocity.y;
      six_dof_msg_.velocity.linear.z = dvl->velocity.z;
   }
   cb_counter_++;
}
} // namespace riptide_gnc
