#include "riptide_gnc/gnc_thrust_converter.hpp"

namespace riptide_gnc
{
GNCThrustConverter::GNCThrustConverter(ros::NodeHandle nh)
{
   nh_ = nh;

   std::string gnc_thrust_sub_topic;
   nh_.param<std::string>("/auv_gnc/guidance_controller/publisher_topic", gnc_thrust_sub_topic, std::string("/auv_gnc/guidance_controller/thrust"));

   gnc_thrust_sub_ = nh_.subscribe<auv_msgs::Thrust>(gnc_thrust_sub_topic, 1, &GNCThrustConverter::gncThrustCB, this);
   riptide_thrust_pub_ = nh_.advertise<riptide_msgs::ThrustStamped>("/command/thrust", 1);
}

void GNCThrustConverter::gncThrustCB(const auv_msgs::Thrust::ConstPtr &gnc_thrust)
{
   riptide_msgs::ThrustStamped thrust;
   thrust.header.stamp = ros::Time::now();

   for (int i = 0; i < gnc_thrust->names.size(); i++)
   {
      if (gnc_thrust->names[i] == "HPF")
         thrust.force.heave_port_fwd = gnc_thrust->thrusts[i];

      if (gnc_thrust->names[i] == "HSF")
         thrust.force.heave_stbd_fwd = gnc_thrust->thrusts[i];

      if (gnc_thrust->names[i] == "HPA")
         thrust.force.heave_port_aft = gnc_thrust->thrusts[i];

      if (gnc_thrust->names[i] == "HSA")
         thrust.force.heave_stbd_aft = gnc_thrust->thrusts[i];

      if (gnc_thrust->names[i] == "VPF")
         thrust.force.vector_port_fwd = gnc_thrust->thrusts[i];

      if (gnc_thrust->names[i] == "VSF")
         thrust.force.vector_stbd_fwd = gnc_thrust->thrusts[i];

      if (gnc_thrust->names[i] == "VPA")
         thrust.force.vector_port_aft = gnc_thrust->thrusts[i];
         
      if (gnc_thrust->names[i] == "VSA")
         thrust.force.vector_stbd_aft = gnc_thrust->thrusts[i];
   }
   riptide_thrust_pub_.publish(thrust);

}


} // namespace riptide_gnc
