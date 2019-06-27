#ifndef GNC_THRUST_CONVERTER
#define GNC_THRUST_CONVERTER

#include "ros/ros.h"
#include "riptide_msgs/ThrustStamped.h"
#include "auv_msgs/Thrust.h"

namespace riptide_gnc
{
class GNCThrustConverter
{
 private:
   ros::NodeHandle nh_;
   ros::Subscriber gnc_thrust_sub_;
   ros::Publisher riptide_thrust_pub_;

   void gncThrustCB(const auv_msgs::Thrust::ConstPtr &gnc_thrust);

 public:
   GNCThrustConverter(ros::NodeHandle nh);
};
} // namespace riptide_gnc

#endif
