#include "riptide_gnc/gnc_thrust_converter.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
   ros::init(argc, argv, "gnc_thrust_converter");
   ros::NodeHandle nh("~");
   riptide_gnc::GNCThrustConverter gnctc(nh);
   ros::spin();
   return 0;
}