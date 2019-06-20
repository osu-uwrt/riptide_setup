#include "riptide_gnc/trans_ekf_combinator.hpp"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "trans_ekf_combinator");
  ros::NodeHandle nh("~");
  riptide_gnc::TransEKFCombinator tec(nh);

  while (ros::ok())
  {
     ros::spinOnce();
  }
  return 0;
}