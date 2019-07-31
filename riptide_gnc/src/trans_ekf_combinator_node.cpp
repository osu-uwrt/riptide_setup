#include "riptide_gnc/trans_ekf_combinator.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trans_ekf_combinator");
  ros::NodeHandle nh("~");
  riptide_gnc::TransEKFCombinator tec(nh);

  while (ros::ok())
  {
    ros::spinOnce();
    if (tec.getCBCounter() > 0) // If any callbacks have been entered, then publish msg
    {
      tec.publishMsg();
    }
  }
  return 0;
}