#include "riptide_gnc/pose_edkf_interface.h"

int main(int argc, char** argv)
{
 ros::init(argc, argv, "attitude_edkf");
 PoseEDKFInterface poseEDKF;
 ros::spin();
}

PoseEDKFInterface::PoseEDKFInterface() : nh("pose_edkf")
{
    
}