#include "riptide_gnc/attitude_edkf_interface.h"

int main(int argc, char** argv)
{
 ros::init(argc, argv, "attitude_edkf");
 AttitudeEDKFInterface attEDKF;
 ros::spin();
}

AttitudeEDKFInterface::AttitudeEDKFInterface() : nh("attitude_edkf")
{
    
}