#include "riptide_hardware/depth_processor.h"

int main(int argc, char** argv)
{
 ros::init(argc, argv, "depth_processor");
 DepthProcessor depth_processor;
 ros::spin();
}

//Constructor
DepthProcessor::DepthProcessor()
{
 depth_sub = nh.subscribe<riptide_msgs::Depth>("arduino/depth", 1, &DepthProcessor::DepthCB, this);
 state_depth_pub = nh.advertise<riptide_msgs::Depth>("state/depth", 1);
}

//Callback
void DepthProcessor::DepthCB(const riptide_msgs::Depth::ConstPtr& depth)
{
    // Adjust Depth according to calibration
    riptide_msgs::Depth corrected;
    corrected.depth = (DEPTH_SLOPE * depth->depth) + DEPTH_OFFSET;
    state_depth_pub.publish(corrected);
}
