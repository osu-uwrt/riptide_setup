#include "riptide_hardware/depth_processor.h"

int main(int argc, char** argv)
{
 ros::init(argc, argv, "depth_processor");
 DepthProcessor depth_processor;
 ros::spin();
}

//Constructor
DepthProcessor::DepthProcessor() : nh()
{
 depth_sub = nh.subscribe<riptide_msgs::Depth>("arduino/depth", 1, &DepthProcessor::DepthCB, this);
 state_depth_pub = nh.advertise<riptide_msgs::Depth>("state/depth", 1);
}

//Callback
void DepthProcessor::DepthCB(const riptide_msgs::Depth::ConstPtr& depth)
{
    riptide_msgs::Depth corrected;
    corrected.depth = (DEPTH_SLOPE * depth->depth) + DEPTH_OFFSET;
    // Eliminate the spikes
    if (corrected.depth > 10 || corrected.depth < -10)
      corrected = lastDepth;
    state_depth_pub.publish(corrected);
    lastDepth = corrected;
}
