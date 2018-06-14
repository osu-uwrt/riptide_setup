#include "riptide_hardware/depth_processor.h"

#define PI 3.141592653

int main(int argc, char** argv)
{
 ros::init(argc, argv, "depth_processor");
 DepthProcessor dp;
 dp.Loop();
}

//Constructor
DepthProcessor::DepthProcessor() : nh("depth_processor") {
 depth_sub = nh.subscribe<riptide_msgs::Depth>("/depth/raw", 1, &DepthProcessor::DepthCB, this);
 depth_state_pub = nh.advertise<riptide_msgs::Depth>("/state/depth", 1);
 DepthProcessor::LoadParam<double>("post_IIR_LPF_bandwidth", post_IIR_LPF_bandwidth);
 DepthProcessor::LoadParam<double>("sensor_rate", sensor_rate);

 // IIR LPF Variables
 double fc = post_IIR_LPF_bandwidth; // Shorthand variable for IIR bandwidth
 dt = 1.0/sensor_rate;
 alpha = 2*PI*dt*fc / (2*PI*dt*fc + 1); // Multiplier
}

// Load parameter from namespace
template <typename T>
void DepthProcessor::LoadParam(string param, T &var)
{
  try
  {
    if (!nh.getParam(param, var))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    string ns = nh.getNamespace();
    ROS_INFO("Depth Processor namespace: %s", ns.c_str());
    ROS_ERROR("\tCritical! Param ""%s""/%s does not exist or is not accessed correctly.", ns.c_str(), param.c_str());
    ROS_ERROR("\tVerify namespace has param %s, or if the parameter exists. Shutting down.", param.c_str());
    ros::shutdown();
  }
}

// Callback
void DepthProcessor::DepthCB(const riptide_msgs::Depth::ConstPtr& depth_msg)
{
  depth_state.header = depth_msg->header;
  depth_state.depth = depth_msg->depth;
  depth_state.pressure = depth_msg->pressure;
  depth_state.temp = depth_msg->temp;
  depth_state.altitude = depth_msg->altitude;

  DepthProcessor::SmoothDataIIR();

  // Publish smoothed data to state/depth
  depth_state_pub.publish(depth_state);
}

// Apply IIR LPF
void DepthProcessor::SmoothDataIIR() {
  depth_state.depth = alpha*depth_state.depth + (1-alpha)*prev_depth;
  prev_depth = depth_state.depth;
}

void DepthProcessor::Loop(){
  ros::Rate rate(1000);
  while (!ros::isShuttingDown()){
    ros::spinOnce();
    rate.sleep();
  }
}
