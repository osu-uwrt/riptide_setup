#include "riptide_teleop/ps3_debug_image.h"

#define GRAVITY 9.81 // [m/s^2]
#define WATER_DENSITY 1000 // [kg/m^3]

float round(float d) {
  return floor(d + 0.5);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ps3_debug_image");
  PS3DebugImage ps3di;
  ps3di.Loop();
}

PS3DebugImage::PS3DebugImage() : nh("ps3_debug_image") {
  /*depth_sub = nh.subscribe<riptide_msgs::Depth>("/state/depth", 1, &PS3Controller::DepthCB, this);
  attitude_pub = nh.advertise<geometry_msgs::Vector3>("/command/manual/attitude", 1);
  lin_accel_pub = nh.advertise<geometry_msgs::Vector3>("/command/manual/accel/linear", 1);
  depth_pub = nh.advertise<riptide_msgs::DepthCommand>("/command/manual/depth", 1);
  reset_pub = nh.advertise<riptide_msgs::ResetControls>("/controls/reset", 1);*/


  PS3DebugImage::InitMsgs();
}

void PS3DebugImage::InitMsgs() {

}

// Load property from namespace
void PS3DebugImage::LoadProperty(std::string name, double &param)
{
  try
  {
    if (!nh.getParam(name, param))
    {
      throw 0;
    }
  }
  catch(int e)
  {
    ROS_ERROR("Critical! PS3 Debug Image has no property set for %s. Shutting down...", name.c_str());
    ros::shutdown();
  }
}

void PS3DebugImage::DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg) {

}

void PS3Controller::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg) {

}

void AccelCB(const geometry_msgs::Accel::ConstPtr& accel_msg) {

}


void PS3DebugImage::Loop()
{
  ros::Rate rate(200);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
