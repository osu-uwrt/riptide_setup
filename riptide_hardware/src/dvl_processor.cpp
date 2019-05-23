#include "riptide_hardware/dvl_processor.h"

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "dvl_processor");
  DVLProcessor dp;
  ros::spin();
}

DVLProcessor::DVLProcessor() : nh("dvl_processor")
{
  imu_state_sub = nh.subscribe<riptide_msgs::Imu>("/state/imu", 1, &DVLProcessor::ImuCB, this);
  dvl_data_sub = nh.subscribe<nortek_dvl::Dvl>("/dvl/dvl", 1, &DVLProcessor::DvlCB, this);
  dvl_state_pub = nh.advertise<riptide_msgs::Dvl>("/state/dvl", 1);

  // Load relative positions between DVL and COM from YAML file
  DVLProcessor::LoadParam<string>("positions_file", positions_file);
  positions = YAML::LoadFile(positions_file);

  // DVLProcessor::LoadDVLPorperties();
}

template <typename T>
void DVLProcessor::LoadParam(string param, T &var)
{
  try
  {
    if (!nh.getParam(param, var))
    {
      throw 0;
    }
  }
  catch (int e)
  {
    string ns = nh.getNamespace();
    ROS_ERROR("DVL Processor Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. SHutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void DVLProcessor::LoadDVLProperties()
{
  for (int i = 0; i < 3; i++)
    dvl_position(i) = positions["properties"]["DVL"][i].as<double>();
}

void DVLProcessor::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg)
{
  Vector3d angular_v;
  angular_v(0) = imu_msg->ang_vel_rad.x;
  angular_v(1) = imu_msg->ang_vel_rad.y;
  angular_v(2) = imu_msg->ang_vel_rad.z;

  relative_v(0) = angular_v(1) * dvl_position(2) + angular_v(2) * dvl_position(1);
  relative_v(1) = angular_v(2) * dvl_position(0) + angular_v(0) * dvl_position(2);
  relative_v(2) = angular_v(0) * dvl_position(1) + angular_v(1) * dvl_position(0);
}
  
void DVLProcessor::DvlCB(const nortek_dvl::Dvl::ConstPtr &dvl_msg)
{
  state.vehicle_v.x = dvl_msg->velocity.x - relative_v(0);
  state.vehicle_v.y = dvl_msg->velocity.y - relative_v(1);
  state.vehicle_v.z = dvl_msg->velocity.z - relative_v(2);
  dvl_state_pub.publish(state);
}

void DVLProcessor::Loop()
{
  ros::Rate rate(100);
  while (!ros::isShuttingDown())
  {
    ros::spinOnce();
    rate.sleep();
  }
}