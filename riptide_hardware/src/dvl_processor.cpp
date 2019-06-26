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

  dvl_state_pub = nh.advertise<nortek_dvl::Dvl>("/state/dvl", 1);
  // dvl_data_pub = nh.advertise<riptide_msgs::Dvl>("/state/dvl2", 1);

  // Load relative positions between DVL and COM from YAML file
  DVLProcessor::LoadParam<string>("properties_file", properties_file);
  properties = YAML::LoadFile(properties_file);
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
    dvl_position(i) = properties["properties"]["dvl"][i].as<double>() - properties["properties"]["center_of_mass"][i].as<double>();
  psi = properties["properties"]["dvl"][3].as<double>() * PI / 180;
}

void DVLProcessor::ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg)
{
  Vector3d angular_vel;
  angular_vel(0) = imu_msg->ang_vel_rad.x;
  angular_vel(1) = imu_msg->ang_vel_rad.y;
  angular_vel(2) = imu_msg->ang_vel_rad.z;
  relative_vel = angular_vel.cross(dvl_position);
}

void DVLProcessor::DvlCB(const nortek_dvl::Dvl::ConstPtr &dvl_msg)
{
  nortek_dvl::Dvl dvl_state(*dvl_msg);

  dvl_state.velocity.x = cos(psi) * dvl_msg->velocity.x + sin(psi) * dvl_msg->velocity.y - relative_vel(0);
  dvl_state.velocity.y = -sin(psi) * dvl_msg->velocity.x + cos(psi) * dvl_msg->velocity.y - relative_vel(1);
  dvl_state.velocity.z = dvl_msg->velocity.z - relative_vel(2);

  dvl_state_pub.publish(dvl_state);

  // riptide_msgs::Dvl dvl_state2(*dvl_state);
  
  // dvl_data_pub.publish(dvl_state2);

}