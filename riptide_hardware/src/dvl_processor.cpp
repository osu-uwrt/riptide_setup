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
  Vector3d angular_vel;
  angular_vel(0) = imu_msg->ang_vel_rad.x;
  angular_vel(1) = imu_msg->ang_vel_rad.y;
  angular_vel(2) = imu_msg->ang_vel_rad.z;

  relative_vel(0) = angular_vel(1) * dvl_position(2) + angular_vel(2) * dvl_position(1);
  relative_vel(1) = angular_vel(2) * dvl_position(0) + angular_vel(0) * dvl_position(2);
  relative_vel(2) = angular_vel(0) * dvl_position(1) + angular_vel(1) * dvl_position(0);
}
  
void DVLProcessor::DvlCB(const nortek_dvl::Dvl::ConstPtr &dvl_msg)
{
  // state.header = dvl_msg->header;
  state.vehicle_time = dvl_msg->time;
  state.vehicle_dt1 = dvl_msg->dt1;
  state.vehicle_dt2 = dvl_msg->dt2;

  state.vehicle_vel.x = dvl_msg->velocity.x - relative_vel(0);
  state.vehicle_vel.y = dvl_msg->velocity.y - relative_vel(1);
  state.vehicle_vel.z = dvl_msg->velocity.z - relative_vel(2);

  state.vehicle_figureOfMerit = dvl_msg->figureOfMerit;
  state.vehicle_beamDistance = dvl_msg->beamDistance;
  state.vehicle_batteryVoltage = dvl_msg->batteryVoltage;
  state.vehicle_speedSound = dvl_msg->speedSound;
  state.vehicle_pressure = dvl_msg->pressure;
  state.vehicle_temp = dvl_msg->temp;

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