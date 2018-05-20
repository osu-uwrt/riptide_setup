#ifndef THRUSTER_CONTROLLER_H
#define THRUSTER_CONTROLLER_H

#include <math.h>
#include <vector>

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <dynamic_reconfigure/server.h>
#include <riptide_controllers/VehiclePropertiesConfig.h>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Accel.h"
#include "riptide_msgs/Imu.h"
#include "imu_3dm_gx4/FilterOutput.h"
#include "riptide_msgs/Depth.h"
#include "riptide_msgs/MassVol.h"
#include "riptide_msgs/ThrustStamped.h"

class ThrusterController
{
 private:
  // Comms
  ros::NodeHandle nh;
  ros::Subscriber state_sub, cmd_sub, depth_sub, mass_vol_sub, buoyancy_sub;
  ros::Publisher cmd_pub, buoyancy_pub;
  riptide_msgs::ThrustStamped thrust;

  bool debug_controller; // If true, key params can be input via messages
  dynamic_reconfigure::Server<riptide_controllers::VehiclePropertiesConfig> server;
  dynamic_reconfigure::Server<riptide_controllers::VehiclePropertiesConfig>::CallbackType cb;


  // Math
  ceres::Problem problem;
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;

  ceres::Problem buoyancyProblem;
  ceres::Solver::Options buoyancyOptions;
  ceres::Solver::Summary buoyancySummary;

  /*// Results
  double surge_port_lo, surge_stbd_lo;
  double sway_fwd, sway_aft;
  double heave_port_aft, heave_stbd_aft, heave_stbd_fwd, heave_port_fwd;//<-
  // TF
  tf::TransformListener *listener;
  tf::StampedTransform tf_surge[2];
  tf::StampedTransform tf_sway[2];
  tf::StampedTransform tf_heave[4]; */


 public:
  ThrusterController(char **argv);
  void LoadProperty(std::string name, double &param);
  void DynamicReconfigCallback(riptide_controllers::VehiclePropertiesConfig &config, uint32_t levels);
  void ImuCB(const riptide_msgs::Imu::ConstPtr &imu_msg);
  void DepthCB(const riptide_msgs::Depth::ConstPtr &depth_msg);
  void AccelCB(const geometry_msgs::Accel::ConstPtr &a);
  void MassVolCB(const riptide_msgs::MassVol::ConstPtr &mv);
  void Loop();
  void BuoyancyCB(const geometry_msgs::Vector3::ConstPtr &b_msg);
};

#endif
