#ifndef DEPTH_CONTROLLER_H
#define DEPTH_CONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "riptide_msgs/Depth.h"
#include "dynamic_reconfigure/server.h"
#include "riptide_controllers/DepthControllerConfig.h"

class DepthController
{
 private:
  // Comms
  ros::NodeHandle nh;
  ros::Subscriber depth_sub;
  ros::Subscriber cmd_sub;
  ros::Publisher cmd_pub;
  geometry_msgs::Accel accel;
  dynamic_reconfigure::Server<riptide_controllers::DepthControllerConfig> config_server;
  dynamic_reconfigure::Server<riptide_controllers::DepthControllerConfig>::CallbackType cb;

  //PID
  float depth_error;
  float current_depth;
  float cmd_depth;
  float error_sum;
  float d_error;
  float last_error;
  float dt;
  float kP;
  float kI;
  float kD;

  bool pid_initialized;
  bool clear_enabled;

  ros::Time sample_start;
  ros::Duration sample_duration;

  void UpdateError();

 public:
   DepthController();
   void CommandCB(const riptide_msgs::Depth::ConstPtr &depth);
   void DepthCB(const riptide_msgs::Depth::ConstPtr &cmd);
   void ConfigureCB(riptide_controllers::DepthControllerConfig &config, int level);
   void Loop();

};

#endif
