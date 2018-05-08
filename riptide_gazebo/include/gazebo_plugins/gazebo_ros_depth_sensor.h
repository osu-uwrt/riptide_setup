#ifndef DEPTH_SENSOR_HH
#define DEPTH_SENSOR_HH

#include <string>

#include "boost/thread.hpp"
#include "boost/thread/mutex.hpp"

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "ros/ros.h"
#include "riptide_msgs/Depth.h"

namespace gazebo
{
class DepthSensor : public ModelPlugin
{
 public:
  DepthSensor();
  virtual ~DepthSensor();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();

 private:
  event::ConnectionPtr update_connection_;
  physics::WorldPtr world_;
  // Sensor
  std::string sensor_link_name_;
  physics::LinkPtr sensor_link_;
  double sea_level_;
  double fluid_density_;
  double gravity_;
  // Msg
  std::string robot_namespace_;
  std::string topic_name_;
  riptide_msgs::Depth depth_;
  // ROS
  ros::NodeHandle* rosnode_;
  ros::Publisher depth_pub_;

};
}
#endif
