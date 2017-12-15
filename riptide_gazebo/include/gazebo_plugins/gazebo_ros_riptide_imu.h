#ifndef IMU_HH
#define IMU_HH

#include <string>
#include "boost/thread.hpp"
#include "boost/thread/mutex.hpp"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "ros/ros.h"
#include "riptide_msgs/Imu.h"

namespace gazebo {

class IMU : public ModelPlugin
{

 public:
  IMU();
  virtual ~IMU();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();

 private:
  event::ConnectionPtr update_connection_;
  physics::WorldPtr world_;

  // Sensor
  std::string sensor_link_name_;
  physics::LinkPtr sensor_link_;
  math::Pose modelPose;
  math::Vector3 angularVelocity;
  math::Vector3 angularAccel;
  math::Vector3 linearAccel;

  // Msg
  std::string robot_namespace_;
  std::string topic_name_;
  riptide_msgs::Imu IMU_;

  // ROS
  ros::NodeHandle* rosnode_;
  ros::Publisher IMU_pub_;

};

}

#endif
