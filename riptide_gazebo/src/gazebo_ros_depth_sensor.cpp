#include "gazebo_plugins/gazebo_ros_depth_sensor.h"

#include <assert.h>
#include <algorithm>
#include <string>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(DepthSensor);

DepthSensor::DepthSensor()
{
  this->depth_.depth = 0;
  this->depth_.pressure = 0;
  this->depth_.temp = 0;
  this->depth_.altitude = 0;
}

DepthSensor::~DepthSensor()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

  this->rosnode_->shutdown();

  delete this->rosnode_;
}

void DepthSensor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->world_ = _model->GetWorld();
  this->gravity_ = this->world_->Gravity().Z();

  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  this->sensor_link_name_ = _sdf->GetElement("sensorLink")->Get<std::string>();
  this->sensor_link_ = _model->GetLink(sensor_link_name_);

  this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);
  this->depth_pub_ = this->rosnode_->advertise<riptide_msgs::Depth>(this->topic_name_, 1);

  this->sea_level_ = _sdf->GetElement("seaLevel")->Get<double>();
  this->fluid_density_ = _sdf->GetElement("fluidDensity")->Get<double>();

  this->update_connection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&DepthSensor::Update, this));
}

  void DepthSensor::Update()
  {
    double z_pos = this->sensor_link_->GetWorldPose().pos.z;
    this->depth_.depth = this->sea_level_ - z_pos;
    this->depth_.pressure = this->fluid_density_ * this->gravity_ * this->depth_.depth;
    this->depth_pub_.publish(this->depth_);
  }
}
