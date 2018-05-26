#include "gazebo_plugins/gazebo_ros_IMU.h"
#include <cmath>
#include <assert.h>
#include <algorithm>
#include <string>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(IMU);

IMU::IMU()
{
  this->IMU.euler_rpy = 0;
  this->IMU.angular_velocity = 0;
  this->IMU.linear_acceleration = 0;
  this->IMU.angular_acceleration= 0;
}

IMU::~IMU()
{
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

  this->rosnode_->shutdown();

  delete this->rosnode_;
}

void IMU::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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

  this->angularVelocity = _model->GetWorldAngularVel();
  this->angularAccel = _model->GetWorldAngularAccel();
  this->linearAccel = _model->GetWorldLinearAccel();
  this->modelPose = _model->GetWorldPose();

  this->update_connection_ = event::Events::ConnectWorldUpdateEnd(boost::bind(&IMU::Update, this));
}

  void IMU::Update()
  {
    this->IMU.euler_rpy.set(this->modelPose.getRoll()*180/M_PI, this->modelPose.getPitch()*180/M_PI, this->modelPose.getYaw()*180/M_PI);
    this->IMU_.ang_vel = angularVelocity;
    this->IMU_.ang_accel = angularAccel;
    this->IMU_.linear_accel = linearAccel;
    this->IMU_pub_.publish(this->IMU_);
  }
}
