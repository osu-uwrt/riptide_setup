#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Transform.h"
#include "imu_3dm_gx4/FilterOutput.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/FluidPressure.h"
#include <string>

class Orientation
{
private:
  ros::NodeHandle nh;
  // Name of reference link:
  std::string target_frame;
  // Message handling groups:
  ros::Publisher filter_pub;
  ros::Subscriber filter_sub;
  imu_3dm_gx4::FilterOutput filter_;
  ros::Publisher imu_pub;
  ros::Subscriber imu_sub;
  sensor_msgs::Imu imu_;
  ros::Publisher field_pub;
  ros::Subscriber field_sub;
  sensor_msgs::MagneticField field_;
  ros::Publisher pressure_pub;
  ros::Subscriber pressure_sub;
  sensor_msgs::FluidPressure pressure_;
  // Correction term:
  tf::Matrix3x3 rotation_matrix;
  tf::Transform rotation;
  // Type conversion helper functions:
  static tf::Vector3 vector3(geometry_msgs::Vector3 msg);
  static geometry_msgs::Vector3 vector3(tf::Vector3 tf);
  static tf::Quaternion quaternion(geometry_msgs::Quaternion msg);
  static geometry_msgs::Quaternion quaternion(tf::Quaternion tf);

public:
  Orientation();
  void filter_cb(const imu_3dm_gx4::FilterOutput::ConstPtr& filter);
  void imu_cb(const sensor_msgs::Imu::ConstPtr& imu);
  void field_cb(const sensor_msgs::MagneticField::ConstPtr& field);
  void pressure_cb(const sensor_msgs::FluidPressure::ConstPtr& pressure);
  void loop();
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "orientation");
  Orientation orientation;
  orientation.loop();
}

Orientation::Orientation()
{
  target_frame = "base_link";

  filter_sub = nh.subscribe<imu_3dm_gx4::FilterOutput>("imu/filter", 1, &Orientation::filter_cb, this);
  filter_pub = nh.advertise<imu_3dm_gx4::FilterOutput>("state/filter", 1);
  filter_.header.frame_id = target_frame;;

  imu_pub = nh.advertise<sensor_msgs::Imu>("state/imu", 1);
  imu_sub = nh.subscribe<sensor_msgs::Imu>("imu/imu", 1, &Orientation::imu_cb, this);
  imu_.header.frame_id = target_frame;;

  field_pub = nh.advertise<sensor_msgs::MagneticField>("state/magnetic_field", 1);
  field_sub = nh.subscribe<sensor_msgs::MagneticField>("imu/magnetic_field", 1, &Orientation::field_cb, this);
  field_.header.frame_id = target_frame;;

  pressure_pub = nh.advertise<sensor_msgs::FluidPressure>("state/pressure", 1);
  pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>("imu/pressure", 1, &Orientation::pressure_cb, this);
  pressure_.header.frame_id = target_frame;;

  rotation_matrix = tf::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1);
  rotation = tf::Transform(rotation_matrix);
}

void Orientation::filter_cb(const imu_3dm_gx4::FilterOutput::ConstPtr& filter)
{
  filter_.header.stamp = filter->header.stamp;
  filter_.quat_status = filter->quat_status;
  filter_.bias_status = filter->bias_status;
  filter_.orientation = quaternion(rotation * quaternion(filter->orientation));
  // filter_.orientation_covariance
  filter_.bias = vector3(rotation * vector3(filter->bias));
  // filter_.bias_covariance
  filter_.bias_covariance_status = filter->bias_covariance_status;
  filter_.orientation_covariance_status = filter->orientation_covariance_status;
  filter_pub.publish(filter_);
}

void Orientation::imu_cb(const sensor_msgs::Imu::ConstPtr& imu)
{
  imu_.header.stamp = imu->header.stamp;
  // imu_.orientation = quaternion(rotation * quaternion(imu->orientation));
  // imu_.orientation_covariance
  imu_.angular_velocity = vector3(rotation * vector3(imu->angular_velocity));
  // imu_.angular_velocity_covariance
  imu_.linear_acceleration = vector3(rotation * vector3(imu->linear_acceleration));
  // imu_.linear_acceleration_covariance
  imu_pub.publish(imu_);
}

void Orientation::field_cb(const sensor_msgs::MagneticField::ConstPtr& field)
{
  field_.header.stamp = field->header.stamp;
  field_.magnetic_field = vector3(rotation * vector3(field->magnetic_field));
  // field_.magnetic_field_covariance
  field_pub.publish(field_);
}

void Orientation::pressure_cb(const sensor_msgs::FluidPressure::ConstPtr& pressure)
{
  pressure_.header.stamp = pressure->header.stamp;
  pressure_.fluid_pressure = pressure->fluid_pressure;
  pressure_.variance = pressure->variance;
  pressure_pub.publish(pressure_);
}

tf::Vector3 Orientation::vector3(geometry_msgs::Vector3 msg)
{
  tf::Vector3 tf;
  vector3MsgToTF(msg, tf);
  return tf;
}

geometry_msgs::Vector3 Orientation::vector3(tf::Vector3 tf)
{
  geometry_msgs::Vector3 msg;
  vector3TFToMsg(tf, msg);
  return msg;
}

tf::Quaternion Orientation::quaternion(geometry_msgs::Quaternion msg)
{
  tf::Quaternion tf;
  quaternionMsgToTF(msg, tf);
  return tf.normalized();
}

geometry_msgs::Quaternion Orientation::quaternion(tf::Quaternion tf)
{
  geometry_msgs::Quaternion msg;
  quaternionTFToMsg(tf, msg);
  return msg;
}

void Orientation::loop()
{
  ros::Rate rate(100);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
