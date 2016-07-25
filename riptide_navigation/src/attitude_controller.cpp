#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <control_toolbox/pid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <riptide_navigation/pidConfig.h>
#include <boost/asio.hpp>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Transform.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/Accel.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

double px,ix,dx,imx,dmx,py,iy,dy,imy,dmy,pz,iz,dz,imz,dmz;
bool first_reconfig= true;
ros::Publisher aa_e;
tf::StampedTransform transform;

void dyn_callback(riptide_navigation::pidConfig &config, uint32_t level) {

if (first_reconfig)
	{
	first_reconfig=false;
	return;
	}
px=config.px;
ix=config.ix;
dx=config.dx;
imx=config.imx;
dmx=config.dmx;
py=config.py;
iy=config.iy;
dy=config.dy;
imy=config.imy;
dmy=config.dmy;
pz=config.pz;
iz=config.iz;
dz=config.dz;
imz=config.imz;
dmz=config.dmz;
}

void callback(const nav_msgs::Odometry::ConstPtr& current_data, const geometry_msgs::PoseStamped::ConstPtr& pose_set, const geometry_msgs::TwistStamped::ConstPtr& twist_set, const geometry_msgs::AccelStampedConstPtr& lin_accel_error)
{

  geometry_msgs::Accel accel_error;
  ros::Time time;
  ros::Duration time_diff;
  ros::Time last_time;
  
  control_toolbox::Pid pidx, pidy, pidz;

  pidx.initPid(px,ix,dx,imx,dmx);
  pidy.initPid(py,iy,dy,imy,dmy);
  pidz.initPid(pz,iz,dz,imz,dmz);
  
  time = ros::Time::now();
  time_diff = time-last_time;
  
  accel_error.angular.x=pidx.control_toolbox::Pid::computeCommand((pose_set->pose.orientation.x-current_data->pose.pose.orientation.x),(twist_set->twist.angular.x-current_data->twist.twist.angular.x),time_diff);
  accel_error.angular.y=pidy.control_toolbox::Pid::computeCommand((pose_set->pose.orientation.y-current_data->pose.pose.orientation.y),(twist_set->twist.angular.y-current_data->twist.twist.angular.y),time_diff);
  accel_error.angular.z=pidz.control_toolbox::Pid::computeCommand((pose_set->pose.orientation.z-current_data->pose.pose.orientation.z),(twist_set->twist.angular.z-current_data->twist.twist.angular.z),time_diff);
  
  last_time = time;

  accel_error.linear.x=lin_accel_error->accel.linear.x;
  accel_error.linear.y=lin_accel_error->accel.linear.y;
  accel_error.linear.z=lin_accel_error->accel.linear.z;

  aa_e.publish(accel_error);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "attitude_controller");
  ros::NodeHandle nh;
  ros::NodeHandle node_priv("~");

  message_filters::Subscriber<nav_msgs::Odometry> kalmanout_sub(nh, "/odometry/filtered",1);
  message_filters::Subscriber<geometry_msgs::PoseStamped> poseset_sub(nh, "pose_set_pt",1);
  message_filters::Subscriber<geometry_msgs::TwistStamped> twistset_sub(nh, "twist_set_pt",1);
  message_filters::Subscriber<geometry_msgs::AccelStamped> linae_sub(nh, "lin_accel_error",1);
 

  dynamic_reconfigure::Server<riptide_navigation::pidConfig> server;
  dynamic_reconfigure::Server<riptide_navigation::pidConfig>::CallbackType f;
  f = boost::bind(&dyn_callback, _1, _2);
  server.setCallback(f);

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry,geometry_msgs::PoseStamped,geometry_msgs::TwistStamped,geometry_msgs::AccelStamped> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), kalmanout_sub, poseset_sub,twistset_sub,linae_sub);  
  sync.registerCallback(boost::bind(&callback,_1,_2,_3,_4));

  aa_e = nh.advertise<geometry_msgs::Accel>("accel_error", 1);
  tf::TransformListener listener;  
 
  ros::Rate rate(20.0);
  
  while (nh.ok()){
  try{
  listener.lookupTransform("/map","/base_link",ros::Time(0),transform);
  }
  catch (tf::TransformException &ex){
  ROS_ERROR ("%s", ex.what());
  }
  ros::spin();
  rate.sleep();
  }
}
