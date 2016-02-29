#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/AccelStamped.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <control_toolbox/pid.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <riptide_pid/pidConfig.h>
#include <boost/asio.hpp>

double p,i,d,im,dm;
bool first_reconfig= true;
ros::Publisher a_e;

void dyn_callback(riptide_pid::pidConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f %f %f", 
            config.p, config.i, 
            config.d, 
            config.im, 
            config.dm);
if (first_reconfig)
	{
	first_reconfig=false;
	return;
	}
p=config.p;
i=config.i;
d=config.d;
im=config.im;
dm=config.dm;
}

void callback(const sensor_msgs::Imu::ConstPtr& current_accel, const geometry_msgs::AccelStamped::ConstPtr& accel_set)
{

  geometry_msgs::Vector3 accel_des,accel_thrusters;
  
  ros::Time time;
  ros::Duration time_diff;
  ros::Time last_time;
  double currentx=0;
  double currenty=0;
  double currentz=0;

  accel_des.x = accel_set->accel.linear.x;
  accel_des.y = accel_set->accel.linear.y;
  accel_des.z = accel_set->accel.linear.z;

  currentx = current_accel->linear_acceleration.x;
  currenty = current_accel->linear_acceleration.y;
  currentz = current_accel->linear_acceleration.z;

  control_toolbox::Pid pid;

  pid.initPid(p,i,d,im,dm);

  time = ros::Time::now();
  time_diff = time-last_time;
  accel_thrusters.x=pid.control_toolbox::Pid::computeCommand((accel_des.x-currentx),time_diff);

  accel_thrusters.y=pid.control_toolbox::Pid::computeCommand((accel_des.y-currenty),time_diff);

  accel_thrusters.z=pid.control_toolbox::Pid::computeCommand((accel_des.z-currentz),time_diff);

  pid.control_toolbox::Pid::getCurrentCmd();
  last_time = time;  
  ROS_INFO("PID Accel Results -> X:%f   Y:%f   Z:%f",accel_thrusters.x,accel_thrusters.y,accel_thrusters.z);

a_e.publish(accel_thrusters);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "pid_linear");
  ros::NodeHandle nh;
  ros::NodeHandle node_priv("~");

  node_priv.param<double>("p",p,1.0);
  node_priv.param<double>("i",i,1.0);
  node_priv.param<double>("d",d,1.0);
  node_priv.param<double>("im",im,.3);
  node_priv.param<double>("dm",dm,-.3);

  geometry_msgs::Accel accels_err;

  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu/imu",1);
  message_filters::Subscriber<geometry_msgs::AccelStamped> accel_sub(nh, "accel_set_pt",1);

  dynamic_reconfigure::Server<riptide_pid::pidConfig> server;
  dynamic_reconfigure::Server<riptide_pid::pidConfig>::CallbackType f;
  f = boost::bind(&dyn_callback, _1, _2);
  server.setCallback(f);


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::AccelStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), imu_sub, accel_sub);  
    sync.registerCallback(boost::bind(&callback,_1,_2));

    a_e = nh.advertise<geometry_msgs::Vector3>("accel_error", 1);
  ros::spin();

}
