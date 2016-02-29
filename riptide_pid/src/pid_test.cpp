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
#include <pid_control/pidConfig.h>
#include <boost/asio.hpp>


void dyn_callback(pid_control::pidConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f %f %f %f", 
            config.p, config.i, 
            config.d, 
            config.im, 
            config.dm);
}

void callback(const sensor_msgs::Imu::ConstPtr& current_accel, const geometry_msgs::AccelStamped::ConstPtr& accel_set)
{

  geometry_msgs::Vector3 accel_des;
  
  ros::Time time;
  ros::Duration time_diff;
  ros::Time last_time;
  double currentx=0;
  double currenty=0;
  double currentz=0;
  ROS_INFO("Entered Callback");

  accel_des.x = accel_set->accel.linear.x;
  accel_des.y = accel_set->accel.linear.y;
  accel_des.z = accel_set->accel.linear.z;

  currentx = current_accel->linear_acceleration.x;
  currenty = current_accel->linear_acceleration.y;
  currentz = current_accel->linear_acceleration.z;

  //double proportional, integral, derivative, i_max, d_max;
  control_toolbox::Pid pid;

  //dynamic_reconfigure::Server<pid_control::pidConfig> server;
  //dynamic_reconfigure::Server<pid_control::pidConfig>::CallbackType f;
  //f = boost::bind(&dyn_callback, _1, _2);
  //server.setCallback(f);

  pid.initPid(3, .5, 2, 0.3, -0.3);
  
  //control_toolbox::Pid::initDynamicReconfig(&nh);

  time = ros::Time::now();
  time_diff = time-last_time;
  pid.control_toolbox::Pid::computeCommand((accel_des.x-currentx),time_diff);
  pid.control_toolbox::Pid::getCurrentCmd();
  last_time = time;  
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "pid_control");
  ros::NodeHandle nh;
  ros::Publisher a_e;

  geometry_msgs::Accel accels_err;

  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "imu/imu",1);
  message_filters::Subscriber<geometry_msgs::AccelStamped> accel_sub(nh, "accel_set_pt",1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::AccelStamped> MySyncPolicy;
  //message_filters::TimeSynchronizer<sensor_msgs::Imu, geometry_msgs::AccelStamped>sync(imu_sub,accel_sub, 10);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), imu_sub, accel_sub);  
    sync.registerCallback(boost::bind(&callback,_1,_2));
    a_e = nh.advertise<geometry_msgs::Accel>("accel_error", 1);

dynamic_reconfigure::Server<pid_control::pidConfig> server;
  dynamic_reconfigure::Server<pid_control::pidConfig>::CallbackType f;
  f = boost::bind(&dyn_callback, _1, _2);
  server.setCallback(f);


  ros::spin();

}
