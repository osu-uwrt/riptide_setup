#include "riptide_hardware/imu_processor.h"

 int main(int argc, char** argv)
 {
   ros::init(argc, argv, "imu_processor");
   IMUProcessor imu(argv);
   imu.loop();
 }

//Constructor
 IMUProcessor::IMUProcessor(char **argv) : nh()
 {
   imu_filter_sub = nh.subscribe<imu_3dm_gx4::FilterOutput>("imu/filter", 1, &IMUProcessor::callback, this);
   imu_state_pub = nh.advertise<riptide_msgs::Imu>("state/imu", 1);
 }

 //Callback
  void IMUProcessor::callback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter_msg)
  {
    //Take in message into state[0]
    state[0].header = filter_msg->header;
    state[0].header.frame_id = "base_link";
    state[0].euler_rpy = filter_msg->euler_rpy;
    state[0].euler_rpy_status = filter_msg->euler_rpy_status;
    state[0].linear_acceleration = filter_msg->linear_acceleration;
    state[0].linear_acceleration_status = filter_msg->linear_acceleration_status;
    state[0].angular_velocity = filter_msg->angular_velocity;
    state[0].angular_velocity_status = filter_msg->angular_velocity_status;

    //Set new delta time (nano seconds)
    if(dt[1] != 0) {
      dt[0] = state[0].header.stamp.nsec - dt[1];
    }

    //Process linear acceleration (Remove centrifugal and tangential components)

    //Process angular acceleration (Use 3-pt backwards rule to approximate angular acceleration)

    //Publish message for current state, adjust states and dt arrays
    imu_state_pub.publish(state[0]);
    state[2] = state[1];
    state[1] = state[0];
    dt[1] = dt[0]; //Will set dt[0] upon receiving new message
  }

//ROS loop function
 void IMUProcessor::loop()
 {
   ros::Rate rate(1000);
   while (!ros::isShuttingDown())
   {
     ros::spinOnce();
     rate.sleep();
   }
 }
