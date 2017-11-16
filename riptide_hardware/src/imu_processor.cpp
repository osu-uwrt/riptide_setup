#include "riptide_hardware/imu_processor.h"

 #define PI 3.141592653

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

   zero_ang_vel_thresh = 1;
   cycles = 1;

 }

 //Callback
  void IMUProcessor::callback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter_msg)
  {
    //Put message data into raw_state[0]
    raw_state[0].header = filter_msg->header;
    raw_state[0].header.frame_id = "base_link";
    raw_state[0].raw_euler_rpy = filter_msg->euler_rpy;
    raw_state[0].euler_rpy = filter_msg->euler_rpy;
    raw_state[0].gyro_bias = filter_msg->gyro_bias;
    raw_state[0].euler_rpy_status = filter_msg->euler_rpy_status;
    raw_state[0].heading = filter_msg->heading;
    raw_state[0].heading_uncertainty = filter_msg->heading_uncertainty;
    raw_state[0].heading_update_source = filter_msg->heading_update_source;
    raw_state[0].heading_flags = filter_msg->heading_flags;
    raw_state[0].linear_acceleration = filter_msg->linear_acceleration;
    raw_state[0].linear_acceleration_status = filter_msg->linear_acceleration_status;
    raw_state[0].angular_velocity = filter_msg->angular_velocity;
    raw_state[0].angular_velocity_status = filter_msg->angular_velocity_status;

    //Convert angular values from radians to degrees
    raw_state[0].raw_euler_rpy.x *= (180.0/PI);
    raw_state[0].raw_euler_rpy.y *= (180.0/PI);
    raw_state[0].raw_euler_rpy.z *= (180.0/PI);
    raw_state[0].euler_rpy.x *= (180.0/PI);
    raw_state[0].euler_rpy.y *= (180.0/PI);
    raw_state[0].euler_rpy.z *= (180.0/PI);
    raw_state[0].gyro_bias.x *= (180.0/PI);
    raw_state[0].gyro_bias.y *= (180.0/PI);
    raw_state[0].gyro_bias.z *= (180.0/PI);
    raw_state[0].heading *= (180/PI);
    raw_state[0].heading_uncertainty *= (180/PI);
    raw_state[0].angular_velocity.x *= (180.0/PI);
    raw_state[0].angular_velocity.y *= (180.0/PI);
    raw_state[0].angular_velocity.z *= (180.0/PI);

    //Set euler_rpy.z equal to heading
    raw_state[0].euler_rpy.z = raw_state[0].heading;

    //Adjust direction/sign of Euler Angles to be consistent with AUV's axes
    if(raw_state[0].euler_rpy.x > -180 && raw_state[0].euler_rpy.x < 0) {
      raw_state[0].euler_rpy.x += 180;
      raw_state[0].raw_euler_rpy.x += 180;
    }
    else if(raw_state[0].euler_rpy.x > 0 && raw_state[0].euler_rpy.x < 180) {
      raw_state[0].euler_rpy.x -= 180;
      raw_state[0].raw_euler_rpy.x -= 180;
    }
    else if(raw_state[0].euler_rpy.x == 0) {
      raw_state[0].euler_rpy.x = 180;
      raw_state[0].raw_euler_rpy.x = 180;
    }
    else if(raw_state[0].euler_rpy.x == 180 || raw_state[0].euler_rpy.x == -180) {
      raw_state[0].euler_rpy.x = 0;
      raw_state[0].raw_euler_rpy.x = 0;
    }
    raw_state[0].euler_rpy.z *= -1; //Negate Euler yaw angle
    raw_state[0].raw_euler_rpy.z *= -1; //Negate raw Euler yaw angle

    //Set new delta time (convert nano seconds to seconds)
    if(raw_state[1].dt == 0) { //If dt[0] not set yet, set to 0.01s, which is roughly what it would be
      raw_state[0].dt = 0.01;
    }
    else { //dt has already been set, so find actual dt
      raw_state[0].dt = (double)raw_state[0].header.stamp.sec + raw_state[0].header.stamp.nsec/(1.0e9) -
                    (double)raw_state[1].header.stamp.sec - raw_state[1].header.stamp.nsec/(1.0e9);
    }

    //Process Drift
    //Check if angular velocity about any axis is approximately 0
    if(abs(raw_state[0].angular_velocity.x) <= zero_ang_vel_thresh) {
      raw_state[0].local_drift.x = raw_state[0].euler_rpy.x - raw_state[1].euler_rpy.x;
    }
    else {
      raw_state[0].local_drift.x = 0;
    }
    raw_state[0].local_drift_rate.x = raw_state[0].local_drift.x / raw_state[0].dt;

    if(abs(raw_state[0].angular_velocity.y) <= zero_ang_vel_thresh) {
      raw_state[0].local_drift.y = raw_state[0].euler_rpy.y - raw_state[1].euler_rpy.y;
    }
    else {
      raw_state[0].local_drift.y = 0;
    }
    raw_state[0].local_drift_rate.y = raw_state[0].local_drift.y / raw_state[0].dt;

    if(abs(raw_state[0].angular_velocity.z) <= zero_ang_vel_thresh) {
      raw_state[0].local_drift.z = raw_state[0].euler_rpy.z - raw_state[1].euler_rpy.z;    }
    else {
      raw_state[0].local_drift.z = 0;
    }
    raw_state[0].local_drift_rate.z = raw_state[0].local_drift.z / raw_state[0].dt;

    //Smooth Velocity and Acceleration data
    smoothData();

    //Process Euler Angles
    if(state[0].euler_rpy.x > -180 && state[0].euler_rpy.x < 0) {
      state[0].euler_rpy.x += 180;
    }
    else if(state[0].euler_rpy.x > 0 && state[0].euler_rpy.x < 180) {
      state[0].euler_rpy.x -= 180;
    }
    else if(state[0].euler_rpy.x == 0) {
      state[0].euler_rpy.x = 180;
    }
    else if(state[0].euler_rpy.x == 180 || state[0].euler_rpy.x == -180) {
      state[0].euler_rpy.x = 0;
    }
    state[0].euler_rpy.z *= -1;
    //Process linear acceleration (Remove centrifugal and tangential components)


    //Process angular acceleration (Use 3-pt backwards rule to approximate angular acceleration)
    /*if(cycles >= 7) { //Need 3 data points (state[2] must have values)
      //ROS_INFO("Cycles: %i", cycles);
      float a, b; //Coefficients for backwards-diff rule

      float c = state[0].dt, d = state[1].dt;
      //ROS_INFO("state[1].dt: %f", c);
      //ROS_INFO("state[2].dt: %f", d);

      a = -1.0/c - 1/(2*d-c);
      b = c/(4*d*d-2*c*d);

      //Estimated angular acceleration to error order (dt)^2
      state[0].angular_acceleration.x = a*state[1].angular_velocity.x + b*state[2].angular_velocity.x - (a+b)*state[0].angular_velocity.x;
      state[0].angular_acceleration.y = a*state[1].angular_velocity.y + b*state[2].angular_velocity.y - (a+b)*state[0].angular_velocity.y;
      state[0].angular_acceleration.z = a*state[1].angular_velocity.z + b*state[2].angular_velocity.z - (a+b)*state[0].angular_velocity.z;
    }*/

    //Must have completed 14 cycles because there need to be 7 smoothed data points
    //before processing  velocities, accelerations, etc.
    if(cycles < 14) {
      cycles += 1;
    }

    //Publish message for current state and adjust previous states


    for(int i=6; i>0; i--) {
      raw_state[i] = raw_state[i-1];
      smoothed_state[i] = smoothed_state[i-1];
    }
    imu_state_pub.publish(raw_state[0]);

  }

//Data smoothing function - use Gaussian 7-point smooth
//NOTE: smoothed values are actually centered about state 4
void IMUProcessor::smoothData()
{
  int size = 7;
  int coef[size] = {1, 3, 6, 7, 6, 3, 1};

  //Can not begin smoothing data unless there are 7 data points
  smoothed_state[0] = raw_state[0];
  if(cycles >= size) {
    //Obtain desired values and put in shorter variable names:
    //"av" = "Angular Velocity"
    //"la" = "Linear Acceleration"
    //row 0 = x-axis, row 1 = y-axis, row 2 = z-axis
    //col 0 = state 0, col 1 = state 1, etc.
    float av[3][size], la[3][size];
    for(int i = 0; i<size; i++) {
      av[0][i] = raw_state[i].angular_velocity.x;
      la[0][i] = raw_state[i].linear_acceleration.x;
    }
    for(int i = 0; i<size; i++) {
      av[1][i] = raw_state[i].angular_velocity.y;
      la[1][i] = raw_state[i].linear_acceleration.y;
    }
    for(int i = 0; i<size; i++) {
      av[2][i] = raw_state[i].angular_velocity.z;
      la[2][i] = raw_state[i].linear_acceleration.z;
    }

    //Set desired smoothed values to 0
    smoothed_state[3].angular_velocity.x = 0;
    smoothed_state[3].angular_velocity.y = 0;
    smoothed_state[3].angular_velocity.z = 0;
    smoothed_state[3].linear_acceleration.x = 0;
    smoothed_state[3].linear_acceleration.y = 0;
    smoothed_state[3].linear_acceleration.z = 0;

    //Smooth Data
    for(int i = 0; i<size; i++) {
      smoothed_state[3].angular_velocity.x += coef[i]*av[0][i]/size;
      smoothed_state[3].angular_velocity.y += coef[i]*av[1][i]/size;
      smoothed_state[3].angular_velocity.z += coef[i]*av[2][i]/size;

      smoothed_state[3].linear_acceleration.x += coef[i]*la[0][i]/size;
      smoothed_state[3].linear_acceleration.y += coef[i]*la[1][i]/size;
      smoothed_state[3].linear_acceleration.z += coef[i]*la[2][i]/size;
    }
  }
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
