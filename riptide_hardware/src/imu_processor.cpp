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
   imu_verbose_state_pub = nh.advertise<riptide_msgs::ImuVerbose>("state/imu_verbose", 1);
   imu_state_pub = nh.advertise<riptide_msgs::Imu>("state/imu", 1);

   zero_ang_vel_thresh = 1;
   cycles = 1;
   c = 3; //Index of center element in state array
   size = 7; //Size of state array
 }

 //Callback
  void IMUProcessor::callback(const imu_3dm_gx4::FilterOutput::ConstPtr& filter_msg)
  {
    //Put message data into state[0]
    state[0].header = filter_msg->header;
    state[0].header.frame_id = "base_link";
    state[0].raw_euler_rpy = filter_msg->euler_rpy;
    state[0].euler_rpy = filter_msg->euler_rpy;
    state[0].gyro_bias = filter_msg->gyro_bias;
    state[0].euler_rpy_status = filter_msg->euler_rpy_status;

    state[0].heading = filter_msg->heading;
    state[0].heading_uncertainty = filter_msg->heading_uncertainty;
    state[0].heading_update_source = filter_msg->heading_update_source;
    state[0].heading_flags = filter_msg->heading_flags;

    state[0].raw_linear_acceleration = filter_msg->linear_acceleration;
    state[0].linear_acceleration = filter_msg->linear_acceleration;
    state[0].linear_acceleration_status = filter_msg->linear_acceleration_status;

    state[0].raw_angular_velocity = filter_msg->angular_velocity;
    state[0].angular_velocity = filter_msg->angular_velocity;
    state[0].angular_velocity_status = filter_msg->angular_velocity_status;

    //Convert angular values from radians to degrees
    cvtRad2Deg();

    //Process Euler Angles (adjust heading and signs)
    processEulerAngles();

    //Set new delta time (convert nano seconds to seconds)
    if(state[1].dt == 0) { //If dt[0] not set yet, set to 0.01s, which is roughly what it would be
      state[0].dt = 0.01;
    }
    else { //dt has already been set, so find actual dt
      state[0].dt = (double)state[0].header.stamp.sec + state[0].header.stamp.nsec/(1.0e9) -
                    (double)state[1].header.stamp.sec - state[1].header.stamp.nsec/(1.0e9);
    }

    //Populate shorthand matrices for data smoothing
    av[0][0] = state[0].raw_angular_velocity.x;
    av[1][0] = state[0].raw_angular_velocity.y;
    av[2][0] = state[0].raw_angular_velocity.z;
    la[0][0] = state[0].raw_linear_acceleration.x;
    la[1][0] = state[0].raw_linear_acceleration.y;
    la[2][0] = state[0].raw_linear_acceleration.z;

    //Further process data
    if(cycles >= size) {
      smoothData();
      processDrift();
    }

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

    //Publish messages
    populateIMUState();
    imu_verbose_state_pub.publish(state[c]);
    imu_state_pub.publish(imu_state);

    //Adjust previous states and shorthand matrices
    for(int i=6; i>0; i--) {
      state[i] = state[i-1];

      for(int j=0; j<3; j++) {
        av[j][i] = av[j][i-1];
        la[j][i] = la[j][i-1];
      }
    }
  }

//Convert all data fields from radians to degrees
void IMUProcessor::cvtRad2Deg() {
  state[0].raw_euler_rpy.x *= (180.0/PI);
  state[0].raw_euler_rpy.y *= (180.0/PI);
  state[0].raw_euler_rpy.z *= (180.0/PI);
  state[0].euler_rpy.x *= (180.0/PI);
  state[0].euler_rpy.y *= (180.0/PI);
  state[0].euler_rpy.z *= (180.0/PI);

  state[0].gyro_bias.x *= (180.0/PI);
  state[0].gyro_bias.y *= (180.0/PI);
  state[0].gyro_bias.z *= (180.0/PI);

  state[0].heading *= (180/PI);
  state[0].heading_uncertainty *= (180/PI);

  state[0].raw_angular_velocity.x *= (180.0/PI);
  state[0].raw_angular_velocity.y *= (180.0/PI);
  state[0].raw_angular_velocity.z *= (180.0/PI);
  state[0].angular_velocity.x *= (180.0/PI);
  state[0].angular_velocity.y *= (180.0/PI);
  state[0].angular_velocity.z *= (180.0/PI);
}

//Adjust Euler angles to be consistent with the AUV's axes
void IMUProcessor::processEulerAngles() {
  //Set YAW equal to heading
  state[0].euler_rpy.z = state[0].heading;

  //Adjust ROLL
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

  //Adjust YAW (negate the value)
  state[0].euler_rpy.z *= -1;
}

//NOTE: This information is no longer needed, but remains here since it might
//be helpful if something goes wrong
//Check if raw angular velocity about any axis is approximately 0, and compute drift
void IMUProcessor::processDrift() {
  //X drift
  if(abs(state[c].angular_velocity.x) <= zero_ang_vel_thresh) {
    state[c].local_drift.x = state[c].euler_rpy.x - state[c+1].euler_rpy.x;
  }
  else {
    state[c].local_drift.x = 0;
  }
  state[c].local_drift_rate.x = state[c].local_drift.x / state[c].dt;

  //Y drift
  if(abs(state[c].angular_velocity.y) <= zero_ang_vel_thresh) {
    state[c].local_drift.y = state[c].euler_rpy.y - state[c+1].euler_rpy.y;
  }
  else {
    state[c].local_drift.y = 0;
  }
  state[c].local_drift_rate.y = state[c].local_drift.y / state[c].dt;

  //Z drift
  if(abs(state[c].angular_velocity.z) <= zero_ang_vel_thresh) {
    state[c].local_drift.z = state[c].euler_rpy.z - state[c+1].euler_rpy.z;    }
  else {
    state[c].local_drift.z = 0;
  }
  state[c].local_drift_rate.z = state[c].local_drift.z / state[c].dt;
}

//Smooth Angular Velocity and Linear Acceleration with a Gaussian 7-point smooth
//NOTE: Smoothed values are actually centered about the middle state within
//the state array, state 4 (index 3)
void IMUProcessor::smoothData()
{
  int coef[size] = {1, 3, 6, 7, 6, 3, 1};
  int sumCoef = 27;

  //Can not begin smoothing data unless there are 7 data points
  state[0] = state[0];

  //Set all desired values to smooth to 0
  state[c].angular_velocity.x = 0;
  state[c].angular_velocity.y = 0;
  state[c].angular_velocity.z = 0;
  state[c].linear_acceleration.x = 0;
  state[c].linear_acceleration.y = 0;
  state[c].linear_acceleration.z = 0;

  //Smooth Data
  //Reminder for using shorthand matrices:
  //"av" = "Angular Velocity"
  //"la" = "Linear Acceleration"
  //row 0 = x-axis, row 1 = y-axis, row 2 = z-axis
  //col 0 = state 0, col 1 = state 1, etc.
  for(int i = 0; i<size; i++) {
    state[c].angular_velocity.x += coef[i]*av[0][i]/sumCoef;
    state[c].angular_velocity.y += coef[i]*av[1][i]/sumCoef;
    state[c].angular_velocity.z += coef[i]*av[2][i]/sumCoef;

    state[c].linear_acceleration.x += coef[i]*la[0][i]/sumCoef;
    state[c].linear_acceleration.y += coef[i]*la[1][i]/sumCoef;
    state[c].linear_acceleration.z += coef[i]*la[2][i]/sumCoef;
  }
}

//Populate imu_state message
void IMUProcessor::populateIMUState() {
  imu_state.header = state[c].header;
  imu_state.euler_rpy = state[c].euler_rpy;
  imu_state.linear_acceleration = state[c].linear_acceleration;
  imu_state.angular_velocity = state[c].angular_velocity;
  imu_state.angular_acceleration = state[c].angular_acceleration;
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
