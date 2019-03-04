#include "riptide_hardware/imu_processor.h"

#define PI 3.141592653

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_processor");
  IMUProcessor imu;
  ros::spin();
}

IMUProcessor::IMUProcessor() : nh("~")
{
  imu_filter_sub = nh.subscribe<imu_3dm_gx4::FilterOutput>("/imu/filter", 1, &IMUProcessor::FilterCallback, this);
  imu_mag_sub = nh.subscribe<imu_3dm_gx4::MagFieldCF>("/imu/magnetic_field", 1, &IMUProcessor::MagCallback, this);
  imu_verbose_state_pub = nh.advertise<riptide_msgs::ImuVerbose>("/state/imu_verbose", 1);
  imu_state_pub = nh.advertise<riptide_msgs::Imu>("/state/imu", 1);

  IMUProcessor::LoadParam<double>("declination", declination);
  IMUProcessor::LoadParam<double>("post_IIR_LPF_bandwidth", post_IIR_LPF_bandwidth);
  IMUProcessor::LoadParam<int>("filter_rate", filter_rate); // Filter rate MUST be an integer, decided by manufacturer

  // IIR LPF Variables
  double fc = post_IIR_LPF_bandwidth; // Shorthand variable for IIR bandwidth
  dt = 1.0 / filter_rate;
  alpha = 2 * PI * dt * fc / (2 * PI * dt * fc + 1); // Multiplier

  prev_ang_vel.x = 0;
  prev_ang_vel.y = 0;
  prev_ang_vel.z = 0;
  prev_linear_accel.x = 0;
  prev_linear_accel.y = 0;
  prev_linear_accel.z = 0;
}

// Load parameter from namespace
template <typename T>
void IMUProcessor::LoadParam(string param, T &var)
{
  try
  {
    if (!nh.getParam(param, var))
    {
      throw 0;
    }
  }
  catch (int e)
  {
    string ns = nh.getNamespace();
    ROS_ERROR("IMU Processor Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. SHutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

// Read magnetometer data and compute heading
void IMUProcessor::MagCallback(const imu_3dm_gx4::MagFieldCF::ConstPtr &mag_msg)
{
  // Read in body frame mag components
  magBX = mag_msg->components.x;
  magBY = mag_msg->components.y;
  magBZ = mag_msg->components.z;

  // Compute norm of body-frame mag vector
  IMUProcessor::Norm(magBX, magBY, magBZ, &mBX, &mBY, &mBZ);

  // Calculate x and y mag components in world frame using rotation matrix
  mWX = R_b2w.getRow(0).x() * mBX + R_b2w.getRow(0).y() * mBY + R_b2w.getRow(0).z() * mBZ;
  mWY = R_b2w.getRow(1).x() * mBX + R_b2w.getRow(1).y() * mBY + R_b2w.getRow(1).z() * mBZ;

  // Calculate heading with arctan (use atan2)
  heading = atan2(mWY, mWX) * 180 / PI;

  // Account for declination
  heading += declination; // Add declination value
  if (heading > 180.0)
  {                 // Keep heading in the range [-180, 180] deg.
    heading -= 360; // Subtract 360 deg.
  }
  else if (heading < -180.0)
  {
    heading += 360; //Add 360 deg.
  }
  verbose_state.heading = heading;

  //Flip the sign since positive z-axis points up
  verbose_state.euler_rpy.z = -verbose_state.heading;
}

// Normalize vector components, and write new values to specified address
void IMUProcessor::Norm(float v1, float v2, float v3, float *x, float *y, float *z)
{
  float magnitude = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
  *x = v1 / magnitude;
  *y = v2 / magnitude;
  *z = v3 / magnitude;
}

// Callback
void IMUProcessor::FilterCallback(const imu_3dm_gx4::FilterOutput::ConstPtr &filter_msg)
{
  // Put message data into verbose_state
  verbose_state.header = filter_msg->header;
  verbose_state.header.frame_id = "base_link";

  verbose_state.raw_euler_rpy = filter_msg->euler_rpy;
  verbose_state.euler_rpy.x = filter_msg->euler_rpy.x;
  verbose_state.euler_rpy.y = filter_msg->euler_rpy.y;
  verbose_state.gyro_bias = filter_msg->gyro_bias;
  verbose_state.euler_rpy_status = filter_msg->euler_rpy_status;

  // DO NOT set euler_rpy.z here. This value is calculated by the magnetometer
  // and is thus based on the magnetic field callback,

  verbose_state.heading_update = filter_msg->heading_update_LORD;
  verbose_state.heading_update_uncertainty = filter_msg->heading_update_uncertainty;
  verbose_state.heading_update_source = filter_msg->heading_update_source;
  verbose_state.heading_update_flags = filter_msg->heading_update_flags;

  verbose_state.raw_linear_accel = filter_msg->linear_acceleration;
  verbose_state.linear_accel = filter_msg->linear_acceleration;
  verbose_state.linear_accel_status = filter_msg->linear_acceleration_status;

  verbose_state.raw_ang_vel = filter_msg->angular_velocity;
  verbose_state.ang_vel = filter_msg->angular_velocity;
  verbose_state.ang_vel_status = filter_msg->angular_velocity_status;

  // Convert angular values from radians to degrees
  IMUProcessor::CvtRad2Deg();

  // Process Euler Angles (adjust heading and signs)
  IMUProcessor::ProcessEulerAngles();

  // Compute rotation matrix (use a yaw of 0 [rad])
  tf.setValue(verbose_state.euler_rpy.x * PI / 180, verbose_state.euler_rpy.y * PI / 180, 0);
  R_b2w.setRPY(tf.x(), tf.y(), tf.z()); //Body to world rotations --> world_vector =  R_b2w * body_vector

  // Smmoth angular velocity and linear accel with IIR LPF
  //IMUProcessor::SmoothDataIIR();

  // Process linear acceleration (Remove centrifugal and tangential components)

  // Process angular acceleration (Use 3-pt backwards rule to approximate angular acceleration)

  // Publish messages
  IMUProcessor::PopulateIMUState();
  imu_verbose_state_pub.publish(verbose_state);
  imu_state_pub.publish(imu_state);
}

// Convert all data fields from radians to degrees
void IMUProcessor::CvtRad2Deg()
{
  verbose_state.raw_euler_rpy.x *= (180.0 / PI);
  verbose_state.raw_euler_rpy.y *= (180.0 / PI);
  verbose_state.raw_euler_rpy.z *= (180.0 / PI);
  verbose_state.euler_rpy.x *= (180.0 / PI);
  verbose_state.euler_rpy.y *= (180.0 / PI);

  verbose_state.gyro_bias.x *= (180.0 / PI);
  verbose_state.gyro_bias.y *= (180.0 / PI);
  verbose_state.gyro_bias.z *= (180.0 / PI);

  verbose_state.heading_update *= (180 / PI);
  verbose_state.heading_update_uncertainty *= (180 / PI);

  verbose_state.raw_ang_vel.x *= (180.0 / PI);
  verbose_state.raw_ang_vel.y *= (180.0 / PI);
  verbose_state.raw_ang_vel.z *= (180.0 / PI);
}

// Adjust Euler angles to be consistent with the AUV's axes
void IMUProcessor::ProcessEulerAngles()
{
  // Adjust ROLL
  if (verbose_state.euler_rpy.x > -180 && verbose_state.euler_rpy.x < 0)
  {
    verbose_state.euler_rpy.x += 180;
  }
  else if (verbose_state.euler_rpy.x > 0 && verbose_state.euler_rpy.x < 180)
  {
    verbose_state.euler_rpy.x -= 180;
  }
  else if (verbose_state.euler_rpy.x == 0)
  {
    verbose_state.euler_rpy.x = 180;
  }
  else if (verbose_state.euler_rpy.x == 180 || verbose_state.euler_rpy.x == -180)
  {
    verbose_state.euler_rpy.x = 0;
  }

  // Adjust PITCH (negate the value - positive y-axis points left)
  verbose_state.euler_rpy.y *= -1;
}

// IIR LPF Smoothing Algorithm
void IMUProcessor::SmoothDataIIR()
{
  // Output = alpha*new + (1-alpha)*previous
  verbose_state.ang_vel.x = alpha * verbose_state.raw_ang_vel.x + (1 - alpha) * prev_ang_vel.x;
  verbose_state.ang_vel.y = alpha * verbose_state.raw_ang_vel.y + (1 - alpha) * prev_ang_vel.y;
  verbose_state.ang_vel.z = alpha * verbose_state.raw_ang_vel.z + (1 - alpha) * prev_ang_vel.z;

  verbose_state.linear_accel.x = alpha * verbose_state.raw_linear_accel.x + (1 - alpha) * prev_linear_accel.x;
  verbose_state.linear_accel.y = alpha * verbose_state.raw_linear_accel.y + (1 - alpha) * prev_linear_accel.y;
  verbose_state.linear_accel.z = alpha * verbose_state.raw_linear_accel.z + (1 - alpha) * prev_linear_accel.z;

  prev_ang_vel.x = verbose_state.ang_vel.x;
  prev_ang_vel.y = verbose_state.ang_vel.y;
  prev_ang_vel.z = verbose_state.ang_vel.z;
  prev_linear_accel.x = verbose_state.linear_accel.x;
  prev_linear_accel.y = verbose_state.linear_accel.y;
  prev_linear_accel.z = verbose_state.linear_accel.z;
}

// Populate imu_state message
void IMUProcessor::PopulateIMUState()
{
  imu_state.header = verbose_state.header;
  imu_state.euler_rpy = verbose_state.euler_rpy;
  imu_state.linear_accel = verbose_state.linear_accel;
  imu_state.ang_vel = verbose_state.ang_vel;
  imu_state.ang_accel = verbose_state.ang_accel;
}

// ROS loop function
void IMUProcessor::Loop()
{
  ros::Rate rate(1000);
  while (!ros::isShuttingDown())
  {
    ros::spinOnce();
    rate.sleep();
  }
}
