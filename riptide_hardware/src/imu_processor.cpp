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
  string imu_name, filter_topic;
  IMUProcessor::LoadParam<string>("imu_name", imu_name);
  filter_topic = "/" + imu_name + "/filter"; // Get topic name using imu's namespace

  imu_filter_sub = nh.subscribe<imu_3dm_gx4::FilterOutput>(filter_topic, 1, &IMUProcessor::FilterCallback, this);
  imu_state_pub = nh.advertise<riptide_msgs::Imu>("/state/imu", 1);

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

void IMUProcessor::FilterCallback(const imu_3dm_gx4::FilterOutput::ConstPtr &filter_msg)
{
  state.header = filter_msg->header;
  state.header.frame_id = filter_msg->header.frame_id;

  state.rpy_rad = filter_msg->euler_rpy;
  state.rpy_rad.z = filter_msg->heading_update_alt; // Use alternate heading calc
  state.rpy_deg.x = filter_msg->euler_rpy.x * (180 / PI);
  state.rpy_deg.y = filter_msg->euler_rpy.y * (180 / PI);
  state.rpy_deg.z = filter_msg->heading_update_alt * (180 / PI); // Use alternate heading calc

  state.heading_alt = filter_msg->heading_update_alt * (180 / PI);
  state.heading_LORD = filter_msg->heading_update_LORD * (180 / PI);

  state.linear_accel = filter_msg->linear_acceleration;
  state.ang_vel_rad = filter_msg->angular_velocity;
  state.ang_vel_deg.x = filter_msg->angular_velocity.x * (180 / PI);
  state.ang_vel_deg.y = filter_msg->angular_velocity.y * (180 / PI);
  state.ang_vel_deg.z = filter_msg->angular_velocity.z * (180 / PI);

  // Smmoth angular velocity and linear accel with IIR LPF
  raw_ang_vel.x = state.ang_vel_deg.x;
  raw_ang_vel.y = state.ang_vel_deg.y;
  raw_ang_vel.z = state.ang_vel_deg.z;
  raw_linear_accel.x = state.linear_accel.x;
  raw_linear_accel.y = state.linear_accel.y;
  raw_linear_accel.x = state.linear_accel.x;
  //IMUProcessor::SmoothDataIIR();

  // Process linear acceleration (Remove centrifugal and tangential components)

  // Process angular acceleration (Use 3-pt backwards rule to approximate angular acceleration)

  imu_state_pub.publish(state);
}

// IIR LPF Smoothing Algorithm
void IMUProcessor::SmoothDataIIR()
{
  // Output = alpha*new + (1-alpha)*previous
  state.ang_vel_deg.x = alpha * raw_ang_vel.x + (1 - alpha) * prev_ang_vel.x;
  state.ang_vel_deg.y = alpha * raw_ang_vel.y + (1 - alpha) * prev_ang_vel.y;
  state.ang_vel_deg.z = alpha * raw_ang_vel.z + (1 - alpha) * prev_ang_vel.z;

  state.linear_accel.x = alpha * raw_linear_accel.x + (1 - alpha) * prev_linear_accel.x;
  state.linear_accel.y = alpha * raw_linear_accel.y + (1 - alpha) * prev_linear_accel.y;
  state.linear_accel.z = alpha * raw_linear_accel.z + (1 - alpha) * prev_linear_accel.z;

  prev_ang_vel.x = state.ang_vel_deg.x;
  prev_ang_vel.y = state.ang_vel_deg.y;
  prev_ang_vel.z = state.ang_vel_deg.z;
  prev_linear_accel.x = state.linear_accel.x;
  prev_linear_accel.y = state.linear_accel.y;
  prev_linear_accel.z = state.linear_accel.z;
}
