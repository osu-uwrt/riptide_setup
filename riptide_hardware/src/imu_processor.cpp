#include "riptide_hardware/imu_processor.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_processor");
  IMUProcessor imu;
  ros::spin();
}

IMUProcessor::IMUProcessor() : nh("~")
{
  std::string imu_name, filter_topic;
  IMUProcessor::LoadParam<std::string>("imu_name", imu_name);
  filter_topic = "/" + imu_name + "/filter"; // Get topic name using imu's namespace

  imu_filter_sub = nh.subscribe<imu_3dm_gx4::FilterOutput>(filter_topic, 1, &IMUProcessor::FilterCallback, this);
  imu_state_pub = nh.advertise<riptide_msgs::Imu>("/state/imu", 1);

  IMUProcessor::LoadParam<double>("post_IIR_LPF_bandwidth", post_IIR_LPF_bandwidth);
  IMUProcessor::LoadParam<int>("filter_rate", filter_rate); // Filter rate MUST be an integer, decided by manufacturer

  /*// IIR LPF Variables
  double fc = post_IIR_LPF_bandwidth; // Shorthand variable for IIR bandwidth
  dt = 1.0 / filter_rate;
  alpha = 2 * M_PI * dt * fc / (2 * M_PI * dt * fc + 1); // Multiplier*/

  // Linear Accel variables
  imu_position.setZero();
  pqr.setZero();
  pqr_dot.setZero();
  prev_pqr1.setZero();
  prev_pqr2.setZero();
  linear_accel.setZero();
  raw_accel.setZero();
  ang_accel.x = 0;
  ang_accel.y = 0;
  ang_accel.z = 0;
  dt = 0;
  init_count = 0;

  // Load relative positions between IMU and COM from YAML file
  IMUProcessor::LoadParam<std::string>("properties_file", properties_file);
  properties = YAML::LoadFile(properties_file);
  IMUProcessor::LoadIMUProperties();

  /*// IIR Smoothing Variables
  prev_ang_vel1.x = 0;
  prev_ang_vel1.y = 0;
  prev_ang_vel1.z = 0;
  prev_ang_vel2.x = 0;
  prev_ang_vel2.y = 0;
  prev_ang_vel2.z = 0;
  prev_linear_accel.x = 0;
  prev_linear_accel.y = 0;
  prev_linear_accel.z = 0;*/
}

// Load parameter from namespace
template <typename T>
void IMUProcessor::LoadParam(std::string param, T &var)
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
    std::string ns = nh.getNamespace();
    ROS_ERROR("IMU Processor Namespace: %s", ns.c_str());
    ROS_ERROR("Critical! Param \"%s/%s\" does not exist or is not accessed correctly. SHutting down.", ns.c_str(), param.c_str());
    ros::shutdown();
  }
}

void IMUProcessor::LoadIMUProperties()
{
  for (int i = 0; i < 3; i++)
    imu_position(i) = properties["properties"]["imu"][i].as<double>() - properties["properties"]["center_of_mass"][i].as<double>();
}

void IMUProcessor::FilterCallback(const imu_3dm_gx4::FilterOutput::ConstPtr &filter_msg)
{
  state.header = filter_msg->header;
  state.header.frame_id = filter_msg->header.frame_id;

  state.quaternion.x = filter_msg->quaternion.x;
  state.quaternion.y = filter_msg->quaternion.y;
  state.quaternion.z = filter_msg->quaternion.z;
  state.quaternion.w = filter_msg->quaternion.w;

  state.rpy_rad = filter_msg->euler_rpy;
  state.rpy_rad.z = filter_msg->heading_update_alt; // Use alternate heading calc
  state.rpy_deg.x = filter_msg->euler_rpy.x * (180 / M_PI);
  state.rpy_deg.y = filter_msg->euler_rpy.y * (180 / M_PI);
  state.rpy_deg.z = filter_msg->heading_update_alt * (180 / M_PI); // Use alternate heading calc

  state.heading_alt = filter_msg->heading_update_alt * (180 / M_PI);
  state.heading_LORD = filter_msg->heading_update_LORD * (180 / M_PI);

  state.ang_vel_rad = filter_msg->angular_velocity;
  state.ang_vel_deg.x = filter_msg->angular_velocity.x * (180 / M_PI);
  state.ang_vel_deg.y = filter_msg->angular_velocity.y * (180 / M_PI);
  state.ang_vel_deg.z = filter_msg->angular_velocity.z * (180 / M_PI);

  // Smmoth angular velocity and linear accel with IIR LPF
  /*raw_ang_vel.x = state.ang_vel_deg.x;
  raw_ang_vel.y = state.ang_vel_deg.y;
  raw_ang_vel.z = state.ang_vel_deg.z;
  raw_linear_accel.x = state.linear_accel.x;
  raw_linear_accel.y = state.linear_accel.y;
  raw_linear_accel.x = state.linear_accel.x;*/
  //IMUProcessor::SmoothDataIIR();

  // Process angular acceleration (Use 3-pt backwards rule to approximate angular acceleration)
  if (init_count < 2) // Need 2 previous data points to perform 3-pt difference backwards rule
  {
    prev_pqr2 = prev_pqr1;
    prev_pqr1(0) = state.ang_vel_rad.x;
    prev_pqr1(1) = state.ang_vel_rad.y;
    prev_pqr1(2) = state.ang_vel_rad.z;
    prev_time = ros::Time::now();
    init_count++;
  }
  else
  {
    ros::Time time = ros::Time::now();
    dt = time.toSec() - prev_time.toSec();
    pqr(0) = state.ang_vel_rad.x;
    pqr(1) = state.ang_vel_rad.y;
    pqr(2) = state.ang_vel_rad.z;
    pqr_dot = (3 * pqr - 4 * prev_pqr1 + prev_pqr2) / (2 * dt);
    
    state.ang_accel_rad.x = pqr_dot(0);
    state.ang_accel_rad.y = pqr_dot(1);
    state.ang_accel_rad.z = pqr_dot(2);
    state.ang_accel_deg.x = state.ang_accel_rad.x * 180 / M_PI;
    state.ang_accel_deg.y = state.ang_accel_rad.y * 180 / M_PI;
    state.ang_accel_deg.z = state.ang_accel_rad.z * 180 / M_PI;

    prev_pqr2 = prev_pqr1;
    prev_pqr1(0) = state.ang_vel_rad.x;
    prev_pqr1(1) = state.ang_vel_rad.y;
    prev_pqr1(2) = state.ang_vel_rad.z;
    prev_time = time;
  }
  
  // Process linear acceleration (Remove centrifugal and tangential components)
  raw_accel(0) = filter_msg->linear_acceleration.x;
  raw_accel(1) = filter_msg->linear_acceleration.y;
  raw_accel(2) = filter_msg->linear_acceleration.z;
  linear_accel = raw_accel - pqr_dot.cross(imu_position) - pqr.cross(pqr.cross(imu_position));
  state.linear_accel.x = linear_accel(0);
  state.linear_accel.y = linear_accel(1);
  state.linear_accel.z = linear_accel(2);
  imu_state_pub.publish(state);
}

/*// IIR LPF Smoothing Algorithm
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
}*/