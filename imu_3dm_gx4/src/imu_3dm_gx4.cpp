#include <ros/ros.h>
#include <ros/node_handle.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <string>
#include <cmath>

#include <imu_3dm_gx4/FilterOutput.h>
#include <imu_3dm_gx4/MagFieldCF.h>
#include "imu.hpp"

using namespace imu_3dm_gx4;

#define kEarthGravity (9.80665)
#define PI (3.141592653)

ros::Publisher pubIMU;
ros::Publisher pubMag;
ros::Publisher pubPressure;
ros::Publisher pubFilter;
std::string frameId;

Imu::Info info;
Imu::DiagnosticFields fields;

//  diagnostic_updater resources
std::shared_ptr<diagnostic_updater::Updater> updater;
std::shared_ptr<diagnostic_updater::TopicDiagnostic> imuDiag;
std::shared_ptr<diagnostic_updater::TopicDiagnostic> filterDiag;

void publishData(const Imu::IMUData &data) {
  sensor_msgs::Imu imu;
  imu_3dm_gx4::MagFieldCF field;
  sensor_msgs::FluidPressure pressure;

  //  assume we have all of these since they were requested
  /// @todo: Replace this with a mode graceful failure...
  assert(data.fields & Imu::IMUData::Accelerometer);
  assert(data.fields & Imu::IMUData::Magnetometer);
  assert(data.fields & Imu::IMUData::Barometer);
  assert(data.fields & Imu::IMUData::Gyroscope);

  //  timestamp identically
  imu.header.stamp = ros::Time::now();
  imu.header.frame_id = frameId;
  field.header.stamp = imu.header.stamp;
  field.header.frame_id = frameId;
  pressure.header.stamp = imu.header.stamp;
  pressure.header.frame_id = frameId;

  imu.orientation_covariance[0] =
      -1; //  orientation data is on a separate topic

  imu.linear_acceleration.x = data.accel[0] * kEarthGravity;
  imu.linear_acceleration.y = data.accel[1] * kEarthGravity;
  imu.linear_acceleration.z = data.accel[2] * kEarthGravity;
  imu.angular_velocity.x = data.gyro[0];
  imu.angular_velocity.y = data.gyro[1];
  imu.angular_velocity.z = data.gyro[2];

  field.mag_field_components.x = data.mag[0];
  field.mag_field_components.y = data.mag[1];
  field.mag_field_components.z = data.mag[2];
  field.mag_field_magnitude = sqrt(data.mag[0]*data.mag[0] +
    data.mag[1]*data.mag[1] + data.mag[2]*data.mag[2]);

  pressure.fluid_pressure = data.pressure;

  //  publish
  pubIMU.publish(imu);
  pubMag.publish(field);
  pubPressure.publish(pressure);
  if (imuDiag) {
    imuDiag->tick(imu.header.stamp);
  }
}

void publishFilter(const Imu::FilterData &data) {
  assert(data.fields & Imu::FilterData::Quaternion);
  assert(data.fields & Imu::FilterData::OrientationEuler);
  assert(data.fields & Imu::FilterData::Acceleration);
  assert(data.fields & Imu::FilterData::AngularRate);
  assert(data.fields & Imu::FilterData::Bias);
  assert(data.fields & Imu::FilterData::AngleUnertainty);
  assert(data.fields & Imu::FilterData::BiasUncertainty);

  imu_3dm_gx4::FilterOutput output;
  output.header.stamp = ros::Time::now();
  output.header.frame_id = frameId;

  output.quaternion.w = data.quaternion[0];
  output.quaternion.x = data.quaternion[1];
  output.quaternion.y = data.quaternion[2];
  output.quaternion.z = data.quaternion[3];
  output.quaternion_status = data.quaternionStatus;

  output.euler_rpy.x = data.eulerRPY[0];
  output.euler_rpy.y = data.eulerRPY[1];
  output.euler_rpy.z = data.eulerRPY[2];
  output.euler_rpy_status = data.eulerRPYStatus;

  output.euler_angle_covariance[0] = data.eulerAngleUncertainty[0]*
      data.eulerAngleUncertainty[0];
  output.euler_angle_covariance[4] = data.eulerAngleUncertainty[1]*
      data.eulerAngleUncertainty[1];
  output.euler_angle_covariance[8] = data.eulerAngleUncertainty[2]*
      data.eulerAngleUncertainty[2];
  output.euler_angle_covariance_status = data.eulerAngleUncertaintyStatus;

  output.gyro_bias.x = data.gyroBias[0];
  output.gyro_bias.y = data.gyroBias[1];
  output.gyro_bias.z = data.gyroBias[2];
  output.gyro_bias_status = data.gyroBiasStatus;

  output.gyro_bias_covariance[0] = data.gyroBiasUncertainty[0]*data.gyroBiasUncertainty[0];
  output.gyro_bias_covariance[4] = data.gyroBiasUncertainty[1]*data.gyroBiasUncertainty[1];
  output.gyro_bias_covariance[8] = data.gyroBiasUncertainty[2]*data.gyroBiasUncertainty[2];
  output.gyro_bias_covariance_status = data.gyroBiasUncertaintyStatus;

  output.heading_update = data.headingUpdate;
  output.heading_update_uncertainty = data.headingUpdateUncertainty;
  output.heading_update_source = data.headingUpdateSource;
  output.heading_update_flags = data.headingUpdateFlags;

  output.linear_acceleration.x = data.acceleration[0];
  output.linear_acceleration.y = data.acceleration[1];
  output.linear_acceleration.z = data.acceleration[2];
  output.linear_acceleration_status = data.accelerationStatus;

  output.angular_velocity.x = data.angularRate[0];
  output.angular_velocity.y = data.angularRate[1];
  output.angular_velocity.z = data.angularRate[2];
  output.angular_velocity_status = data.angularRateStatus;

  pubFilter.publish(output);
  if (filterDiag) {
    filterDiag->tick(output.header.stamp);
  }
}

std::shared_ptr<diagnostic_updater::TopicDiagnostic> configTopicDiagnostic(
    const std::string& name, double * target) {
  std::shared_ptr<diagnostic_updater::TopicDiagnostic> diag;
  const double period = 1.0 / *target;  //  for 1000Hz, period is 1e-3

  diagnostic_updater::FrequencyStatusParam freqParam(target, target, 0.01, 10);
  diagnostic_updater::TimeStampStatusParam timeParam(0, period * 0.5);
  diag.reset(new diagnostic_updater::TopicDiagnostic(name,
                                                     *updater,
                                                     freqParam,
                                                     timeParam));
  return diag;
}

void updateDiagnosticInfo(diagnostic_updater::DiagnosticStatusWrapper& stat,
                          imu_3dm_gx4::Imu* imu) {
  //  add base device info
  std::map<std::string,std::string> map = info.toMap();
  for (const std::pair<std::string,std::string>& p : map) {
    stat.add(p.first, p.second);
  }

  try {
    //  try to read diagnostic info
    imu->getDiagnosticInfo(fields);

    auto map = fields.toMap();
    for (const std::pair<std::string, unsigned int>& p : map) {
      stat.add(p.first, p.second);
    }
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Read diagnostic info.");
  }
  catch (std::exception& e) {
    const std::string message = std::string("Failed: ") + e.what();
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, message);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_3dm_gx4");
  ros::NodeHandle nh("~");

  std::string device;
  int baudrate;
  bool enableMagUpdate, enableAccelUpdate;
  int requestedImuRate, requestedFilterRate;
  bool verbose;

  // Variables for IMU Reference Position
  std::string headingUpdateSource, declinationSource;
  std::string name, city, location;
  float roll, pitch, yaw;
  double latitude, longitude, altitude, manualDeclination;
  bool enable_sensor_to_vehicle_tf;

  // Sensor LPF Bandwidths
  int magLPFBandwidth3DM, accelLPFBandwidth3DM, gyroLPFBandwidth3DM;

  // Hard and Soft Iron Offset
  bool enable_iron_offset;
  float hx, hy, hz, m11, m12, m13, m21, m22, m23, m31, m32, m33;

  // Variabls for Magnetometer Magnitude Error Adaptive Measurements
  /*bool enableMagErrAdaptMsmt;
  float magLPFBandwidth, magLowLim, magHighLim, magLowLimUncertainty;
  float magHighLimUncertainty, magMinUncertainty;
  float LPFBandwidth, lowLim, highLim, lowLimUncertainty;
  float highLimUncertainty, minUncertainty;*/

  // Load Main Parameters from Launch File
  nh.param<std::string>("device", device, "/dev/imu_front");
  nh.param<int>("baudrate", baudrate, 115200);
  nh.param<std::string>("frame_id", frameId, std::string("imu"));
  nh.param<int>("imu_rate", requestedImuRate, 100);
  nh.param<int>("filter_rate", requestedFilterRate, 100);
  nh.param<bool>("enable_mag_update", enableMagUpdate, true);
  nh.param<bool>("enable_accel_update", enableAccelUpdate, true);
  nh.param<bool>("verbose", verbose, false);

  // Parameters for IMU Reference Position
  nh.param<std::string>("name", name, (std::string)"imu_front");
  nh.param<std::string>("city", city, (std::string)"columbus");
  nh.param<std::string>("location", location, (std::string)"CAR");
  nh.param<double>("latitude", latitude, 39.9984f); //Default is Columbus latitude
  nh.param<double>("longitude", longitude, -83.0179f); //Default is Columbus longitude
  nh.param<double>("altitude", altitude, 224.0f); //Default is Columbus altitude
  nh.param<double>("declination", manualDeclination, 7.01f); //Default is Columbus declination
  nh.param<float>("roll", roll, 0.0f); //Default is 0.0
  nh.param<float>("pitch", pitch, -90.0f); //Default is -90.0
  nh.param<float>("yaw", yaw, 180.0f); //Default is 180.0
  nh.param<bool>("enable_sensor_to_vehicle_tf", enable_sensor_to_vehicle_tf, true); //Default is Columbus declination
  nh.param<std::string>("heading_update_source", headingUpdateSource, std::string("magnetometer")); //Default is magnetometer
  nh.param<std::string>("declination_source", declinationSource, std::string("manual")); //Default is World Magnetic Model

  nh.param<int>("mag_LPF_bandwidth", magLPFBandwidth3DM, 1);
  nh.param<int>("accel_LPF_bandwidth", accelLPFBandwidth3DM, 25);
  nh.param<int>("gyro_LPF_bandwidth", gyroLPFBandwidth3DM, 25);

  nh.param<bool>("enable_iron_offset", enable_iron_offset, false);
  nh.param<float>("hx", hx, 0.0);
  nh.param<float>("hy", hy, 0.0);
  nh.param<float>("hz", hz, 0.0);
  nh.param<float>("m11", m11, 1.0);
  nh.param<float>("m12", m12, 0.0);
  nh.param<float>("m13", m13, 0.0);
  nh.param<float>("m2", m21, 0.0);
  nh.param<float>("m22", m22, 1.0);
  nh.param<float>("m23", m23, 0.0);
  nh.param<float>("m31", m31, 0.0);
  nh.param<float>("m32", m32, 0.0);
  nh.param<float>("m33", m33, 1.0);
  float hard_offset[3] = {hx, hy, hz};
  float soft_matrix[9] = {m11, m12, m13, m21, m22, m23, m31, m32, m33};

  // Parameters to adjust Magnetometer Magnitude Error Adaptive Measurement
  /*nh.param<bool>("enable_mag_err_adapt_msmt", enableMagErrAdaptMsmt, false);
  nh.param<float>("LPF_bandwidth", magLPFBandwidth, 25);
  nh.param<float>("low_limit", magLowLim, -0.1);
  nh.param<float>("high_limit", magHighLim, 0.1);
  nh.param<float>("low_limit_uncertainty", magLowLimUncertainty, 0.1);
  nh.param<float>("high_limit_uncertainty", magHighLimUncertainty, 0.1);
  nh.param<float>("min_uncertainty", magMinUncertainty, 0.05);*/

  if (requestedFilterRate < 0 || requestedImuRate < 0) {
    ROS_ERROR("imu_rate and filter_rate must be > 0");
    return -1;
  }

  pubIMU = nh.advertise<sensor_msgs::Imu>("imu", 1);
  pubMag = nh.advertise<imu_3dm_gx4::MagFieldCF>("magnetic_field", 1);
  pubPressure = nh.advertise<sensor_msgs::FluidPressure>("pressure", 1);
  pubFilter = nh.advertise<imu_3dm_gx4::FilterOutput>("filter", 1);

  // Ceate new instance of the IMU
  Imu imu(device, verbose);
  try {
    imu.connect();

    ROS_INFO("Selecting baud rate %u", baudrate);
    imu.selectBaudRate(baudrate);

    ROS_INFO("Fetching device info.");
    imu.getDeviceInfo(info);
    std::map<std::string,std::string> map = info.toMap();
    for (const std::pair<std::string,std::string>& p : map) {
      ROS_INFO("\t%s: %s", p.first.c_str(), p.second.c_str());
    }

    ROS_INFO("Idling the device");
    imu.idle();

    // Read back data rates
    uint16_t imuBaseRate, filterBaseRate;
    imu.getIMUDataBaseRate(imuBaseRate);
    ROS_INFO("IMU data base rate: %u Hz", imuBaseRate);
    imu.getFilterDataBaseRate(filterBaseRate);
    ROS_INFO("Filter data base rate: %u Hz", filterBaseRate);

    // Calculate and set decimation rates
    if (static_cast<uint16_t>(requestedImuRate) > imuBaseRate) {
      throw std::runtime_error("imu_rate cannot exceed " +
                               std::to_string(imuBaseRate));
    }
    if (static_cast<uint16_t>(requestedFilterRate) > filterBaseRate) {
      throw std::runtime_error("filter_rate cannot exceed " +
                               std::to_string(filterBaseRate));
    }

    const uint16_t imuDecimation = imuBaseRate / requestedImuRate;
    const uint16_t filterDecimation = filterBaseRate / requestedFilterRate;

    ROS_INFO("Selecting IMU decimation: %u", imuDecimation);
    //The following variables are taken from 'enum' in the struct called IMUData
    imu.setIMUDataRate(
        imuDecimation, Imu::IMUData::Accelerometer |
          Imu::IMUData::Gyroscope |
          Imu::IMUData::Magnetometer |
          Imu::IMUData::Barometer);

    ROS_INFO("Selecting filter decimation: %u", filterDecimation);
    //The following variables are taken from 'enum' in the struct called FilterData
    imu.setFilterDataRate(filterDecimation, Imu::FilterData::Quaternion |
                          Imu::FilterData::OrientationEuler |
                          Imu::FilterData::HeadingUpdate |
                          Imu::FilterData::Acceleration |
                          Imu::FilterData::AngularRate |
                          Imu::FilterData::Bias |
                          Imu::FilterData::AngleUnertainty |
                          Imu::FilterData::BiasUncertainty);

    ROS_INFO("Enabling IMU data stream");
    imu.enableIMUStream(true);


    ROS_INFO("Enabling filter data stream");
    imu.enableFilterStream(true);

    ROS_INFO("Enabling filter measurements");
    imu.enableMeasurements(enableAccelUpdate, enableMagUpdate);

    ROS_INFO("Enabling gyro bias estimation");
    imu.enableBiasEstimation(true);

    imu.setIMUDataCallback(publishData);
    imu.setFilterDataCallback(publishFilter);

    // Additional IMU Settings //////////////////////////////////////////////
    //Set parameters and display them to console thru ROS_INFO
    //Convert to radians
    roll *= (PI/180);
    pitch *= (PI/180);
    yaw *= (PI/180);
    manualDeclination *= (PI/180);

    ROS_INFO("IMU Name = %s", name.c_str());
    ROS_INFO("City = %s", city.c_str());
    ROS_INFO("Location = %s", location.c_str());

    ROS_INFO("Sensor to Vehicle Frame Transformation");
    imu.setSensorToVehicleTF(roll, pitch, yaw);
    ROS_INFO("\tRoll (deg): %f", roll*180/PI);
    ROS_INFO("\tPitch (deg): %f", pitch*180/PI);
    ROS_INFO("\tYaw (deg): %f", yaw*180/PI);

    ROS_INFO("Heading Update Source");
    imu.setHeadingUpdateSource(headingUpdateSource);
    ROS_INFO("\tUpate Source: %s", headingUpdateSource.c_str());

    ROS_INFO("Reference Position");
    imu.setReferencePosition(latitude, longitude, altitude);
    ROS_INFO("\tLatitude (deg): %f", latitude);
    ROS_INFO("\tLongitude (deg): %f", longitude);
    ROS_INFO("\tAltitude (m): %f", altitude);

    ROS_INFO("Declination Source");
    imu.setDeclinationSource(declinationSource, manualDeclination);
    ROS_INFO("\tDec Source: %s", declinationSource.c_str());
    ROS_INFO("\tManual Dec (deg): %f", manualDeclination*180/PI);

    ROS_INFO("Sensor LPF Bandwidths");
    imu.setLPFBandwidth("mag", "IIR", "manual", magLPFBandwidth3DM);
    imu.setLPFBandwidth("accel", "IIR", "manual", accelLPFBandwidth3DM);
    imu.setLPFBandwidth("gyro", "IIR", "manual", gyroLPFBandwidth3DM);
    ROS_INFO("\tMag LPF (Hz): %i", magLPFBandwidth3DM);
    ROS_INFO("\tAccel LPF (Hz): %i", accelLPFBandwidth3DM);
    ROS_INFO("\tGyro LPF (Hz): %i", gyroLPFBandwidth3DM);

    ROS_INFO("Setting Hard and Soft Iron Offsets");
    if(enable_iron_offset) {
      imu.setHardIronOffset(hard_offset);
      imu.setSoftIronMatrix(soft_matrix);
    }
    ROS_INFO("\tEnable Status: %i", enable_iron_offset);
    ROS_INFO("\tFile: %s_%s_%s.yaml", name.c_str(), city.c_str(), location.c_str());

    /*ROS_INFO("Magnetometer Magnitude Err. Adapt Msmt");
    imu.setMagFilterErrAdaptMsmt(enableMagErrAdaptMsmt, magLPFBandwidth,
        magLowLim, magHighLim, magLowLimUncertainty, magHighLimUncertainty,
        magMinUncertainty);
    imu.getMagFilterErrAdaptMsmt(LPFBandwidth, lowLim, highLim,
      lowLimUncertainty, highLimUncertainty, minUncertainty);

    std::string boolValue = "enabled";
    if(!enableMagErrAdaptMsmt) {
      boolValue = "disabled";
    }

    ROS_INFO("\tEnable Status: %s", boolValue.c_str());
    ROS_INFO("\tLPF Bandwidth (Hz): %f", LPFBandwidth);
    ROS_INFO("\tLow Lim (Gauss): %f", lowLim);
    ROS_INFO("\tHigh Lim (Gauss): %f", highLim);
    ROS_INFO("\tLow Lim Uncrty (Gauss): %f", lowLimUncertainty);
    ROS_INFO("\tHigh Lim Uncrty (Gauss): %f", highLimUncertainty);
    ROS_INFO("\tMin Uncrty (Gauss): %f", minUncertainty);*/
    //////////////////////////////////////////////////////////////////////////

    // Configure diagnostic updater
    if (!nh.hasParam("diagnostic_period")) {
      nh.setParam("diagnostic_period", 0.2);  //  5hz period
    }

    updater.reset(new diagnostic_updater::Updater());
    const std::string hwId = info.modelName + "-" + info.modelNumber;
    updater->setHardwareID(hwId);

    // Calculate the actual rates we will get
    double imuRate = imuBaseRate / (1.0 * imuDecimation);
    double filterRate = filterBaseRate / (1.0 * filterDecimation);
    imuDiag = configTopicDiagnostic("imu",&imuRate);
    filterDiag = configTopicDiagnostic("filter",&filterRate);

    updater->add("diagnostic_info",
                 boost::bind(&updateDiagnosticInfo, _1, &imu));

    ROS_INFO("Resuming the device");
    imu.resume();

    while (ros::ok()) {
      imu.runOnce();
      updater->update();
    }
    imu.disconnect();
  }
  catch (Imu::io_error &e) {
    ROS_ERROR("IO error: %s\n", e.what());
  }
  catch (Imu::timeout_error &e) {
    ROS_ERROR("Timeout: %s\n", e.what());
  }
  catch (std::exception &e) {
    ROS_ERROR("Exception: %s\n", e.what());
  }

  return 0;
}
