# The imu_3dm_gx4 Package

![Picture of IMU](https://www.microstrain.com/sites/default/files/styles/larger__550x550_/public/gx4-25.jpg?itok=vB8GWQpI)

The `imu_3dm_gx4` package provides support for the [Lord Corporation](http://www.microstrain.com) Microstrain [3DM-GX4-25](http://www.microstrain.com/inertial/3dm-gx4-25) series IMU. The package employs the MIP packet format, so it could conceivably be adapted to support other versions of Microstrain products with relatively little effort. At present, the 15 and 45 series AHRS systems are not supported.

Supported platforms: Ubuntu 12.04, 14.04, and 16.04

NOTE:
This package originates from [KumarRobotics/imu_3dm_gx4](https://github.com/KumarRobotics/imu_3dm_gx4)) and has been adapted extensively from the 0.0.4 version.

## Version History (KumarRobotics versions are 0.0.1 - 0.0.4)

* **0.1.4**
  - Removes `enable_filter` argument so the Adaptive Extended Kalman Filter (AEKF) output is always present.
  - Updated `/imu/magnetic_field` message type from `sensor_msgs/MagneticField` to `imu_3dm_gx4/msg/MagFieldCF`.
  - Added more data fields provided by AEKF to `imu_3dm_gx4/FilterOutput` message.
  - Added launch file arguments for: reference location, heading update source, declination source, sensor LPF bandwidths, and enabling iron offsets loaded from file
* **0.0.4**:
  - Fixed issue where packets would be dropped if the header checksum was broken up
  into multiple packets.
  - Added `verbose` option.
* **0.0.3**:
  - Replaced `decimation` options with `rate` options. Decimation is automatically calculated.
  - Added a file for use with Kalibr.
  - Added a udev rule to configure permissions.
* **0.0.2**:
  - Units of acceleration are now m/s^2, in agreement with ROS convention.
  - Cleaned up code base, replaced error codes with exceptions.
  - Status can now be viewed from `rqt_runtime_monitor`.
  - Filter output is now in a single, custom message with covariances and important status flags.
  - Added option to enable/disable accelerometer update in the estimator.
  - Removed TF broadcast option.
  - Reformatted code base to clang-llvm convention.
* **0.0.1**:
  - First release.

## Options

The `imu_3dm_gx4` node supports the following base options:
* `device`: Path to the device in `/dev`. Defaults to `/dev/ttyACM0`.
* `baudrate`: Baudrate to employ with serial communication. Defaults to `115200`.
* `frame_id`: Frame to use in headers.
* `imu_rate`: IMU rate to use, in Hz. Default is 100.
* `verbose`: If true, packet reads and mismatched checksums will be logged.

The following options allow the user to change the IMU's internal infinite impulse response (IIR) low pass filter (LPF) bandwidths on each of the IMU's sensors. A value of `-1` will disable the internal IIR LPF.
* `mag_LPF_Bandwidth` (Default is `15`): LPF bandwidth for the magnetometer
* `accel_LPF_Bandwidth` (Default is `50`): LPF bandwidth for the accelerometer
* `gyro_LPF_Bandwidth` (Default is `50`): LPF bandwidth for the gyroscope

The following option is for setting the IMU to load a pre-defined set of Hard and Soft Iron coefficients:
* `enable_iron_offset` (Default is `false`); Indicates if IMU should load pre-defined coefficients from a YAML file. PROVIDE DATA FORMAT

The sensor-to-vehicle transformation feature is always active (with default values of `0 deg.`). In case when the sensor frame does not align with that of the vehicle body-frame, then the user may specify the transformation (via Euler Angles in the order of yaw, pitch, and roll) from the sensor TO the body-frame. The user simply needs to define the following three data fields in a YAML file called `sensor_to_vehicle_tf.yaml`
* `roll`
* `pitch`
* `yaw`

The following options are for high-level usage of the IMU's onboard estimation filter:
* `filter_rate` (Default is `100`): Filter rate to use, in Hz.
* `enable_mag_update` (Default is `true`): Indicates if the IMU should use its internal magnetometer to correct the heading angle estimate. The updated heading has a separate data field within the `/imu/FilterOutput` message called `heading_update`.
* `enable_accel_update` (Default is `true`): Indicates if the IMU will use the accelerometer to correct the roll/pitch angle estimates. Corrections do NOT get stored to separate fields within the `/imu/FilterOutput` message, they affect the actual values themselves.

The following options are required for the heading update feature:
* `name` (Default is `imu_front`): Common name for the IMU
* `city` (Default is `columbus`): City in which reference location resides
* `location` (Default is `CAR`): Actual reference location. Other examples: `apartment`
* `heading_update_source` (Default is `magnetometer`): Possible options are: `none`, `external`, or `magnetometer`
  - Note: `magnetometer` indicates the IMU should uses its internal magnetometer. 
  - IMPORTANT: The 3DM-GX4-25 model does NOT perform the heading update well in the presence of EMI. Even though setting the filtered magnetometer data (from the IIR LPF) is supposed to be used in the correction step, the heading update still exhibits considerable sinusoidal behavior. This driver offers an ALTERNATIVE heading update feature which performs the calculation correctly.
* `declination_source` (Default is `wmm`): Possible options are: `none`, `wmm`, or `manual`
  - Note: `wmm` indicates the IMU should use its internal World Magnetometer Model (the GX4 has the 2005 model preloaded)

**In order to launch the node** (streaming IMU data at 100Hz), execute:

`$ roslaunch imu_3dm_gx4 imu.launch`

### Note on rate parameters:

The `_rate` options command the requested frequency by calculating a 'decimation value', which is defined by the relation:

```
  frequency = base_frequency / decimation
```

Where the base frequency is 1kHz for the GX4. Since decimation values are integers, certain frequencies cannot be expressed by the above relation. 800Hz, for example, cannot be specified by an integer decimation rate. In these cases, you will receive whichever frequency is obtained by rounding decimation down to the nearest integer. Hence, requesting 800Hz will produce 1kHz.

## Output

On launch, the node will configure the IMU according to the parameters and then enable streaming node. The following topics are published with syncrhrnoized timestamps:

* `/imu_3dm_gx4/imu`: An instance of `sensor_msgs/Imu`. Orientation quaternion not provided in this message since is already part of the filter message.
* `/imu_3dm_gx4/magnetic_field`: An instance of `imu_3dm_gx4/MagFieldCF`. This essage contains:
  - The magnetic field components (Gauss)
  - Total magnetic field magnitude (Gauss)
  - Covariance of the magnetic field (Gauss^2)
* `/imu_3dm_gx4/pressure`: An instance of `sensor_msgs/FluidPressure`.

The topic for the IMU's estimation filter is published on a separate topic on an asyhcnronous timestamp:

* `/imu_3dm_gx4/filter`: An instance of `imu_3dm_gx4/FilterOuput`. This message has been revised from the original KumarRobotics message to include additional data fields. This message contains
  - Orientation estimates in Euler Angles (rad) and the covariances (rad^2), and status flags
  - Orientation estimates in Quaternions and status flags
  - Filtered linear accelerations (m/s^2) and angular velocities (rad/s), and status flags
  - The heading update (as performed by Lord Microstrain) as well as the heading update from this driver (the recommended field), both in units of (rad)
  - Gyro bias and covariances (rad^2)

## Known Issues

* Even when the `enable_mag_update` option is set to false (and the device acknowledges the setting with a positive ACK), the `quat_status` field is received as 3. This has not been fully debugged yet.

## Specifications

* Specifications can be found at [3DM-GX4-25](http://www.microstrain.com/inertial/3dm-gx4-25).

We provide YAML file to work with [Kalibr](https://github.com/ethz-asl/kalibr), which can be found in the `calib` folder.

## FAQs

1. What data rates can I use?
The driver supports up to 1000Hz for IMU data, and up to 500Hz for filter data. For high data rates, it is recommended that you use a baudrate of 921600.

2. The driver can't open my device, even though the path is specified correctly - what gives??
Make sure you have ownership of the device in `/dev`, or are part of the dialout group.
