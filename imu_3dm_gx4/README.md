# imu_3dm_gx4

![Picture of IMU](https://www.microstrain.com/sites/default/files/styles/larger__550x550_/public/gx4-25.jpg?itok=vB8GWQpI)

The `imu_3dm_gx4` package provides support for the [Lord Corporation](http://www.microstrain.com) Microstrain [3DM-GX4-25](http://www.microstrain.com/inertial/3dm-gx4-25) series IMU. The package employs the MIP packet format, so it could conceivably be adapted to support other versions of Microstrain products with relatively little effort. At present, the 15 and 45 series AHRS systems are not supported.

This package works on Ubuntu 12.04 and 14.04.

## Version History

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

The following additional options are present for leveraging the 3DM's onboard estimation filter:
* `enable_filter`: If true, the IMU estimation filter is enabled. Default is false.
* `filter_rate`: Filter rate to use, in Hz. Default is 100.
* `enable_mag_update`: If true, the IMU will use the magnetometer to correct the heading angle estimate. Default is false.
* `enable_accel_update`: If true, the IMU will use the accelerometer to correct
the roll/pitch angle estimates. Default is true.

**In order to launch the node** (streaming IMU data at 100Hz), execute:

`$ roslaunch imu_3dm_gx4 imu.launch`

### Note on rate parameters:

The `_rate` options command the requested frequency by calculating a 'decimation value', which is defined by the relation:

```
  frequency = base_frequency / decimation
```

Where the base frequency is 1kHz for the GX4. Since decimation values are integers, certain frequencies cannot be expressed by the above relation. 800Hz, for example, cannot be specified by an integer decimation rate. In these cases, you will receive whichever frequency is obtained by rounding decimation down to the nearest integer. Hence, requesting 800Hz will produce 1kHz.

## Output

On launch, the node will configure the IMU according to the parameters and then enable streaming node. At least three topics are published:

* `/imu_3dm_gx4/imu`: An instance of `sensor_msgs/Imu`. Orientation quaternion not provided in this message since is already part of the filter message.
* `/imu_3dm_gx4/magnetic_field`: An instance of `sensor_msgs/MagneticField`.
* `/imu_3dm_gx4/pressure`: An instance of `sensor_msgs/FluidPressure`.

All of the above topics are published with synchronized timestamps.

Additional topics will be published if `enable_filter` is true:

* `/imu_3dm_gx4/filter`: An instance of `imu_3dm_gx4/FilterOuput`. Custom message indicating all the onboard filter outputs.

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
