# Note on status flags:
# Status flags are implemented as bit-fields.
#  0 = invalid
#  1 = valid
#  2 = valid and referenced to magnetic north
#
# Note that covariance on orientation becomes invalid as pitch angle exceeds 70 # degrees. This will be indicated by the status flag.

#NOTE: any data field with the prefix "raw_" means it is direct output from the Estimation Kalman Filter

#NOTE: All angles are in [deg], all distances are in [m]

std_msgs/Header header

geometry_msgs/Vector3 raw_euler_rpy #[deg]
geometry_msgs/Vector3 euler_rpy #The Euler Angles that we WANT to use, [deg]
geometry_msgs/Vector3 gyro_bias

float64 heading #OUR Calculated heading using filtered mag components, [deg]
float64 heading_update #The updated heading (yaw), produced by the internal magnetometer
float64 heading_update_uncertainty #1-sigma heading uncertainty
float64 heading_update_source #0 = none, 1 = internal magnetometer, 4 = external heading update
float64 heading_update_flags #0 = no update received within 2 sec, 1 = update received within 2 sec

geometry_msgs/Vector3 raw_linear_accel #This is actually ABSOLUTE acceleration, [m/s^2]
geometry_msgs/Vector3 linear_accel #Smoothed/corrected linear acceleration, [m/s^2]
geometry_msgs/Vector3 raw_ang_vel #[rad/s]
geometry_msgs/Vector3 ang_vel #Smoothed ang_v, [deg/s]
geometry_msgs/Vector3 ang_accel #[rad/s^2]

float64 euler_rpy_status #0 = invalid, 1 = valid, 2 = values referenced to magnetic north
float64 ang_vel_status #0 = invalid, 1 = valid
float64 linear_accel_status #0 = invalid, 1 = valid
