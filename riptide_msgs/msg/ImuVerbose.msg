# Note on status flags:
# Status flags are implemented as bit-fields.
#  0 = invalid
#  1 = valid
#  2 = valid and referenced to magnetic north
#
# Note that covariance on orientation becomes invalid as pitch angle exceeds 70 # degrees. This will be indicated by the status flag.

#NOTE: any data field with the prefix "raw_" means it is direct output from the Estimation Kalman Filter

std_msgs/Header header

float64 mag_field_N # Gauss
float64 mag_field_E # Gauss
float64 mag_field_D # Gauss
float64 mag_field_magnitude # Gauss
float64 mag_inclination # Degrees
float64 mag_declination # Degrees
uint16 mag_status # 0 = invalid, 1 = valid

geometry_msgs/Vector3 raw_euler_rpy
geometry_msgs/Vector3 euler_rpy #The Euler Angles that we WANT to use
geometry_msgs/Vector3 gyro_bias

float64 heading #The correct heading (yaw), as updated by the internal magnetometer
float64 heading_uncertainty #1-sigma heading uncertainty
float64 heading_update_source #0 = none, 1 = internal magnetometer, 4 = external heading update
float64 heading_flags #0 = no update received within 2 sec, 1 = update received within 2 sec

geometry_msgs/Vector3 raw_linear_acceleration #This is actually ABSOLUTE acceleration, all values in [m/s^2]
geometry_msgs/Vector3 linear_acceleration #Corrected LINEAR acceleration, all values in [m/s^2]
geometry_msgs/Vector3 raw_angular_velocity #All values in [deg/s]
geometry_msgs/Vector3 angular_velocity #All values in [deg/s]
geometry_msgs/Vector3 angular_acceleration #All values in [deg/s^2]

float64 euler_rpy_status #0 = invalid, 1 = valid, 2 = values referenced to magnetic north
float64 angular_velocity_status #0 = invalid, 1 = valid
float64 linear_acceleration_status #0 = invalid, 1 = valid

float64 dt #Time since last message [s]
