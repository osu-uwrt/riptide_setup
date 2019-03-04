# Output from the 3DM-GX4 attitude estimation filter.

# Note on status flags:
# Status flags are implemented as bit-fields.
#  0 = invalid
#  1 = valid
#  2 = valid and referenced to magnetic north
#
# Note that covariance on orientation becomes invalid as pitch angle exceeds 70 # degrees. 
# This will be indicated by the status flag.

std_msgs/Header header

# Quaternion, and status
geometry_msgs/Quaternion quaternion
uint16 quaternion_status

# Gyroscope Euler angles roll, pitch, yaw, and status
geometry_msgs/Vector3 euler_rpy #Roll, Pitch, and Yaw in [radians]
float64 euler_rpy_status
float64[9] euler_angle_covariance
uint16 euler_angle_covariance_status

# Heading Update Data
float64 heading_update_alt #Heading in [radians]
float64 heading_update_LORD #Heading in [radians]
float64 heading_update_uncertainty #1-sigma heading uncertainty
float64 heading_update_source
float64 heading_update_flags #0 = no update received within 2 sec, 1 = update received within 2 sec

# Gyro bias, diagonal covariance, and status
geometry_msgs/Vector3 gyro_bias #Gyro bias from sensor frame [radians/sec]
uint16 gyro_bias_status
float64[9] gyro_bias_covariance
uint16 gyro_bias_covariance_status

# Linear accelerations along x,y,z axes, and status
geometry_msgs/Vector3 linear_acceleration #X, Y, and Z axes in [m/s^2]
float64 linear_acceleration_status

# Angular rates along x,y,z axes, and status
geometry_msgs/Vector3 angular_velocity #X, Y, and Z axes in [radians/s]
float64 angular_velocity_status

# Constants
uint16 STATUS_INVALID = 0
uint16 STATUS_VALID = 1
uint16 STATUS_VALID_REFERENCED = 2
