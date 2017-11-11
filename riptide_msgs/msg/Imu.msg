# Note on status flags:
# Status flags are implemented as bit-fields.
#  0 = invalid
#  1 = valid
#  2 = valid and referenced to magnetic north
#
# Note that covariance on orientation becomes invalid as pitch angle exceeds 70 # degrees. This will be indicated by the status flag.

std_msgs/Header header
geometry_msgs/Vector3 raw_euler_rpy #Direct output from Estimation Filter
geometry_msgs/Vector3 euler_rpy #The Euler Angles that we WANT to use
geometry_msgs/Vector3 gyro_bias

float64 heading #The correct heading (yaw), as updated by the internal magnetometer
float64 heading_uncertainty #1-sigma heading uncertainty
float64 heading_update_source #0 = none, 1 = internal magnetometer, 4 = external heading update
float64 heading_flags #0 = no update received within 2 sec, 1 = update received within 2 sec

geometry_msgs/Vector3 linear_acceleration
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 angular_acceleration

float64 euler_rpy_status #0 = invalid, 1 = valid, 2 = values referenced to magnetic north
float64 angular_velocity_status #0 = invalid, 1 = valid
float64 linear_acceleration_status #0 = invalid, 1 = valid

float64 dt #Time since last message
geometry_msgs/Vector3 cumulative_drift #Cumulative (absolute value of) drift since running the IMU
geometry_msgs/Vector3 local_drift #Amount of drift upon immediately noticing "zero" angular velocity
geometry_msgs/Vector3 local_drift_rate #Drift rate upon immediately noticing "zero" angular velocity
