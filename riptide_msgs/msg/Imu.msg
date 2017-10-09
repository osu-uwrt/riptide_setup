std_msgs/Header header
geometry_msgs/Vector3 raw_euler_rpy
geometry_msgs/Vector3 euler_rpy
geometry_msgs/Vector3 linear_acceleration
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 angular_acceleration

float64 euler_rpy_status #0 = invalid, 1 = valid, 2 = values referenced to magnetic north
float64 angular_velocity_status #0 = invalid, 1 = valid
float64 linear_acceleration_status #0 = invalid, 1 = valid

float64 dt
geometry_msgs/Vector3 cumulative_drift #Cumulative (absolute value of) drift since running the IMU
geometry_msgs/Vector3 local_drift #Amount of drift upon immediately noticing "zero" angular velocity
geometry_msgs/Vector3 local_drift_rate #Drift rate upon immediately noticing "zero" angular velocity
