#Magnetic Field Message - Estimation Kalman Filter
std_msgs/Header header
float64 mag_field_N # Gauss
float64 mag_field_E # Gauss
float64 mag_field_D # Gauss
float64 mag_field_magnitude # Gauss
float64 inclination # Degrees
float64 declination # Degrees
uint16 mag_status # 0 = invalid, 1 = valid
