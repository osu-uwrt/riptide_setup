;; Auto-generated. Do not edit!


(when (boundp 'imu_3dm_gx4::FilterOutput)
  (if (not (find-package "IMU_3DM_GX4"))
    (make-package "IMU_3DM_GX4"))
  (shadow 'FilterOutput (find-package "IMU_3DM_GX4")))
(unless (find-package "IMU_3DM_GX4::FILTEROUTPUT")
  (make-package "IMU_3DM_GX4::FILTEROUTPUT"))

(in-package "ROS")
;;//! \htmlinclude FilterOutput.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*STATUS_INVALID*" (find-package "IMU_3DM_GX4::FILTEROUTPUT"))
(shadow '*STATUS_INVALID* (find-package "IMU_3DM_GX4::FILTEROUTPUT"))
(defconstant imu_3dm_gx4::FilterOutput::*STATUS_INVALID* 0)
(intern "*STATUS_VALID*" (find-package "IMU_3DM_GX4::FILTEROUTPUT"))
(shadow '*STATUS_VALID* (find-package "IMU_3DM_GX4::FILTEROUTPUT"))
(defconstant imu_3dm_gx4::FilterOutput::*STATUS_VALID* 1)
(intern "*STATUS_VALID_REFERENCED*" (find-package "IMU_3DM_GX4::FILTEROUTPUT"))
(shadow '*STATUS_VALID_REFERENCED* (find-package "IMU_3DM_GX4::FILTEROUTPUT"))
(defconstant imu_3dm_gx4::FilterOutput::*STATUS_VALID_REFERENCED* 2)
(defclass imu_3dm_gx4::FilterOutput
  :super ros::object
  :slots (_header _quaternion _quaternion_status _euler_rpy _euler_rpy_status _euler_angle_covariance _euler_angle_covariance_status _heading_update_alt _heading_update_LORD _heading_update_uncertainty _heading_update_source _heading_update_flags _gyro_bias _gyro_bias_status _gyro_bias_covariance _gyro_bias_covariance_status _linear_acceleration _linear_acceleration_status _angular_velocity _angular_velocity_status ))

(defmethod imu_3dm_gx4::FilterOutput
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:quaternion __quaternion) (instance geometry_msgs::Quaternion :init))
    ((:quaternion_status __quaternion_status) 0)
    ((:euler_rpy __euler_rpy) (instance geometry_msgs::Vector3 :init))
    ((:euler_rpy_status __euler_rpy_status) 0.0)
    ((:euler_angle_covariance __euler_angle_covariance) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:euler_angle_covariance_status __euler_angle_covariance_status) 0)
    ((:heading_update_alt __heading_update_alt) 0.0)
    ((:heading_update_LORD __heading_update_LORD) 0.0)
    ((:heading_update_uncertainty __heading_update_uncertainty) 0.0)
    ((:heading_update_source __heading_update_source) 0.0)
    ((:heading_update_flags __heading_update_flags) 0.0)
    ((:gyro_bias __gyro_bias) (instance geometry_msgs::Vector3 :init))
    ((:gyro_bias_status __gyro_bias_status) 0)
    ((:gyro_bias_covariance __gyro_bias_covariance) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:gyro_bias_covariance_status __gyro_bias_covariance_status) 0)
    ((:linear_acceleration __linear_acceleration) (instance geometry_msgs::Vector3 :init))
    ((:linear_acceleration_status __linear_acceleration_status) 0.0)
    ((:angular_velocity __angular_velocity) (instance geometry_msgs::Vector3 :init))
    ((:angular_velocity_status __angular_velocity_status) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _quaternion __quaternion)
   (setq _quaternion_status (round __quaternion_status))
   (setq _euler_rpy __euler_rpy)
   (setq _euler_rpy_status (float __euler_rpy_status))
   (setq _euler_angle_covariance __euler_angle_covariance)
   (setq _euler_angle_covariance_status (round __euler_angle_covariance_status))
   (setq _heading_update_alt (float __heading_update_alt))
   (setq _heading_update_LORD (float __heading_update_LORD))
   (setq _heading_update_uncertainty (float __heading_update_uncertainty))
   (setq _heading_update_source (float __heading_update_source))
   (setq _heading_update_flags (float __heading_update_flags))
   (setq _gyro_bias __gyro_bias)
   (setq _gyro_bias_status (round __gyro_bias_status))
   (setq _gyro_bias_covariance __gyro_bias_covariance)
   (setq _gyro_bias_covariance_status (round __gyro_bias_covariance_status))
   (setq _linear_acceleration __linear_acceleration)
   (setq _linear_acceleration_status (float __linear_acceleration_status))
   (setq _angular_velocity __angular_velocity)
   (setq _angular_velocity_status (float __angular_velocity_status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:quaternion
   (&rest __quaternion)
   (if (keywordp (car __quaternion))
       (send* _quaternion __quaternion)
     (progn
       (if __quaternion (setq _quaternion (car __quaternion)))
       _quaternion)))
  (:quaternion_status
   (&optional __quaternion_status)
   (if __quaternion_status (setq _quaternion_status __quaternion_status)) _quaternion_status)
  (:euler_rpy
   (&rest __euler_rpy)
   (if (keywordp (car __euler_rpy))
       (send* _euler_rpy __euler_rpy)
     (progn
       (if __euler_rpy (setq _euler_rpy (car __euler_rpy)))
       _euler_rpy)))
  (:euler_rpy_status
   (&optional __euler_rpy_status)
   (if __euler_rpy_status (setq _euler_rpy_status __euler_rpy_status)) _euler_rpy_status)
  (:euler_angle_covariance
   (&optional __euler_angle_covariance)
   (if __euler_angle_covariance (setq _euler_angle_covariance __euler_angle_covariance)) _euler_angle_covariance)
  (:euler_angle_covariance_status
   (&optional __euler_angle_covariance_status)
   (if __euler_angle_covariance_status (setq _euler_angle_covariance_status __euler_angle_covariance_status)) _euler_angle_covariance_status)
  (:heading_update_alt
   (&optional __heading_update_alt)
   (if __heading_update_alt (setq _heading_update_alt __heading_update_alt)) _heading_update_alt)
  (:heading_update_LORD
   (&optional __heading_update_LORD)
   (if __heading_update_LORD (setq _heading_update_LORD __heading_update_LORD)) _heading_update_LORD)
  (:heading_update_uncertainty
   (&optional __heading_update_uncertainty)
   (if __heading_update_uncertainty (setq _heading_update_uncertainty __heading_update_uncertainty)) _heading_update_uncertainty)
  (:heading_update_source
   (&optional __heading_update_source)
   (if __heading_update_source (setq _heading_update_source __heading_update_source)) _heading_update_source)
  (:heading_update_flags
   (&optional __heading_update_flags)
   (if __heading_update_flags (setq _heading_update_flags __heading_update_flags)) _heading_update_flags)
  (:gyro_bias
   (&rest __gyro_bias)
   (if (keywordp (car __gyro_bias))
       (send* _gyro_bias __gyro_bias)
     (progn
       (if __gyro_bias (setq _gyro_bias (car __gyro_bias)))
       _gyro_bias)))
  (:gyro_bias_status
   (&optional __gyro_bias_status)
   (if __gyro_bias_status (setq _gyro_bias_status __gyro_bias_status)) _gyro_bias_status)
  (:gyro_bias_covariance
   (&optional __gyro_bias_covariance)
   (if __gyro_bias_covariance (setq _gyro_bias_covariance __gyro_bias_covariance)) _gyro_bias_covariance)
  (:gyro_bias_covariance_status
   (&optional __gyro_bias_covariance_status)
   (if __gyro_bias_covariance_status (setq _gyro_bias_covariance_status __gyro_bias_covariance_status)) _gyro_bias_covariance_status)
  (:linear_acceleration
   (&rest __linear_acceleration)
   (if (keywordp (car __linear_acceleration))
       (send* _linear_acceleration __linear_acceleration)
     (progn
       (if __linear_acceleration (setq _linear_acceleration (car __linear_acceleration)))
       _linear_acceleration)))
  (:linear_acceleration_status
   (&optional __linear_acceleration_status)
   (if __linear_acceleration_status (setq _linear_acceleration_status __linear_acceleration_status)) _linear_acceleration_status)
  (:angular_velocity
   (&rest __angular_velocity)
   (if (keywordp (car __angular_velocity))
       (send* _angular_velocity __angular_velocity)
     (progn
       (if __angular_velocity (setq _angular_velocity (car __angular_velocity)))
       _angular_velocity)))
  (:angular_velocity_status
   (&optional __angular_velocity_status)
   (if __angular_velocity_status (setq _angular_velocity_status __angular_velocity_status)) _angular_velocity_status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Quaternion _quaternion
    (send _quaternion :serialization-length)
    ;; uint16 _quaternion_status
    2
    ;; geometry_msgs/Vector3 _euler_rpy
    (send _euler_rpy :serialization-length)
    ;; float64 _euler_rpy_status
    8
    ;; float64[9] _euler_angle_covariance
    (* 8    9)
    ;; uint16 _euler_angle_covariance_status
    2
    ;; float64 _heading_update_alt
    8
    ;; float64 _heading_update_LORD
    8
    ;; float64 _heading_update_uncertainty
    8
    ;; float64 _heading_update_source
    8
    ;; float64 _heading_update_flags
    8
    ;; geometry_msgs/Vector3 _gyro_bias
    (send _gyro_bias :serialization-length)
    ;; uint16 _gyro_bias_status
    2
    ;; float64[9] _gyro_bias_covariance
    (* 8    9)
    ;; uint16 _gyro_bias_covariance_status
    2
    ;; geometry_msgs/Vector3 _linear_acceleration
    (send _linear_acceleration :serialization-length)
    ;; float64 _linear_acceleration_status
    8
    ;; geometry_msgs/Vector3 _angular_velocity
    (send _angular_velocity :serialization-length)
    ;; float64 _angular_velocity_status
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Quaternion _quaternion
       (send _quaternion :serialize s)
     ;; uint16 _quaternion_status
       (write-word _quaternion_status s)
     ;; geometry_msgs/Vector3 _euler_rpy
       (send _euler_rpy :serialize s)
     ;; float64 _euler_rpy_status
       (sys::poke _euler_rpy_status (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[9] _euler_angle_covariance
     (dotimes (i 9)
       (sys::poke (elt _euler_angle_covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint16 _euler_angle_covariance_status
       (write-word _euler_angle_covariance_status s)
     ;; float64 _heading_update_alt
       (sys::poke _heading_update_alt (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading_update_LORD
       (sys::poke _heading_update_LORD (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading_update_uncertainty
       (sys::poke _heading_update_uncertainty (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading_update_source
       (sys::poke _heading_update_source (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading_update_flags
       (sys::poke _heading_update_flags (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Vector3 _gyro_bias
       (send _gyro_bias :serialize s)
     ;; uint16 _gyro_bias_status
       (write-word _gyro_bias_status s)
     ;; float64[9] _gyro_bias_covariance
     (dotimes (i 9)
       (sys::poke (elt _gyro_bias_covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint16 _gyro_bias_covariance_status
       (write-word _gyro_bias_covariance_status s)
     ;; geometry_msgs/Vector3 _linear_acceleration
       (send _linear_acceleration :serialize s)
     ;; float64 _linear_acceleration_status
       (sys::poke _linear_acceleration_status (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Vector3 _angular_velocity
       (send _angular_velocity :serialize s)
     ;; float64 _angular_velocity_status
       (sys::poke _angular_velocity_status (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Quaternion _quaternion
     (send _quaternion :deserialize buf ptr-) (incf ptr- (send _quaternion :serialization-length))
   ;; uint16 _quaternion_status
     (setq _quaternion_status (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; geometry_msgs/Vector3 _euler_rpy
     (send _euler_rpy :deserialize buf ptr-) (incf ptr- (send _euler_rpy :serialization-length))
   ;; float64 _euler_rpy_status
     (setq _euler_rpy_status (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[9] _euler_angle_covariance
   (dotimes (i (length _euler_angle_covariance))
     (setf (elt _euler_angle_covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; uint16 _euler_angle_covariance_status
     (setq _euler_angle_covariance_status (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _heading_update_alt
     (setq _heading_update_alt (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading_update_LORD
     (setq _heading_update_LORD (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading_update_uncertainty
     (setq _heading_update_uncertainty (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading_update_source
     (setq _heading_update_source (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading_update_flags
     (setq _heading_update_flags (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Vector3 _gyro_bias
     (send _gyro_bias :deserialize buf ptr-) (incf ptr- (send _gyro_bias :serialization-length))
   ;; uint16 _gyro_bias_status
     (setq _gyro_bias_status (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64[9] _gyro_bias_covariance
   (dotimes (i (length _gyro_bias_covariance))
     (setf (elt _gyro_bias_covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; uint16 _gyro_bias_covariance_status
     (setq _gyro_bias_covariance_status (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; geometry_msgs/Vector3 _linear_acceleration
     (send _linear_acceleration :deserialize buf ptr-) (incf ptr- (send _linear_acceleration :serialization-length))
   ;; float64 _linear_acceleration_status
     (setq _linear_acceleration_status (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Vector3 _angular_velocity
     (send _angular_velocity :deserialize buf ptr-) (incf ptr- (send _angular_velocity :serialization-length))
   ;; float64 _angular_velocity_status
     (setq _angular_velocity_status (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get imu_3dm_gx4::FilterOutput :md5sum-) "9ebeb70bfcf2c275d32570ff4a241bad")
(setf (get imu_3dm_gx4::FilterOutput :datatype-) "imu_3dm_gx4/FilterOutput")
(setf (get imu_3dm_gx4::FilterOutput :definition-)
      "# Output from the 3DM-GX4 attitude estimation filter.

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

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :imu_3dm_gx4/FilterOutput "9ebeb70bfcf2c275d32570ff4a241bad")


