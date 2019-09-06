;; Auto-generated. Do not edit!


(when (boundp 'imu_3dm_gx4::MagFieldKF)
  (if (not (find-package "IMU_3DM_GX4"))
    (make-package "IMU_3DM_GX4"))
  (shadow 'MagFieldKF (find-package "IMU_3DM_GX4")))
(unless (find-package "IMU_3DM_GX4::MAGFIELDKF")
  (make-package "IMU_3DM_GX4::MAGFIELDKF"))

(in-package "ROS")
;;//! \htmlinclude MagFieldKF.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass imu_3dm_gx4::MagFieldKF
  :super ros::object
  :slots (_header _mag_field_N _mag_field_E _mag_field_D _mag_field_magnitude _inclination _declination _mag_status ))

(defmethod imu_3dm_gx4::MagFieldKF
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:mag_field_N __mag_field_N) 0.0)
    ((:mag_field_E __mag_field_E) 0.0)
    ((:mag_field_D __mag_field_D) 0.0)
    ((:mag_field_magnitude __mag_field_magnitude) 0.0)
    ((:inclination __inclination) 0.0)
    ((:declination __declination) 0.0)
    ((:mag_status __mag_status) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _mag_field_N (float __mag_field_N))
   (setq _mag_field_E (float __mag_field_E))
   (setq _mag_field_D (float __mag_field_D))
   (setq _mag_field_magnitude (float __mag_field_magnitude))
   (setq _inclination (float __inclination))
   (setq _declination (float __declination))
   (setq _mag_status (round __mag_status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:mag_field_N
   (&optional __mag_field_N)
   (if __mag_field_N (setq _mag_field_N __mag_field_N)) _mag_field_N)
  (:mag_field_E
   (&optional __mag_field_E)
   (if __mag_field_E (setq _mag_field_E __mag_field_E)) _mag_field_E)
  (:mag_field_D
   (&optional __mag_field_D)
   (if __mag_field_D (setq _mag_field_D __mag_field_D)) _mag_field_D)
  (:mag_field_magnitude
   (&optional __mag_field_magnitude)
   (if __mag_field_magnitude (setq _mag_field_magnitude __mag_field_magnitude)) _mag_field_magnitude)
  (:inclination
   (&optional __inclination)
   (if __inclination (setq _inclination __inclination)) _inclination)
  (:declination
   (&optional __declination)
   (if __declination (setq _declination __declination)) _declination)
  (:mag_status
   (&optional __mag_status)
   (if __mag_status (setq _mag_status __mag_status)) _mag_status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _mag_field_N
    8
    ;; float64 _mag_field_E
    8
    ;; float64 _mag_field_D
    8
    ;; float64 _mag_field_magnitude
    8
    ;; float64 _inclination
    8
    ;; float64 _declination
    8
    ;; uint16 _mag_status
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _mag_field_N
       (sys::poke _mag_field_N (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mag_field_E
       (sys::poke _mag_field_E (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mag_field_D
       (sys::poke _mag_field_D (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mag_field_magnitude
       (sys::poke _mag_field_magnitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _inclination
       (sys::poke _inclination (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _declination
       (sys::poke _declination (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint16 _mag_status
       (write-word _mag_status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _mag_field_N
     (setq _mag_field_N (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mag_field_E
     (setq _mag_field_E (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mag_field_D
     (setq _mag_field_D (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mag_field_magnitude
     (setq _mag_field_magnitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _inclination
     (setq _inclination (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _declination
     (setq _declination (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint16 _mag_status
     (setq _mag_status (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get imu_3dm_gx4::MagFieldKF :md5sum-) "80d2cbcb915a78650f5f0199105cb92b")
(setf (get imu_3dm_gx4::MagFieldKF :datatype-) "imu_3dm_gx4/MagFieldKF")
(setf (get imu_3dm_gx4::MagFieldKF :definition-)
      "#Magnetic Field Message - Estimation Kalman Filter
std_msgs/Header header
float64 mag_field_N # Gauss
float64 mag_field_E # Gauss
float64 mag_field_D # Gauss
float64 mag_field_magnitude # Gauss
float64 inclination # Degrees
float64 declination # Degrees
uint16 mag_status # 0 = invalid, 1 = valid

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

")



(provide :imu_3dm_gx4/MagFieldKF "80d2cbcb915a78650f5f0199105cb92b")


