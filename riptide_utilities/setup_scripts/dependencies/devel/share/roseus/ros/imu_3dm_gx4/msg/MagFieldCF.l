;; Auto-generated. Do not edit!


(when (boundp 'imu_3dm_gx4::MagFieldCF)
  (if (not (find-package "IMU_3DM_GX4"))
    (make-package "IMU_3DM_GX4"))
  (shadow 'MagFieldCF (find-package "IMU_3DM_GX4")))
(unless (find-package "IMU_3DM_GX4::MAGFIELDCF")
  (make-package "IMU_3DM_GX4::MAGFIELDCF"))

(in-package "ROS")
;;//! \htmlinclude MagFieldCF.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass imu_3dm_gx4::MagFieldCF
  :super ros::object
  :slots (_header _components _magnitude _covariance ))

(defmethod imu_3dm_gx4::MagFieldCF
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:components __components) (instance geometry_msgs::Vector3 :init))
    ((:magnitude __magnitude) 0.0)
    ((:covariance __covariance) (make-array 9 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _components __components)
   (setq _magnitude (float __magnitude))
   (setq _covariance __covariance)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:components
   (&rest __components)
   (if (keywordp (car __components))
       (send* _components __components)
     (progn
       (if __components (setq _components (car __components)))
       _components)))
  (:magnitude
   (&optional __magnitude)
   (if __magnitude (setq _magnitude __magnitude)) _magnitude)
  (:covariance
   (&optional __covariance)
   (if __covariance (setq _covariance __covariance)) _covariance)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _components
    (send _components :serialization-length)
    ;; float64 _magnitude
    8
    ;; float64[9] _covariance
    (* 8    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _components
       (send _components :serialize s)
     ;; float64 _magnitude
       (sys::poke _magnitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[9] _covariance
     (dotimes (i 9)
       (sys::poke (elt _covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _components
     (send _components :deserialize buf ptr-) (incf ptr- (send _components :serialization-length))
   ;; float64 _magnitude
     (setq _magnitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[9] _covariance
   (dotimes (i (length _covariance))
     (setf (elt _covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get imu_3dm_gx4::MagFieldCF :md5sum-) "cd32a4ab24c73ea547c18eb391466a22")
(setf (get imu_3dm_gx4::MagFieldCF :datatype-) "imu_3dm_gx4/MagFieldCF")
(setf (get imu_3dm_gx4::MagFieldCF :definition-)
      "#Magnetic Field Message - Complimentary Filter
std_msgs/Header header
geometry_msgs/Vector3 components # X, Y, Z components [Gauss]
float64 magnitude # Total magnitude [Gauss]
float64[9] covariance

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



(provide :imu_3dm_gx4/MagFieldCF "cd32a4ab24c73ea547c18eb391466a22")


