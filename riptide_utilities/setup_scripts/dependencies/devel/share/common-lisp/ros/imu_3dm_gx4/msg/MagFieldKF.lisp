; Auto-generated. Do not edit!


(cl:in-package imu_3dm_gx4-msg)


;//! \htmlinclude MagFieldKF.msg.html

(cl:defclass <MagFieldKF> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (mag_field_N
    :reader mag_field_N
    :initarg :mag_field_N
    :type cl:float
    :initform 0.0)
   (mag_field_E
    :reader mag_field_E
    :initarg :mag_field_E
    :type cl:float
    :initform 0.0)
   (mag_field_D
    :reader mag_field_D
    :initarg :mag_field_D
    :type cl:float
    :initform 0.0)
   (mag_field_magnitude
    :reader mag_field_magnitude
    :initarg :mag_field_magnitude
    :type cl:float
    :initform 0.0)
   (inclination
    :reader inclination
    :initarg :inclination
    :type cl:float
    :initform 0.0)
   (declination
    :reader declination
    :initarg :declination
    :type cl:float
    :initform 0.0)
   (mag_status
    :reader mag_status
    :initarg :mag_status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MagFieldKF (<MagFieldKF>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MagFieldKF>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MagFieldKF)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name imu_3dm_gx4-msg:<MagFieldKF> is deprecated: use imu_3dm_gx4-msg:MagFieldKF instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MagFieldKF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:header-val is deprecated.  Use imu_3dm_gx4-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mag_field_N-val :lambda-list '(m))
(cl:defmethod mag_field_N-val ((m <MagFieldKF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:mag_field_N-val is deprecated.  Use imu_3dm_gx4-msg:mag_field_N instead.")
  (mag_field_N m))

(cl:ensure-generic-function 'mag_field_E-val :lambda-list '(m))
(cl:defmethod mag_field_E-val ((m <MagFieldKF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:mag_field_E-val is deprecated.  Use imu_3dm_gx4-msg:mag_field_E instead.")
  (mag_field_E m))

(cl:ensure-generic-function 'mag_field_D-val :lambda-list '(m))
(cl:defmethod mag_field_D-val ((m <MagFieldKF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:mag_field_D-val is deprecated.  Use imu_3dm_gx4-msg:mag_field_D instead.")
  (mag_field_D m))

(cl:ensure-generic-function 'mag_field_magnitude-val :lambda-list '(m))
(cl:defmethod mag_field_magnitude-val ((m <MagFieldKF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:mag_field_magnitude-val is deprecated.  Use imu_3dm_gx4-msg:mag_field_magnitude instead.")
  (mag_field_magnitude m))

(cl:ensure-generic-function 'inclination-val :lambda-list '(m))
(cl:defmethod inclination-val ((m <MagFieldKF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:inclination-val is deprecated.  Use imu_3dm_gx4-msg:inclination instead.")
  (inclination m))

(cl:ensure-generic-function 'declination-val :lambda-list '(m))
(cl:defmethod declination-val ((m <MagFieldKF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:declination-val is deprecated.  Use imu_3dm_gx4-msg:declination instead.")
  (declination m))

(cl:ensure-generic-function 'mag_status-val :lambda-list '(m))
(cl:defmethod mag_status-val ((m <MagFieldKF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:mag_status-val is deprecated.  Use imu_3dm_gx4-msg:mag_status instead.")
  (mag_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MagFieldKF>) ostream)
  "Serializes a message object of type '<MagFieldKF>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mag_field_N))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mag_field_E))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mag_field_D))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mag_field_magnitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'inclination))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'declination))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mag_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mag_status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MagFieldKF>) istream)
  "Deserializes a message object of type '<MagFieldKF>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mag_field_N) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mag_field_E) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mag_field_D) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mag_field_magnitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'inclination) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'declination) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mag_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'mag_status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MagFieldKF>)))
  "Returns string type for a message object of type '<MagFieldKF>"
  "imu_3dm_gx4/MagFieldKF")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MagFieldKF)))
  "Returns string type for a message object of type 'MagFieldKF"
  "imu_3dm_gx4/MagFieldKF")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MagFieldKF>)))
  "Returns md5sum for a message object of type '<MagFieldKF>"
  "80d2cbcb915a78650f5f0199105cb92b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MagFieldKF)))
  "Returns md5sum for a message object of type 'MagFieldKF"
  "80d2cbcb915a78650f5f0199105cb92b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MagFieldKF>)))
  "Returns full string definition for message of type '<MagFieldKF>"
  (cl:format cl:nil "#Magnetic Field Message - Estimation Kalman Filter~%std_msgs/Header header~%float64 mag_field_N # Gauss~%float64 mag_field_E # Gauss~%float64 mag_field_D # Gauss~%float64 mag_field_magnitude # Gauss~%float64 inclination # Degrees~%float64 declination # Degrees~%uint16 mag_status # 0 = invalid, 1 = valid~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MagFieldKF)))
  "Returns full string definition for message of type 'MagFieldKF"
  (cl:format cl:nil "#Magnetic Field Message - Estimation Kalman Filter~%std_msgs/Header header~%float64 mag_field_N # Gauss~%float64 mag_field_E # Gauss~%float64 mag_field_D # Gauss~%float64 mag_field_magnitude # Gauss~%float64 inclination # Degrees~%float64 declination # Degrees~%uint16 mag_status # 0 = invalid, 1 = valid~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MagFieldKF>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
     8
     8
     8
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MagFieldKF>))
  "Converts a ROS message object to a list"
  (cl:list 'MagFieldKF
    (cl:cons ':header (header msg))
    (cl:cons ':mag_field_N (mag_field_N msg))
    (cl:cons ':mag_field_E (mag_field_E msg))
    (cl:cons ':mag_field_D (mag_field_D msg))
    (cl:cons ':mag_field_magnitude (mag_field_magnitude msg))
    (cl:cons ':inclination (inclination msg))
    (cl:cons ':declination (declination msg))
    (cl:cons ':mag_status (mag_status msg))
))
