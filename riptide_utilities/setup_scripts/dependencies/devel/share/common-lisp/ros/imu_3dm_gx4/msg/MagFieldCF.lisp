; Auto-generated. Do not edit!


(cl:in-package imu_3dm_gx4-msg)


;//! \htmlinclude MagFieldCF.msg.html

(cl:defclass <MagFieldCF> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (components
    :reader components
    :initarg :components
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (magnitude
    :reader magnitude
    :initarg :magnitude
    :type cl:float
    :initform 0.0)
   (covariance
    :reader covariance
    :initarg :covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass MagFieldCF (<MagFieldCF>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MagFieldCF>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MagFieldCF)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name imu_3dm_gx4-msg:<MagFieldCF> is deprecated: use imu_3dm_gx4-msg:MagFieldCF instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MagFieldCF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:header-val is deprecated.  Use imu_3dm_gx4-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'components-val :lambda-list '(m))
(cl:defmethod components-val ((m <MagFieldCF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:components-val is deprecated.  Use imu_3dm_gx4-msg:components instead.")
  (components m))

(cl:ensure-generic-function 'magnitude-val :lambda-list '(m))
(cl:defmethod magnitude-val ((m <MagFieldCF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:magnitude-val is deprecated.  Use imu_3dm_gx4-msg:magnitude instead.")
  (magnitude m))

(cl:ensure-generic-function 'covariance-val :lambda-list '(m))
(cl:defmethod covariance-val ((m <MagFieldCF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:covariance-val is deprecated.  Use imu_3dm_gx4-msg:covariance instead.")
  (covariance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MagFieldCF>) ostream)
  "Serializes a message object of type '<MagFieldCF>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'components) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'magnitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'covariance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MagFieldCF>) istream)
  "Deserializes a message object of type '<MagFieldCF>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'components) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'magnitude) (roslisp-utils:decode-double-float-bits bits)))
  (cl:setf (cl:slot-value msg 'covariance) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'covariance)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MagFieldCF>)))
  "Returns string type for a message object of type '<MagFieldCF>"
  "imu_3dm_gx4/MagFieldCF")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MagFieldCF)))
  "Returns string type for a message object of type 'MagFieldCF"
  "imu_3dm_gx4/MagFieldCF")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MagFieldCF>)))
  "Returns md5sum for a message object of type '<MagFieldCF>"
  "cd32a4ab24c73ea547c18eb391466a22")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MagFieldCF)))
  "Returns md5sum for a message object of type 'MagFieldCF"
  "cd32a4ab24c73ea547c18eb391466a22")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MagFieldCF>)))
  "Returns full string definition for message of type '<MagFieldCF>"
  (cl:format cl:nil "#Magnetic Field Message - Complimentary Filter~%std_msgs/Header header~%geometry_msgs/Vector3 components # X, Y, Z components [Gauss]~%float64 magnitude # Total magnitude [Gauss]~%float64[9] covariance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MagFieldCF)))
  "Returns full string definition for message of type 'MagFieldCF"
  (cl:format cl:nil "#Magnetic Field Message - Complimentary Filter~%std_msgs/Header header~%geometry_msgs/Vector3 components # X, Y, Z components [Gauss]~%float64 magnitude # Total magnitude [Gauss]~%float64[9] covariance~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MagFieldCF>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'components))
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MagFieldCF>))
  "Converts a ROS message object to a list"
  (cl:list 'MagFieldCF
    (cl:cons ':header (header msg))
    (cl:cons ':components (components msg))
    (cl:cons ':magnitude (magnitude msg))
    (cl:cons ':covariance (covariance msg))
))
