; Auto-generated. Do not edit!


(cl:in-package nortek_dvl-msg)


;//! \htmlinclude DvlStatus.msg.html

(cl:defclass <DvlStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (b1_vel_valid
    :reader b1_vel_valid
    :initarg :b1_vel_valid
    :type cl:boolean
    :initform cl:nil)
   (b2_vel_valid
    :reader b2_vel_valid
    :initarg :b2_vel_valid
    :type cl:boolean
    :initform cl:nil)
   (b3_vel_valid
    :reader b3_vel_valid
    :initarg :b3_vel_valid
    :type cl:boolean
    :initform cl:nil)
   (b4_vel_valid
    :reader b4_vel_valid
    :initarg :b4_vel_valid
    :type cl:boolean
    :initform cl:nil)
   (b1_dist_valid
    :reader b1_dist_valid
    :initarg :b1_dist_valid
    :type cl:boolean
    :initform cl:nil)
   (b2_dist_valid
    :reader b2_dist_valid
    :initarg :b2_dist_valid
    :type cl:boolean
    :initform cl:nil)
   (b3_dist_valid
    :reader b3_dist_valid
    :initarg :b3_dist_valid
    :type cl:boolean
    :initform cl:nil)
   (b4_dist_valid
    :reader b4_dist_valid
    :initarg :b4_dist_valid
    :type cl:boolean
    :initform cl:nil)
   (b1_fom_valid
    :reader b1_fom_valid
    :initarg :b1_fom_valid
    :type cl:boolean
    :initform cl:nil)
   (b2_fom_valid
    :reader b2_fom_valid
    :initarg :b2_fom_valid
    :type cl:boolean
    :initform cl:nil)
   (b3_fom_valid
    :reader b3_fom_valid
    :initarg :b3_fom_valid
    :type cl:boolean
    :initform cl:nil)
   (b4_fom_valid
    :reader b4_fom_valid
    :initarg :b4_fom_valid
    :type cl:boolean
    :initform cl:nil)
   (x_vel_valid
    :reader x_vel_valid
    :initarg :x_vel_valid
    :type cl:boolean
    :initform cl:nil)
   (y_vel_valid
    :reader y_vel_valid
    :initarg :y_vel_valid
    :type cl:boolean
    :initform cl:nil)
   (z1_vel_valid
    :reader z1_vel_valid
    :initarg :z1_vel_valid
    :type cl:boolean
    :initform cl:nil)
   (z2_vel_valid
    :reader z2_vel_valid
    :initarg :z2_vel_valid
    :type cl:boolean
    :initform cl:nil)
   (x_fom_valid
    :reader x_fom_valid
    :initarg :x_fom_valid
    :type cl:boolean
    :initform cl:nil)
   (y_fom_valid
    :reader y_fom_valid
    :initarg :y_fom_valid
    :type cl:boolean
    :initform cl:nil)
   (z1_fom_valid
    :reader z1_fom_valid
    :initarg :z1_fom_valid
    :type cl:boolean
    :initform cl:nil)
   (z2_fom_valid
    :reader z2_fom_valid
    :initarg :z2_fom_valid
    :type cl:boolean
    :initform cl:nil)
   (proc_cap
    :reader proc_cap
    :initarg :proc_cap
    :type cl:fixnum
    :initform 0)
   (wakeup_state
    :reader wakeup_state
    :initarg :wakeup_state
    :type cl:string
    :initform ""))
)

(cl:defclass DvlStatus (<DvlStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DvlStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DvlStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name nortek_dvl-msg:<DvlStatus> is deprecated: use nortek_dvl-msg:DvlStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:header-val is deprecated.  Use nortek_dvl-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'b1_vel_valid-val :lambda-list '(m))
(cl:defmethod b1_vel_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:b1_vel_valid-val is deprecated.  Use nortek_dvl-msg:b1_vel_valid instead.")
  (b1_vel_valid m))

(cl:ensure-generic-function 'b2_vel_valid-val :lambda-list '(m))
(cl:defmethod b2_vel_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:b2_vel_valid-val is deprecated.  Use nortek_dvl-msg:b2_vel_valid instead.")
  (b2_vel_valid m))

(cl:ensure-generic-function 'b3_vel_valid-val :lambda-list '(m))
(cl:defmethod b3_vel_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:b3_vel_valid-val is deprecated.  Use nortek_dvl-msg:b3_vel_valid instead.")
  (b3_vel_valid m))

(cl:ensure-generic-function 'b4_vel_valid-val :lambda-list '(m))
(cl:defmethod b4_vel_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:b4_vel_valid-val is deprecated.  Use nortek_dvl-msg:b4_vel_valid instead.")
  (b4_vel_valid m))

(cl:ensure-generic-function 'b1_dist_valid-val :lambda-list '(m))
(cl:defmethod b1_dist_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:b1_dist_valid-val is deprecated.  Use nortek_dvl-msg:b1_dist_valid instead.")
  (b1_dist_valid m))

(cl:ensure-generic-function 'b2_dist_valid-val :lambda-list '(m))
(cl:defmethod b2_dist_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:b2_dist_valid-val is deprecated.  Use nortek_dvl-msg:b2_dist_valid instead.")
  (b2_dist_valid m))

(cl:ensure-generic-function 'b3_dist_valid-val :lambda-list '(m))
(cl:defmethod b3_dist_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:b3_dist_valid-val is deprecated.  Use nortek_dvl-msg:b3_dist_valid instead.")
  (b3_dist_valid m))

(cl:ensure-generic-function 'b4_dist_valid-val :lambda-list '(m))
(cl:defmethod b4_dist_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:b4_dist_valid-val is deprecated.  Use nortek_dvl-msg:b4_dist_valid instead.")
  (b4_dist_valid m))

(cl:ensure-generic-function 'b1_fom_valid-val :lambda-list '(m))
(cl:defmethod b1_fom_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:b1_fom_valid-val is deprecated.  Use nortek_dvl-msg:b1_fom_valid instead.")
  (b1_fom_valid m))

(cl:ensure-generic-function 'b2_fom_valid-val :lambda-list '(m))
(cl:defmethod b2_fom_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:b2_fom_valid-val is deprecated.  Use nortek_dvl-msg:b2_fom_valid instead.")
  (b2_fom_valid m))

(cl:ensure-generic-function 'b3_fom_valid-val :lambda-list '(m))
(cl:defmethod b3_fom_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:b3_fom_valid-val is deprecated.  Use nortek_dvl-msg:b3_fom_valid instead.")
  (b3_fom_valid m))

(cl:ensure-generic-function 'b4_fom_valid-val :lambda-list '(m))
(cl:defmethod b4_fom_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:b4_fom_valid-val is deprecated.  Use nortek_dvl-msg:b4_fom_valid instead.")
  (b4_fom_valid m))

(cl:ensure-generic-function 'x_vel_valid-val :lambda-list '(m))
(cl:defmethod x_vel_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:x_vel_valid-val is deprecated.  Use nortek_dvl-msg:x_vel_valid instead.")
  (x_vel_valid m))

(cl:ensure-generic-function 'y_vel_valid-val :lambda-list '(m))
(cl:defmethod y_vel_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:y_vel_valid-val is deprecated.  Use nortek_dvl-msg:y_vel_valid instead.")
  (y_vel_valid m))

(cl:ensure-generic-function 'z1_vel_valid-val :lambda-list '(m))
(cl:defmethod z1_vel_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:z1_vel_valid-val is deprecated.  Use nortek_dvl-msg:z1_vel_valid instead.")
  (z1_vel_valid m))

(cl:ensure-generic-function 'z2_vel_valid-val :lambda-list '(m))
(cl:defmethod z2_vel_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:z2_vel_valid-val is deprecated.  Use nortek_dvl-msg:z2_vel_valid instead.")
  (z2_vel_valid m))

(cl:ensure-generic-function 'x_fom_valid-val :lambda-list '(m))
(cl:defmethod x_fom_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:x_fom_valid-val is deprecated.  Use nortek_dvl-msg:x_fom_valid instead.")
  (x_fom_valid m))

(cl:ensure-generic-function 'y_fom_valid-val :lambda-list '(m))
(cl:defmethod y_fom_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:y_fom_valid-val is deprecated.  Use nortek_dvl-msg:y_fom_valid instead.")
  (y_fom_valid m))

(cl:ensure-generic-function 'z1_fom_valid-val :lambda-list '(m))
(cl:defmethod z1_fom_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:z1_fom_valid-val is deprecated.  Use nortek_dvl-msg:z1_fom_valid instead.")
  (z1_fom_valid m))

(cl:ensure-generic-function 'z2_fom_valid-val :lambda-list '(m))
(cl:defmethod z2_fom_valid-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:z2_fom_valid-val is deprecated.  Use nortek_dvl-msg:z2_fom_valid instead.")
  (z2_fom_valid m))

(cl:ensure-generic-function 'proc_cap-val :lambda-list '(m))
(cl:defmethod proc_cap-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:proc_cap-val is deprecated.  Use nortek_dvl-msg:proc_cap instead.")
  (proc_cap m))

(cl:ensure-generic-function 'wakeup_state-val :lambda-list '(m))
(cl:defmethod wakeup_state-val ((m <DvlStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader nortek_dvl-msg:wakeup_state-val is deprecated.  Use nortek_dvl-msg:wakeup_state instead.")
  (wakeup_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DvlStatus>) ostream)
  "Serializes a message object of type '<DvlStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b1_vel_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b2_vel_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b3_vel_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b4_vel_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b1_dist_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b2_dist_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b3_dist_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b4_dist_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b1_fom_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b2_fom_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b3_fom_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'b4_fom_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'x_vel_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'y_vel_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'z1_vel_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'z2_vel_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'x_fom_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'y_fom_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'z1_fom_valid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'z2_fom_valid) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'proc_cap)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'wakeup_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'wakeup_state))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DvlStatus>) istream)
  "Deserializes a message object of type '<DvlStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'b1_vel_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'b2_vel_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'b3_vel_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'b4_vel_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'b1_dist_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'b2_dist_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'b3_dist_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'b4_dist_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'b1_fom_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'b2_fom_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'b3_fom_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'b4_fom_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'x_vel_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'y_vel_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'z1_vel_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'z2_vel_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'x_fom_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'y_fom_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'z1_fom_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'z2_fom_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'proc_cap) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'wakeup_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'wakeup_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DvlStatus>)))
  "Returns string type for a message object of type '<DvlStatus>"
  "nortek_dvl/DvlStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DvlStatus)))
  "Returns string type for a message object of type 'DvlStatus"
  "nortek_dvl/DvlStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DvlStatus>)))
  "Returns md5sum for a message object of type '<DvlStatus>"
  "e1789433d0fd4a0c672172f8d32e464d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DvlStatus)))
  "Returns md5sum for a message object of type 'DvlStatus"
  "e1789433d0fd4a0c672172f8d32e464d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DvlStatus>)))
  "Returns full string definition for message of type '<DvlStatus>"
  (cl:format cl:nil "Header header~%~%bool b1_vel_valid~%bool b2_vel_valid~%bool b3_vel_valid~%bool b4_vel_valid~%~%bool b1_dist_valid~%bool b2_dist_valid~%bool b3_dist_valid~%bool b4_dist_valid~%~%bool b1_fom_valid~%bool b2_fom_valid~%bool b3_fom_valid~%bool b4_fom_valid~%~%bool x_vel_valid~%bool y_vel_valid~%bool z1_vel_valid~%bool z2_vel_valid~%~%bool x_fom_valid~%bool y_fom_valid~%bool z1_fom_valid~%bool z2_fom_valid~%~%int8 proc_cap~%string wakeup_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DvlStatus)))
  "Returns full string definition for message of type 'DvlStatus"
  (cl:format cl:nil "Header header~%~%bool b1_vel_valid~%bool b2_vel_valid~%bool b3_vel_valid~%bool b4_vel_valid~%~%bool b1_dist_valid~%bool b2_dist_valid~%bool b3_dist_valid~%bool b4_dist_valid~%~%bool b1_fom_valid~%bool b2_fom_valid~%bool b3_fom_valid~%bool b4_fom_valid~%~%bool x_vel_valid~%bool y_vel_valid~%bool z1_vel_valid~%bool z2_vel_valid~%~%bool x_fom_valid~%bool y_fom_valid~%bool z1_fom_valid~%bool z2_fom_valid~%~%int8 proc_cap~%string wakeup_state~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DvlStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     4 (cl:length (cl:slot-value msg 'wakeup_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DvlStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'DvlStatus
    (cl:cons ':header (header msg))
    (cl:cons ':b1_vel_valid (b1_vel_valid msg))
    (cl:cons ':b2_vel_valid (b2_vel_valid msg))
    (cl:cons ':b3_vel_valid (b3_vel_valid msg))
    (cl:cons ':b4_vel_valid (b4_vel_valid msg))
    (cl:cons ':b1_dist_valid (b1_dist_valid msg))
    (cl:cons ':b2_dist_valid (b2_dist_valid msg))
    (cl:cons ':b3_dist_valid (b3_dist_valid msg))
    (cl:cons ':b4_dist_valid (b4_dist_valid msg))
    (cl:cons ':b1_fom_valid (b1_fom_valid msg))
    (cl:cons ':b2_fom_valid (b2_fom_valid msg))
    (cl:cons ':b3_fom_valid (b3_fom_valid msg))
    (cl:cons ':b4_fom_valid (b4_fom_valid msg))
    (cl:cons ':x_vel_valid (x_vel_valid msg))
    (cl:cons ':y_vel_valid (y_vel_valid msg))
    (cl:cons ':z1_vel_valid (z1_vel_valid msg))
    (cl:cons ':z2_vel_valid (z2_vel_valid msg))
    (cl:cons ':x_fom_valid (x_fom_valid msg))
    (cl:cons ':y_fom_valid (y_fom_valid msg))
    (cl:cons ':z1_fom_valid (z1_fom_valid msg))
    (cl:cons ':z2_fom_valid (z2_fom_valid msg))
    (cl:cons ':proc_cap (proc_cap msg))
    (cl:cons ':wakeup_state (wakeup_state msg))
))
