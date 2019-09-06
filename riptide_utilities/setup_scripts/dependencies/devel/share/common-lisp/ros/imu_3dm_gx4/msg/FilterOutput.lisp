; Auto-generated. Do not edit!


(cl:in-package imu_3dm_gx4-msg)


;//! \htmlinclude FilterOutput.msg.html

(cl:defclass <FilterOutput> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (quaternion
    :reader quaternion
    :initarg :quaternion
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (quaternion_status
    :reader quaternion_status
    :initarg :quaternion_status
    :type cl:fixnum
    :initform 0)
   (euler_rpy
    :reader euler_rpy
    :initarg :euler_rpy
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (euler_rpy_status
    :reader euler_rpy_status
    :initarg :euler_rpy_status
    :type cl:float
    :initform 0.0)
   (euler_angle_covariance
    :reader euler_angle_covariance
    :initarg :euler_angle_covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0))
   (euler_angle_covariance_status
    :reader euler_angle_covariance_status
    :initarg :euler_angle_covariance_status
    :type cl:fixnum
    :initform 0)
   (heading_update_alt
    :reader heading_update_alt
    :initarg :heading_update_alt
    :type cl:float
    :initform 0.0)
   (heading_update_LORD
    :reader heading_update_LORD
    :initarg :heading_update_LORD
    :type cl:float
    :initform 0.0)
   (heading_update_uncertainty
    :reader heading_update_uncertainty
    :initarg :heading_update_uncertainty
    :type cl:float
    :initform 0.0)
   (heading_update_source
    :reader heading_update_source
    :initarg :heading_update_source
    :type cl:float
    :initform 0.0)
   (heading_update_flags
    :reader heading_update_flags
    :initarg :heading_update_flags
    :type cl:float
    :initform 0.0)
   (gyro_bias
    :reader gyro_bias
    :initarg :gyro_bias
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (gyro_bias_status
    :reader gyro_bias_status
    :initarg :gyro_bias_status
    :type cl:fixnum
    :initform 0)
   (gyro_bias_covariance
    :reader gyro_bias_covariance
    :initarg :gyro_bias_covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0))
   (gyro_bias_covariance_status
    :reader gyro_bias_covariance_status
    :initarg :gyro_bias_covariance_status
    :type cl:fixnum
    :initform 0)
   (linear_acceleration
    :reader linear_acceleration
    :initarg :linear_acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (linear_acceleration_status
    :reader linear_acceleration_status
    :initarg :linear_acceleration_status
    :type cl:float
    :initform 0.0)
   (angular_velocity
    :reader angular_velocity
    :initarg :angular_velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (angular_velocity_status
    :reader angular_velocity_status
    :initarg :angular_velocity_status
    :type cl:float
    :initform 0.0))
)

(cl:defclass FilterOutput (<FilterOutput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FilterOutput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FilterOutput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name imu_3dm_gx4-msg:<FilterOutput> is deprecated: use imu_3dm_gx4-msg:FilterOutput instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:header-val is deprecated.  Use imu_3dm_gx4-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'quaternion-val :lambda-list '(m))
(cl:defmethod quaternion-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:quaternion-val is deprecated.  Use imu_3dm_gx4-msg:quaternion instead.")
  (quaternion m))

(cl:ensure-generic-function 'quaternion_status-val :lambda-list '(m))
(cl:defmethod quaternion_status-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:quaternion_status-val is deprecated.  Use imu_3dm_gx4-msg:quaternion_status instead.")
  (quaternion_status m))

(cl:ensure-generic-function 'euler_rpy-val :lambda-list '(m))
(cl:defmethod euler_rpy-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:euler_rpy-val is deprecated.  Use imu_3dm_gx4-msg:euler_rpy instead.")
  (euler_rpy m))

(cl:ensure-generic-function 'euler_rpy_status-val :lambda-list '(m))
(cl:defmethod euler_rpy_status-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:euler_rpy_status-val is deprecated.  Use imu_3dm_gx4-msg:euler_rpy_status instead.")
  (euler_rpy_status m))

(cl:ensure-generic-function 'euler_angle_covariance-val :lambda-list '(m))
(cl:defmethod euler_angle_covariance-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:euler_angle_covariance-val is deprecated.  Use imu_3dm_gx4-msg:euler_angle_covariance instead.")
  (euler_angle_covariance m))

(cl:ensure-generic-function 'euler_angle_covariance_status-val :lambda-list '(m))
(cl:defmethod euler_angle_covariance_status-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:euler_angle_covariance_status-val is deprecated.  Use imu_3dm_gx4-msg:euler_angle_covariance_status instead.")
  (euler_angle_covariance_status m))

(cl:ensure-generic-function 'heading_update_alt-val :lambda-list '(m))
(cl:defmethod heading_update_alt-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:heading_update_alt-val is deprecated.  Use imu_3dm_gx4-msg:heading_update_alt instead.")
  (heading_update_alt m))

(cl:ensure-generic-function 'heading_update_LORD-val :lambda-list '(m))
(cl:defmethod heading_update_LORD-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:heading_update_LORD-val is deprecated.  Use imu_3dm_gx4-msg:heading_update_LORD instead.")
  (heading_update_LORD m))

(cl:ensure-generic-function 'heading_update_uncertainty-val :lambda-list '(m))
(cl:defmethod heading_update_uncertainty-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:heading_update_uncertainty-val is deprecated.  Use imu_3dm_gx4-msg:heading_update_uncertainty instead.")
  (heading_update_uncertainty m))

(cl:ensure-generic-function 'heading_update_source-val :lambda-list '(m))
(cl:defmethod heading_update_source-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:heading_update_source-val is deprecated.  Use imu_3dm_gx4-msg:heading_update_source instead.")
  (heading_update_source m))

(cl:ensure-generic-function 'heading_update_flags-val :lambda-list '(m))
(cl:defmethod heading_update_flags-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:heading_update_flags-val is deprecated.  Use imu_3dm_gx4-msg:heading_update_flags instead.")
  (heading_update_flags m))

(cl:ensure-generic-function 'gyro_bias-val :lambda-list '(m))
(cl:defmethod gyro_bias-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:gyro_bias-val is deprecated.  Use imu_3dm_gx4-msg:gyro_bias instead.")
  (gyro_bias m))

(cl:ensure-generic-function 'gyro_bias_status-val :lambda-list '(m))
(cl:defmethod gyro_bias_status-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:gyro_bias_status-val is deprecated.  Use imu_3dm_gx4-msg:gyro_bias_status instead.")
  (gyro_bias_status m))

(cl:ensure-generic-function 'gyro_bias_covariance-val :lambda-list '(m))
(cl:defmethod gyro_bias_covariance-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:gyro_bias_covariance-val is deprecated.  Use imu_3dm_gx4-msg:gyro_bias_covariance instead.")
  (gyro_bias_covariance m))

(cl:ensure-generic-function 'gyro_bias_covariance_status-val :lambda-list '(m))
(cl:defmethod gyro_bias_covariance_status-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:gyro_bias_covariance_status-val is deprecated.  Use imu_3dm_gx4-msg:gyro_bias_covariance_status instead.")
  (gyro_bias_covariance_status m))

(cl:ensure-generic-function 'linear_acceleration-val :lambda-list '(m))
(cl:defmethod linear_acceleration-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:linear_acceleration-val is deprecated.  Use imu_3dm_gx4-msg:linear_acceleration instead.")
  (linear_acceleration m))

(cl:ensure-generic-function 'linear_acceleration_status-val :lambda-list '(m))
(cl:defmethod linear_acceleration_status-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:linear_acceleration_status-val is deprecated.  Use imu_3dm_gx4-msg:linear_acceleration_status instead.")
  (linear_acceleration_status m))

(cl:ensure-generic-function 'angular_velocity-val :lambda-list '(m))
(cl:defmethod angular_velocity-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:angular_velocity-val is deprecated.  Use imu_3dm_gx4-msg:angular_velocity instead.")
  (angular_velocity m))

(cl:ensure-generic-function 'angular_velocity_status-val :lambda-list '(m))
(cl:defmethod angular_velocity_status-val ((m <FilterOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu_3dm_gx4-msg:angular_velocity_status-val is deprecated.  Use imu_3dm_gx4-msg:angular_velocity_status instead.")
  (angular_velocity_status m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<FilterOutput>)))
    "Constants for message type '<FilterOutput>"
  '((:STATUS_INVALID . 0)
    (:STATUS_VALID . 1)
    (:STATUS_VALID_REFERENCED . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'FilterOutput)))
    "Constants for message type 'FilterOutput"
  '((:STATUS_INVALID . 0)
    (:STATUS_VALID . 1)
    (:STATUS_VALID_REFERENCED . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FilterOutput>) ostream)
  "Serializes a message object of type '<FilterOutput>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'quaternion) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'quaternion_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'quaternion_status)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'euler_rpy) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'euler_rpy_status))))
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
   (cl:slot-value msg 'euler_angle_covariance))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'euler_angle_covariance_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'euler_angle_covariance_status)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading_update_alt))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading_update_LORD))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading_update_uncertainty))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading_update_source))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'heading_update_flags))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gyro_bias) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gyro_bias_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gyro_bias_status)) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'gyro_bias_covariance))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gyro_bias_covariance_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gyro_bias_covariance_status)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'linear_acceleration) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'linear_acceleration_status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angular_velocity) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angular_velocity_status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FilterOutput>) istream)
  "Deserializes a message object of type '<FilterOutput>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'quaternion) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'quaternion_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'quaternion_status)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'euler_rpy) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'euler_rpy_status) (roslisp-utils:decode-double-float-bits bits)))
  (cl:setf (cl:slot-value msg 'euler_angle_covariance) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'euler_angle_covariance)))
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
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'euler_angle_covariance_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'euler_angle_covariance_status)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_update_alt) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_update_LORD) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_update_uncertainty) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_update_source) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading_update_flags) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gyro_bias) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gyro_bias_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gyro_bias_status)) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'gyro_bias_covariance) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'gyro_bias_covariance)))
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
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gyro_bias_covariance_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gyro_bias_covariance_status)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'linear_acceleration) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear_acceleration_status) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angular_velocity) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_velocity_status) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FilterOutput>)))
  "Returns string type for a message object of type '<FilterOutput>"
  "imu_3dm_gx4/FilterOutput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FilterOutput)))
  "Returns string type for a message object of type 'FilterOutput"
  "imu_3dm_gx4/FilterOutput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FilterOutput>)))
  "Returns md5sum for a message object of type '<FilterOutput>"
  "9ebeb70bfcf2c275d32570ff4a241bad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FilterOutput)))
  "Returns md5sum for a message object of type 'FilterOutput"
  "9ebeb70bfcf2c275d32570ff4a241bad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FilterOutput>)))
  "Returns full string definition for message of type '<FilterOutput>"
  (cl:format cl:nil "# Output from the 3DM-GX4 attitude estimation filter.~%~%# Note on status flags:~%# Status flags are implemented as bit-fields.~%#  0 = invalid~%#  1 = valid~%#  2 = valid and referenced to magnetic north~%#~%# Note that covariance on orientation becomes invalid as pitch angle exceeds 70 # degrees. ~%# This will be indicated by the status flag.~%~%std_msgs/Header header~%~%# Quaternion, and status~%geometry_msgs/Quaternion quaternion~%uint16 quaternion_status~%~%# Gyroscope Euler angles roll, pitch, yaw, and status~%geometry_msgs/Vector3 euler_rpy #Roll, Pitch, and Yaw in [radians]~%float64 euler_rpy_status~%float64[9] euler_angle_covariance~%uint16 euler_angle_covariance_status~%~%# Heading Update Data~%float64 heading_update_alt #Heading in [radians]~%float64 heading_update_LORD #Heading in [radians]~%float64 heading_update_uncertainty #1-sigma heading uncertainty~%float64 heading_update_source~%float64 heading_update_flags #0 = no update received within 2 sec, 1 = update received within 2 sec~%~%# Gyro bias, diagonal covariance, and status~%geometry_msgs/Vector3 gyro_bias #Gyro bias from sensor frame [radians/sec]~%uint16 gyro_bias_status~%float64[9] gyro_bias_covariance~%uint16 gyro_bias_covariance_status~%~%# Linear accelerations along x,y,z axes, and status~%geometry_msgs/Vector3 linear_acceleration #X, Y, and Z axes in [m/s^2]~%float64 linear_acceleration_status~%~%# Angular rates along x,y,z axes, and status~%geometry_msgs/Vector3 angular_velocity #X, Y, and Z axes in [radians/s]~%float64 angular_velocity_status~%~%# Constants~%uint16 STATUS_INVALID = 0~%uint16 STATUS_VALID = 1~%uint16 STATUS_VALID_REFERENCED = 2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FilterOutput)))
  "Returns full string definition for message of type 'FilterOutput"
  (cl:format cl:nil "# Output from the 3DM-GX4 attitude estimation filter.~%~%# Note on status flags:~%# Status flags are implemented as bit-fields.~%#  0 = invalid~%#  1 = valid~%#  2 = valid and referenced to magnetic north~%#~%# Note that covariance on orientation becomes invalid as pitch angle exceeds 70 # degrees. ~%# This will be indicated by the status flag.~%~%std_msgs/Header header~%~%# Quaternion, and status~%geometry_msgs/Quaternion quaternion~%uint16 quaternion_status~%~%# Gyroscope Euler angles roll, pitch, yaw, and status~%geometry_msgs/Vector3 euler_rpy #Roll, Pitch, and Yaw in [radians]~%float64 euler_rpy_status~%float64[9] euler_angle_covariance~%uint16 euler_angle_covariance_status~%~%# Heading Update Data~%float64 heading_update_alt #Heading in [radians]~%float64 heading_update_LORD #Heading in [radians]~%float64 heading_update_uncertainty #1-sigma heading uncertainty~%float64 heading_update_source~%float64 heading_update_flags #0 = no update received within 2 sec, 1 = update received within 2 sec~%~%# Gyro bias, diagonal covariance, and status~%geometry_msgs/Vector3 gyro_bias #Gyro bias from sensor frame [radians/sec]~%uint16 gyro_bias_status~%float64[9] gyro_bias_covariance~%uint16 gyro_bias_covariance_status~%~%# Linear accelerations along x,y,z axes, and status~%geometry_msgs/Vector3 linear_acceleration #X, Y, and Z axes in [m/s^2]~%float64 linear_acceleration_status~%~%# Angular rates along x,y,z axes, and status~%geometry_msgs/Vector3 angular_velocity #X, Y, and Z axes in [radians/s]~%float64 angular_velocity_status~%~%# Constants~%uint16 STATUS_INVALID = 0~%uint16 STATUS_VALID = 1~%uint16 STATUS_VALID_REFERENCED = 2~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FilterOutput>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'quaternion))
     2
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'euler_rpy))
     8
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'euler_angle_covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     2
     8
     8
     8
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gyro_bias))
     2
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gyro_bias_covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     2
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'linear_acceleration))
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angular_velocity))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FilterOutput>))
  "Converts a ROS message object to a list"
  (cl:list 'FilterOutput
    (cl:cons ':header (header msg))
    (cl:cons ':quaternion (quaternion msg))
    (cl:cons ':quaternion_status (quaternion_status msg))
    (cl:cons ':euler_rpy (euler_rpy msg))
    (cl:cons ':euler_rpy_status (euler_rpy_status msg))
    (cl:cons ':euler_angle_covariance (euler_angle_covariance msg))
    (cl:cons ':euler_angle_covariance_status (euler_angle_covariance_status msg))
    (cl:cons ':heading_update_alt (heading_update_alt msg))
    (cl:cons ':heading_update_LORD (heading_update_LORD msg))
    (cl:cons ':heading_update_uncertainty (heading_update_uncertainty msg))
    (cl:cons ':heading_update_source (heading_update_source msg))
    (cl:cons ':heading_update_flags (heading_update_flags msg))
    (cl:cons ':gyro_bias (gyro_bias msg))
    (cl:cons ':gyro_bias_status (gyro_bias_status msg))
    (cl:cons ':gyro_bias_covariance (gyro_bias_covariance msg))
    (cl:cons ':gyro_bias_covariance_status (gyro_bias_covariance_status msg))
    (cl:cons ':linear_acceleration (linear_acceleration msg))
    (cl:cons ':linear_acceleration_status (linear_acceleration_status msg))
    (cl:cons ':angular_velocity (angular_velocity msg))
    (cl:cons ':angular_velocity_status (angular_velocity_status msg))
))
