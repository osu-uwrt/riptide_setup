// Auto-generated. Do not edit!

// (in-package imu_3dm_gx4.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class FilterOutput {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.quaternion = null;
      this.quaternion_status = null;
      this.euler_rpy = null;
      this.euler_rpy_status = null;
      this.euler_angle_covariance = null;
      this.euler_angle_covariance_status = null;
      this.heading_update_alt = null;
      this.heading_update_LORD = null;
      this.heading_update_uncertainty = null;
      this.heading_update_source = null;
      this.heading_update_flags = null;
      this.gyro_bias = null;
      this.gyro_bias_status = null;
      this.gyro_bias_covariance = null;
      this.gyro_bias_covariance_status = null;
      this.linear_acceleration = null;
      this.linear_acceleration_status = null;
      this.angular_velocity = null;
      this.angular_velocity_status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('quaternion')) {
        this.quaternion = initObj.quaternion
      }
      else {
        this.quaternion = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('quaternion_status')) {
        this.quaternion_status = initObj.quaternion_status
      }
      else {
        this.quaternion_status = 0;
      }
      if (initObj.hasOwnProperty('euler_rpy')) {
        this.euler_rpy = initObj.euler_rpy
      }
      else {
        this.euler_rpy = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('euler_rpy_status')) {
        this.euler_rpy_status = initObj.euler_rpy_status
      }
      else {
        this.euler_rpy_status = 0.0;
      }
      if (initObj.hasOwnProperty('euler_angle_covariance')) {
        this.euler_angle_covariance = initObj.euler_angle_covariance
      }
      else {
        this.euler_angle_covariance = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('euler_angle_covariance_status')) {
        this.euler_angle_covariance_status = initObj.euler_angle_covariance_status
      }
      else {
        this.euler_angle_covariance_status = 0;
      }
      if (initObj.hasOwnProperty('heading_update_alt')) {
        this.heading_update_alt = initObj.heading_update_alt
      }
      else {
        this.heading_update_alt = 0.0;
      }
      if (initObj.hasOwnProperty('heading_update_LORD')) {
        this.heading_update_LORD = initObj.heading_update_LORD
      }
      else {
        this.heading_update_LORD = 0.0;
      }
      if (initObj.hasOwnProperty('heading_update_uncertainty')) {
        this.heading_update_uncertainty = initObj.heading_update_uncertainty
      }
      else {
        this.heading_update_uncertainty = 0.0;
      }
      if (initObj.hasOwnProperty('heading_update_source')) {
        this.heading_update_source = initObj.heading_update_source
      }
      else {
        this.heading_update_source = 0.0;
      }
      if (initObj.hasOwnProperty('heading_update_flags')) {
        this.heading_update_flags = initObj.heading_update_flags
      }
      else {
        this.heading_update_flags = 0.0;
      }
      if (initObj.hasOwnProperty('gyro_bias')) {
        this.gyro_bias = initObj.gyro_bias
      }
      else {
        this.gyro_bias = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('gyro_bias_status')) {
        this.gyro_bias_status = initObj.gyro_bias_status
      }
      else {
        this.gyro_bias_status = 0;
      }
      if (initObj.hasOwnProperty('gyro_bias_covariance')) {
        this.gyro_bias_covariance = initObj.gyro_bias_covariance
      }
      else {
        this.gyro_bias_covariance = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('gyro_bias_covariance_status')) {
        this.gyro_bias_covariance_status = initObj.gyro_bias_covariance_status
      }
      else {
        this.gyro_bias_covariance_status = 0;
      }
      if (initObj.hasOwnProperty('linear_acceleration')) {
        this.linear_acceleration = initObj.linear_acceleration
      }
      else {
        this.linear_acceleration = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('linear_acceleration_status')) {
        this.linear_acceleration_status = initObj.linear_acceleration_status
      }
      else {
        this.linear_acceleration_status = 0.0;
      }
      if (initObj.hasOwnProperty('angular_velocity')) {
        this.angular_velocity = initObj.angular_velocity
      }
      else {
        this.angular_velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('angular_velocity_status')) {
        this.angular_velocity_status = initObj.angular_velocity_status
      }
      else {
        this.angular_velocity_status = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FilterOutput
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [quaternion]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.quaternion, buffer, bufferOffset);
    // Serialize message field [quaternion_status]
    bufferOffset = _serializer.uint16(obj.quaternion_status, buffer, bufferOffset);
    // Serialize message field [euler_rpy]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.euler_rpy, buffer, bufferOffset);
    // Serialize message field [euler_rpy_status]
    bufferOffset = _serializer.float64(obj.euler_rpy_status, buffer, bufferOffset);
    // Check that the constant length array field [euler_angle_covariance] has the right length
    if (obj.euler_angle_covariance.length !== 9) {
      throw new Error('Unable to serialize array field euler_angle_covariance - length must be 9')
    }
    // Serialize message field [euler_angle_covariance]
    bufferOffset = _arraySerializer.float64(obj.euler_angle_covariance, buffer, bufferOffset, 9);
    // Serialize message field [euler_angle_covariance_status]
    bufferOffset = _serializer.uint16(obj.euler_angle_covariance_status, buffer, bufferOffset);
    // Serialize message field [heading_update_alt]
    bufferOffset = _serializer.float64(obj.heading_update_alt, buffer, bufferOffset);
    // Serialize message field [heading_update_LORD]
    bufferOffset = _serializer.float64(obj.heading_update_LORD, buffer, bufferOffset);
    // Serialize message field [heading_update_uncertainty]
    bufferOffset = _serializer.float64(obj.heading_update_uncertainty, buffer, bufferOffset);
    // Serialize message field [heading_update_source]
    bufferOffset = _serializer.float64(obj.heading_update_source, buffer, bufferOffset);
    // Serialize message field [heading_update_flags]
    bufferOffset = _serializer.float64(obj.heading_update_flags, buffer, bufferOffset);
    // Serialize message field [gyro_bias]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.gyro_bias, buffer, bufferOffset);
    // Serialize message field [gyro_bias_status]
    bufferOffset = _serializer.uint16(obj.gyro_bias_status, buffer, bufferOffset);
    // Check that the constant length array field [gyro_bias_covariance] has the right length
    if (obj.gyro_bias_covariance.length !== 9) {
      throw new Error('Unable to serialize array field gyro_bias_covariance - length must be 9')
    }
    // Serialize message field [gyro_bias_covariance]
    bufferOffset = _arraySerializer.float64(obj.gyro_bias_covariance, buffer, bufferOffset, 9);
    // Serialize message field [gyro_bias_covariance_status]
    bufferOffset = _serializer.uint16(obj.gyro_bias_covariance_status, buffer, bufferOffset);
    // Serialize message field [linear_acceleration]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.linear_acceleration, buffer, bufferOffset);
    // Serialize message field [linear_acceleration_status]
    bufferOffset = _serializer.float64(obj.linear_acceleration_status, buffer, bufferOffset);
    // Serialize message field [angular_velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.angular_velocity, buffer, bufferOffset);
    // Serialize message field [angular_velocity_status]
    bufferOffset = _serializer.float64(obj.angular_velocity_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FilterOutput
    let len;
    let data = new FilterOutput(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [quaternion]
    data.quaternion = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [quaternion_status]
    data.quaternion_status = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [euler_rpy]
    data.euler_rpy = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [euler_rpy_status]
    data.euler_rpy_status = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [euler_angle_covariance]
    data.euler_angle_covariance = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [euler_angle_covariance_status]
    data.euler_angle_covariance_status = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [heading_update_alt]
    data.heading_update_alt = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading_update_LORD]
    data.heading_update_LORD = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading_update_uncertainty]
    data.heading_update_uncertainty = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading_update_source]
    data.heading_update_source = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading_update_flags]
    data.heading_update_flags = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gyro_bias]
    data.gyro_bias = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [gyro_bias_status]
    data.gyro_bias_status = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [gyro_bias_covariance]
    data.gyro_bias_covariance = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [gyro_bias_covariance_status]
    data.gyro_bias_covariance_status = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [linear_acceleration]
    data.linear_acceleration = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [linear_acceleration_status]
    data.linear_acceleration_status = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [angular_velocity]
    data.angular_velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [angular_velocity_status]
    data.angular_velocity_status = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 344;
  }

  static datatype() {
    // Returns string type for a message object
    return 'imu_3dm_gx4/FilterOutput';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9ebeb70bfcf2c275d32570ff4a241bad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Output from the 3DM-GX4 attitude estimation filter.
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FilterOutput(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.quaternion !== undefined) {
      resolved.quaternion = geometry_msgs.msg.Quaternion.Resolve(msg.quaternion)
    }
    else {
      resolved.quaternion = new geometry_msgs.msg.Quaternion()
    }

    if (msg.quaternion_status !== undefined) {
      resolved.quaternion_status = msg.quaternion_status;
    }
    else {
      resolved.quaternion_status = 0
    }

    if (msg.euler_rpy !== undefined) {
      resolved.euler_rpy = geometry_msgs.msg.Vector3.Resolve(msg.euler_rpy)
    }
    else {
      resolved.euler_rpy = new geometry_msgs.msg.Vector3()
    }

    if (msg.euler_rpy_status !== undefined) {
      resolved.euler_rpy_status = msg.euler_rpy_status;
    }
    else {
      resolved.euler_rpy_status = 0.0
    }

    if (msg.euler_angle_covariance !== undefined) {
      resolved.euler_angle_covariance = msg.euler_angle_covariance;
    }
    else {
      resolved.euler_angle_covariance = new Array(9).fill(0)
    }

    if (msg.euler_angle_covariance_status !== undefined) {
      resolved.euler_angle_covariance_status = msg.euler_angle_covariance_status;
    }
    else {
      resolved.euler_angle_covariance_status = 0
    }

    if (msg.heading_update_alt !== undefined) {
      resolved.heading_update_alt = msg.heading_update_alt;
    }
    else {
      resolved.heading_update_alt = 0.0
    }

    if (msg.heading_update_LORD !== undefined) {
      resolved.heading_update_LORD = msg.heading_update_LORD;
    }
    else {
      resolved.heading_update_LORD = 0.0
    }

    if (msg.heading_update_uncertainty !== undefined) {
      resolved.heading_update_uncertainty = msg.heading_update_uncertainty;
    }
    else {
      resolved.heading_update_uncertainty = 0.0
    }

    if (msg.heading_update_source !== undefined) {
      resolved.heading_update_source = msg.heading_update_source;
    }
    else {
      resolved.heading_update_source = 0.0
    }

    if (msg.heading_update_flags !== undefined) {
      resolved.heading_update_flags = msg.heading_update_flags;
    }
    else {
      resolved.heading_update_flags = 0.0
    }

    if (msg.gyro_bias !== undefined) {
      resolved.gyro_bias = geometry_msgs.msg.Vector3.Resolve(msg.gyro_bias)
    }
    else {
      resolved.gyro_bias = new geometry_msgs.msg.Vector3()
    }

    if (msg.gyro_bias_status !== undefined) {
      resolved.gyro_bias_status = msg.gyro_bias_status;
    }
    else {
      resolved.gyro_bias_status = 0
    }

    if (msg.gyro_bias_covariance !== undefined) {
      resolved.gyro_bias_covariance = msg.gyro_bias_covariance;
    }
    else {
      resolved.gyro_bias_covariance = new Array(9).fill(0)
    }

    if (msg.gyro_bias_covariance_status !== undefined) {
      resolved.gyro_bias_covariance_status = msg.gyro_bias_covariance_status;
    }
    else {
      resolved.gyro_bias_covariance_status = 0
    }

    if (msg.linear_acceleration !== undefined) {
      resolved.linear_acceleration = geometry_msgs.msg.Vector3.Resolve(msg.linear_acceleration)
    }
    else {
      resolved.linear_acceleration = new geometry_msgs.msg.Vector3()
    }

    if (msg.linear_acceleration_status !== undefined) {
      resolved.linear_acceleration_status = msg.linear_acceleration_status;
    }
    else {
      resolved.linear_acceleration_status = 0.0
    }

    if (msg.angular_velocity !== undefined) {
      resolved.angular_velocity = geometry_msgs.msg.Vector3.Resolve(msg.angular_velocity)
    }
    else {
      resolved.angular_velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.angular_velocity_status !== undefined) {
      resolved.angular_velocity_status = msg.angular_velocity_status;
    }
    else {
      resolved.angular_velocity_status = 0.0
    }

    return resolved;
    }
};

// Constants for message
FilterOutput.Constants = {
  STATUS_INVALID: 0,
  STATUS_VALID: 1,
  STATUS_VALID_REFERENCED: 2,
}

module.exports = FilterOutput;
