// Auto-generated. Do not edit!

// (in-package imu_3dm_gx4.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class MagFieldKF {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.mag_field_N = null;
      this.mag_field_E = null;
      this.mag_field_D = null;
      this.mag_field_magnitude = null;
      this.inclination = null;
      this.declination = null;
      this.mag_status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('mag_field_N')) {
        this.mag_field_N = initObj.mag_field_N
      }
      else {
        this.mag_field_N = 0.0;
      }
      if (initObj.hasOwnProperty('mag_field_E')) {
        this.mag_field_E = initObj.mag_field_E
      }
      else {
        this.mag_field_E = 0.0;
      }
      if (initObj.hasOwnProperty('mag_field_D')) {
        this.mag_field_D = initObj.mag_field_D
      }
      else {
        this.mag_field_D = 0.0;
      }
      if (initObj.hasOwnProperty('mag_field_magnitude')) {
        this.mag_field_magnitude = initObj.mag_field_magnitude
      }
      else {
        this.mag_field_magnitude = 0.0;
      }
      if (initObj.hasOwnProperty('inclination')) {
        this.inclination = initObj.inclination
      }
      else {
        this.inclination = 0.0;
      }
      if (initObj.hasOwnProperty('declination')) {
        this.declination = initObj.declination
      }
      else {
        this.declination = 0.0;
      }
      if (initObj.hasOwnProperty('mag_status')) {
        this.mag_status = initObj.mag_status
      }
      else {
        this.mag_status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MagFieldKF
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [mag_field_N]
    bufferOffset = _serializer.float64(obj.mag_field_N, buffer, bufferOffset);
    // Serialize message field [mag_field_E]
    bufferOffset = _serializer.float64(obj.mag_field_E, buffer, bufferOffset);
    // Serialize message field [mag_field_D]
    bufferOffset = _serializer.float64(obj.mag_field_D, buffer, bufferOffset);
    // Serialize message field [mag_field_magnitude]
    bufferOffset = _serializer.float64(obj.mag_field_magnitude, buffer, bufferOffset);
    // Serialize message field [inclination]
    bufferOffset = _serializer.float64(obj.inclination, buffer, bufferOffset);
    // Serialize message field [declination]
    bufferOffset = _serializer.float64(obj.declination, buffer, bufferOffset);
    // Serialize message field [mag_status]
    bufferOffset = _serializer.uint16(obj.mag_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MagFieldKF
    let len;
    let data = new MagFieldKF(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [mag_field_N]
    data.mag_field_N = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mag_field_E]
    data.mag_field_E = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mag_field_D]
    data.mag_field_D = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mag_field_magnitude]
    data.mag_field_magnitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [inclination]
    data.inclination = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [declination]
    data.declination = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mag_status]
    data.mag_status = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 50;
  }

  static datatype() {
    // Returns string type for a message object
    return 'imu_3dm_gx4/MagFieldKF';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '80d2cbcb915a78650f5f0199105cb92b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Magnetic Field Message - Estimation Kalman Filter
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MagFieldKF(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.mag_field_N !== undefined) {
      resolved.mag_field_N = msg.mag_field_N;
    }
    else {
      resolved.mag_field_N = 0.0
    }

    if (msg.mag_field_E !== undefined) {
      resolved.mag_field_E = msg.mag_field_E;
    }
    else {
      resolved.mag_field_E = 0.0
    }

    if (msg.mag_field_D !== undefined) {
      resolved.mag_field_D = msg.mag_field_D;
    }
    else {
      resolved.mag_field_D = 0.0
    }

    if (msg.mag_field_magnitude !== undefined) {
      resolved.mag_field_magnitude = msg.mag_field_magnitude;
    }
    else {
      resolved.mag_field_magnitude = 0.0
    }

    if (msg.inclination !== undefined) {
      resolved.inclination = msg.inclination;
    }
    else {
      resolved.inclination = 0.0
    }

    if (msg.declination !== undefined) {
      resolved.declination = msg.declination;
    }
    else {
      resolved.declination = 0.0
    }

    if (msg.mag_status !== undefined) {
      resolved.mag_status = msg.mag_status;
    }
    else {
      resolved.mag_status = 0
    }

    return resolved;
    }
};

module.exports = MagFieldKF;
