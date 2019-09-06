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

class MagFieldCF {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.components = null;
      this.magnitude = null;
      this.covariance = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('components')) {
        this.components = initObj.components
      }
      else {
        this.components = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('magnitude')) {
        this.magnitude = initObj.magnitude
      }
      else {
        this.magnitude = 0.0;
      }
      if (initObj.hasOwnProperty('covariance')) {
        this.covariance = initObj.covariance
      }
      else {
        this.covariance = new Array(9).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MagFieldCF
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [components]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.components, buffer, bufferOffset);
    // Serialize message field [magnitude]
    bufferOffset = _serializer.float64(obj.magnitude, buffer, bufferOffset);
    // Check that the constant length array field [covariance] has the right length
    if (obj.covariance.length !== 9) {
      throw new Error('Unable to serialize array field covariance - length must be 9')
    }
    // Serialize message field [covariance]
    bufferOffset = _arraySerializer.float64(obj.covariance, buffer, bufferOffset, 9);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MagFieldCF
    let len;
    let data = new MagFieldCF(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [components]
    data.components = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [magnitude]
    data.magnitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [covariance]
    data.covariance = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 104;
  }

  static datatype() {
    // Returns string type for a message object
    return 'imu_3dm_gx4/MagFieldCF';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cd32a4ab24c73ea547c18eb391466a22';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    #Magnetic Field Message - Complimentary Filter
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MagFieldCF(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.components !== undefined) {
      resolved.components = geometry_msgs.msg.Vector3.Resolve(msg.components)
    }
    else {
      resolved.components = new geometry_msgs.msg.Vector3()
    }

    if (msg.magnitude !== undefined) {
      resolved.magnitude = msg.magnitude;
    }
    else {
      resolved.magnitude = 0.0
    }

    if (msg.covariance !== undefined) {
      resolved.covariance = msg.covariance;
    }
    else {
      resolved.covariance = new Array(9).fill(0)
    }

    return resolved;
    }
};

module.exports = MagFieldCF;
