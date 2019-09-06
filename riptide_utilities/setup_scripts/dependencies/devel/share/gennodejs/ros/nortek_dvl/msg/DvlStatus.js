// Auto-generated. Do not edit!

// (in-package nortek_dvl.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class DvlStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.b1_vel_valid = null;
      this.b2_vel_valid = null;
      this.b3_vel_valid = null;
      this.b4_vel_valid = null;
      this.b1_dist_valid = null;
      this.b2_dist_valid = null;
      this.b3_dist_valid = null;
      this.b4_dist_valid = null;
      this.b1_fom_valid = null;
      this.b2_fom_valid = null;
      this.b3_fom_valid = null;
      this.b4_fom_valid = null;
      this.x_vel_valid = null;
      this.y_vel_valid = null;
      this.z1_vel_valid = null;
      this.z2_vel_valid = null;
      this.x_fom_valid = null;
      this.y_fom_valid = null;
      this.z1_fom_valid = null;
      this.z2_fom_valid = null;
      this.proc_cap = null;
      this.wakeup_state = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('b1_vel_valid')) {
        this.b1_vel_valid = initObj.b1_vel_valid
      }
      else {
        this.b1_vel_valid = false;
      }
      if (initObj.hasOwnProperty('b2_vel_valid')) {
        this.b2_vel_valid = initObj.b2_vel_valid
      }
      else {
        this.b2_vel_valid = false;
      }
      if (initObj.hasOwnProperty('b3_vel_valid')) {
        this.b3_vel_valid = initObj.b3_vel_valid
      }
      else {
        this.b3_vel_valid = false;
      }
      if (initObj.hasOwnProperty('b4_vel_valid')) {
        this.b4_vel_valid = initObj.b4_vel_valid
      }
      else {
        this.b4_vel_valid = false;
      }
      if (initObj.hasOwnProperty('b1_dist_valid')) {
        this.b1_dist_valid = initObj.b1_dist_valid
      }
      else {
        this.b1_dist_valid = false;
      }
      if (initObj.hasOwnProperty('b2_dist_valid')) {
        this.b2_dist_valid = initObj.b2_dist_valid
      }
      else {
        this.b2_dist_valid = false;
      }
      if (initObj.hasOwnProperty('b3_dist_valid')) {
        this.b3_dist_valid = initObj.b3_dist_valid
      }
      else {
        this.b3_dist_valid = false;
      }
      if (initObj.hasOwnProperty('b4_dist_valid')) {
        this.b4_dist_valid = initObj.b4_dist_valid
      }
      else {
        this.b4_dist_valid = false;
      }
      if (initObj.hasOwnProperty('b1_fom_valid')) {
        this.b1_fom_valid = initObj.b1_fom_valid
      }
      else {
        this.b1_fom_valid = false;
      }
      if (initObj.hasOwnProperty('b2_fom_valid')) {
        this.b2_fom_valid = initObj.b2_fom_valid
      }
      else {
        this.b2_fom_valid = false;
      }
      if (initObj.hasOwnProperty('b3_fom_valid')) {
        this.b3_fom_valid = initObj.b3_fom_valid
      }
      else {
        this.b3_fom_valid = false;
      }
      if (initObj.hasOwnProperty('b4_fom_valid')) {
        this.b4_fom_valid = initObj.b4_fom_valid
      }
      else {
        this.b4_fom_valid = false;
      }
      if (initObj.hasOwnProperty('x_vel_valid')) {
        this.x_vel_valid = initObj.x_vel_valid
      }
      else {
        this.x_vel_valid = false;
      }
      if (initObj.hasOwnProperty('y_vel_valid')) {
        this.y_vel_valid = initObj.y_vel_valid
      }
      else {
        this.y_vel_valid = false;
      }
      if (initObj.hasOwnProperty('z1_vel_valid')) {
        this.z1_vel_valid = initObj.z1_vel_valid
      }
      else {
        this.z1_vel_valid = false;
      }
      if (initObj.hasOwnProperty('z2_vel_valid')) {
        this.z2_vel_valid = initObj.z2_vel_valid
      }
      else {
        this.z2_vel_valid = false;
      }
      if (initObj.hasOwnProperty('x_fom_valid')) {
        this.x_fom_valid = initObj.x_fom_valid
      }
      else {
        this.x_fom_valid = false;
      }
      if (initObj.hasOwnProperty('y_fom_valid')) {
        this.y_fom_valid = initObj.y_fom_valid
      }
      else {
        this.y_fom_valid = false;
      }
      if (initObj.hasOwnProperty('z1_fom_valid')) {
        this.z1_fom_valid = initObj.z1_fom_valid
      }
      else {
        this.z1_fom_valid = false;
      }
      if (initObj.hasOwnProperty('z2_fom_valid')) {
        this.z2_fom_valid = initObj.z2_fom_valid
      }
      else {
        this.z2_fom_valid = false;
      }
      if (initObj.hasOwnProperty('proc_cap')) {
        this.proc_cap = initObj.proc_cap
      }
      else {
        this.proc_cap = 0;
      }
      if (initObj.hasOwnProperty('wakeup_state')) {
        this.wakeup_state = initObj.wakeup_state
      }
      else {
        this.wakeup_state = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DvlStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [b1_vel_valid]
    bufferOffset = _serializer.bool(obj.b1_vel_valid, buffer, bufferOffset);
    // Serialize message field [b2_vel_valid]
    bufferOffset = _serializer.bool(obj.b2_vel_valid, buffer, bufferOffset);
    // Serialize message field [b3_vel_valid]
    bufferOffset = _serializer.bool(obj.b3_vel_valid, buffer, bufferOffset);
    // Serialize message field [b4_vel_valid]
    bufferOffset = _serializer.bool(obj.b4_vel_valid, buffer, bufferOffset);
    // Serialize message field [b1_dist_valid]
    bufferOffset = _serializer.bool(obj.b1_dist_valid, buffer, bufferOffset);
    // Serialize message field [b2_dist_valid]
    bufferOffset = _serializer.bool(obj.b2_dist_valid, buffer, bufferOffset);
    // Serialize message field [b3_dist_valid]
    bufferOffset = _serializer.bool(obj.b3_dist_valid, buffer, bufferOffset);
    // Serialize message field [b4_dist_valid]
    bufferOffset = _serializer.bool(obj.b4_dist_valid, buffer, bufferOffset);
    // Serialize message field [b1_fom_valid]
    bufferOffset = _serializer.bool(obj.b1_fom_valid, buffer, bufferOffset);
    // Serialize message field [b2_fom_valid]
    bufferOffset = _serializer.bool(obj.b2_fom_valid, buffer, bufferOffset);
    // Serialize message field [b3_fom_valid]
    bufferOffset = _serializer.bool(obj.b3_fom_valid, buffer, bufferOffset);
    // Serialize message field [b4_fom_valid]
    bufferOffset = _serializer.bool(obj.b4_fom_valid, buffer, bufferOffset);
    // Serialize message field [x_vel_valid]
    bufferOffset = _serializer.bool(obj.x_vel_valid, buffer, bufferOffset);
    // Serialize message field [y_vel_valid]
    bufferOffset = _serializer.bool(obj.y_vel_valid, buffer, bufferOffset);
    // Serialize message field [z1_vel_valid]
    bufferOffset = _serializer.bool(obj.z1_vel_valid, buffer, bufferOffset);
    // Serialize message field [z2_vel_valid]
    bufferOffset = _serializer.bool(obj.z2_vel_valid, buffer, bufferOffset);
    // Serialize message field [x_fom_valid]
    bufferOffset = _serializer.bool(obj.x_fom_valid, buffer, bufferOffset);
    // Serialize message field [y_fom_valid]
    bufferOffset = _serializer.bool(obj.y_fom_valid, buffer, bufferOffset);
    // Serialize message field [z1_fom_valid]
    bufferOffset = _serializer.bool(obj.z1_fom_valid, buffer, bufferOffset);
    // Serialize message field [z2_fom_valid]
    bufferOffset = _serializer.bool(obj.z2_fom_valid, buffer, bufferOffset);
    // Serialize message field [proc_cap]
    bufferOffset = _serializer.int8(obj.proc_cap, buffer, bufferOffset);
    // Serialize message field [wakeup_state]
    bufferOffset = _serializer.string(obj.wakeup_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DvlStatus
    let len;
    let data = new DvlStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [b1_vel_valid]
    data.b1_vel_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [b2_vel_valid]
    data.b2_vel_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [b3_vel_valid]
    data.b3_vel_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [b4_vel_valid]
    data.b4_vel_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [b1_dist_valid]
    data.b1_dist_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [b2_dist_valid]
    data.b2_dist_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [b3_dist_valid]
    data.b3_dist_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [b4_dist_valid]
    data.b4_dist_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [b1_fom_valid]
    data.b1_fom_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [b2_fom_valid]
    data.b2_fom_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [b3_fom_valid]
    data.b3_fom_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [b4_fom_valid]
    data.b4_fom_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [x_vel_valid]
    data.x_vel_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [y_vel_valid]
    data.y_vel_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [z1_vel_valid]
    data.z1_vel_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [z2_vel_valid]
    data.z2_vel_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [x_fom_valid]
    data.x_fom_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [y_fom_valid]
    data.y_fom_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [z1_fom_valid]
    data.z1_fom_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [z2_fom_valid]
    data.z2_fom_valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [proc_cap]
    data.proc_cap = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [wakeup_state]
    data.wakeup_state = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.wakeup_state.length;
    return length + 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nortek_dvl/DvlStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e1789433d0fd4a0c672172f8d32e464d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    bool b1_vel_valid
    bool b2_vel_valid
    bool b3_vel_valid
    bool b4_vel_valid
    
    bool b1_dist_valid
    bool b2_dist_valid
    bool b3_dist_valid
    bool b4_dist_valid
    
    bool b1_fom_valid
    bool b2_fom_valid
    bool b3_fom_valid
    bool b4_fom_valid
    
    bool x_vel_valid
    bool y_vel_valid
    bool z1_vel_valid
    bool z2_vel_valid
    
    bool x_fom_valid
    bool y_fom_valid
    bool z1_fom_valid
    bool z2_fom_valid
    
    int8 proc_cap
    string wakeup_state
    
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
    const resolved = new DvlStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.b1_vel_valid !== undefined) {
      resolved.b1_vel_valid = msg.b1_vel_valid;
    }
    else {
      resolved.b1_vel_valid = false
    }

    if (msg.b2_vel_valid !== undefined) {
      resolved.b2_vel_valid = msg.b2_vel_valid;
    }
    else {
      resolved.b2_vel_valid = false
    }

    if (msg.b3_vel_valid !== undefined) {
      resolved.b3_vel_valid = msg.b3_vel_valid;
    }
    else {
      resolved.b3_vel_valid = false
    }

    if (msg.b4_vel_valid !== undefined) {
      resolved.b4_vel_valid = msg.b4_vel_valid;
    }
    else {
      resolved.b4_vel_valid = false
    }

    if (msg.b1_dist_valid !== undefined) {
      resolved.b1_dist_valid = msg.b1_dist_valid;
    }
    else {
      resolved.b1_dist_valid = false
    }

    if (msg.b2_dist_valid !== undefined) {
      resolved.b2_dist_valid = msg.b2_dist_valid;
    }
    else {
      resolved.b2_dist_valid = false
    }

    if (msg.b3_dist_valid !== undefined) {
      resolved.b3_dist_valid = msg.b3_dist_valid;
    }
    else {
      resolved.b3_dist_valid = false
    }

    if (msg.b4_dist_valid !== undefined) {
      resolved.b4_dist_valid = msg.b4_dist_valid;
    }
    else {
      resolved.b4_dist_valid = false
    }

    if (msg.b1_fom_valid !== undefined) {
      resolved.b1_fom_valid = msg.b1_fom_valid;
    }
    else {
      resolved.b1_fom_valid = false
    }

    if (msg.b2_fom_valid !== undefined) {
      resolved.b2_fom_valid = msg.b2_fom_valid;
    }
    else {
      resolved.b2_fom_valid = false
    }

    if (msg.b3_fom_valid !== undefined) {
      resolved.b3_fom_valid = msg.b3_fom_valid;
    }
    else {
      resolved.b3_fom_valid = false
    }

    if (msg.b4_fom_valid !== undefined) {
      resolved.b4_fom_valid = msg.b4_fom_valid;
    }
    else {
      resolved.b4_fom_valid = false
    }

    if (msg.x_vel_valid !== undefined) {
      resolved.x_vel_valid = msg.x_vel_valid;
    }
    else {
      resolved.x_vel_valid = false
    }

    if (msg.y_vel_valid !== undefined) {
      resolved.y_vel_valid = msg.y_vel_valid;
    }
    else {
      resolved.y_vel_valid = false
    }

    if (msg.z1_vel_valid !== undefined) {
      resolved.z1_vel_valid = msg.z1_vel_valid;
    }
    else {
      resolved.z1_vel_valid = false
    }

    if (msg.z2_vel_valid !== undefined) {
      resolved.z2_vel_valid = msg.z2_vel_valid;
    }
    else {
      resolved.z2_vel_valid = false
    }

    if (msg.x_fom_valid !== undefined) {
      resolved.x_fom_valid = msg.x_fom_valid;
    }
    else {
      resolved.x_fom_valid = false
    }

    if (msg.y_fom_valid !== undefined) {
      resolved.y_fom_valid = msg.y_fom_valid;
    }
    else {
      resolved.y_fom_valid = false
    }

    if (msg.z1_fom_valid !== undefined) {
      resolved.z1_fom_valid = msg.z1_fom_valid;
    }
    else {
      resolved.z1_fom_valid = false
    }

    if (msg.z2_fom_valid !== undefined) {
      resolved.z2_fom_valid = msg.z2_fom_valid;
    }
    else {
      resolved.z2_fom_valid = false
    }

    if (msg.proc_cap !== undefined) {
      resolved.proc_cap = msg.proc_cap;
    }
    else {
      resolved.proc_cap = 0
    }

    if (msg.wakeup_state !== undefined) {
      resolved.wakeup_state = msg.wakeup_state;
    }
    else {
      resolved.wakeup_state = ''
    }

    return resolved;
    }
};

module.exports = DvlStatus;
