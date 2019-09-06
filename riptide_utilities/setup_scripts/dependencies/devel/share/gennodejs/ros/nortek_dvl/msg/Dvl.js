// Auto-generated. Do not edit!

// (in-package nortek_dvl.msg)


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

class Dvl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.time = null;
      this.dt1 = null;
      this.dt2 = null;
      this.velocity = null;
      this.figureOfMerit = null;
      this.beamDistance = null;
      this.batteryVoltage = null;
      this.speedSound = null;
      this.pressure = null;
      this.temp = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0.0;
      }
      if (initObj.hasOwnProperty('dt1')) {
        this.dt1 = initObj.dt1
      }
      else {
        this.dt1 = 0.0;
      }
      if (initObj.hasOwnProperty('dt2')) {
        this.dt2 = initObj.dt2
      }
      else {
        this.dt2 = 0.0;
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('figureOfMerit')) {
        this.figureOfMerit = initObj.figureOfMerit
      }
      else {
        this.figureOfMerit = 0.0;
      }
      if (initObj.hasOwnProperty('beamDistance')) {
        this.beamDistance = initObj.beamDistance
      }
      else {
        this.beamDistance = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('batteryVoltage')) {
        this.batteryVoltage = initObj.batteryVoltage
      }
      else {
        this.batteryVoltage = 0.0;
      }
      if (initObj.hasOwnProperty('speedSound')) {
        this.speedSound = initObj.speedSound
      }
      else {
        this.speedSound = 0.0;
      }
      if (initObj.hasOwnProperty('pressure')) {
        this.pressure = initObj.pressure
      }
      else {
        this.pressure = 0.0;
      }
      if (initObj.hasOwnProperty('temp')) {
        this.temp = initObj.temp
      }
      else {
        this.temp = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Dvl
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.float64(obj.time, buffer, bufferOffset);
    // Serialize message field [dt1]
    bufferOffset = _serializer.float64(obj.dt1, buffer, bufferOffset);
    // Serialize message field [dt2]
    bufferOffset = _serializer.float64(obj.dt2, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity, buffer, bufferOffset);
    // Serialize message field [figureOfMerit]
    bufferOffset = _serializer.float64(obj.figureOfMerit, buffer, bufferOffset);
    // Check that the constant length array field [beamDistance] has the right length
    if (obj.beamDistance.length !== 4) {
      throw new Error('Unable to serialize array field beamDistance - length must be 4')
    }
    // Serialize message field [beamDistance]
    bufferOffset = _arraySerializer.float64(obj.beamDistance, buffer, bufferOffset, 4);
    // Serialize message field [batteryVoltage]
    bufferOffset = _serializer.float64(obj.batteryVoltage, buffer, bufferOffset);
    // Serialize message field [speedSound]
    bufferOffset = _serializer.float64(obj.speedSound, buffer, bufferOffset);
    // Serialize message field [pressure]
    bufferOffset = _serializer.float64(obj.pressure, buffer, bufferOffset);
    // Serialize message field [temp]
    bufferOffset = _serializer.float64(obj.temp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Dvl
    let len;
    let data = new Dvl(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dt1]
    data.dt1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dt2]
    data.dt2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [figureOfMerit]
    data.figureOfMerit = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [beamDistance]
    data.beamDistance = _arrayDeserializer.float64(buffer, bufferOffset, 4)
    // Deserialize message field [batteryVoltage]
    data.batteryVoltage = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [speedSound]
    data.speedSound = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pressure]
    data.pressure = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [temp]
    data.temp = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 120;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nortek_dvl/Dvl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b1ec29ea990029dc78359fd83b508011';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 time    #ping time
    float64 dt1     #time from trigger to center of water track cell
    float64 dt2     #time from start of output message to center of water track cell
    geometry_msgs/Vector3 velocity
    float64 figureOfMerit
    float64[4] beamDistance
    float64 batteryVoltage
    float64 speedSound
    float64 pressure
    float64 temp
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
    const resolved = new Dvl(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0.0
    }

    if (msg.dt1 !== undefined) {
      resolved.dt1 = msg.dt1;
    }
    else {
      resolved.dt1 = 0.0
    }

    if (msg.dt2 !== undefined) {
      resolved.dt2 = msg.dt2;
    }
    else {
      resolved.dt2 = 0.0
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = geometry_msgs.msg.Vector3.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new geometry_msgs.msg.Vector3()
    }

    if (msg.figureOfMerit !== undefined) {
      resolved.figureOfMerit = msg.figureOfMerit;
    }
    else {
      resolved.figureOfMerit = 0.0
    }

    if (msg.beamDistance !== undefined) {
      resolved.beamDistance = msg.beamDistance;
    }
    else {
      resolved.beamDistance = new Array(4).fill(0)
    }

    if (msg.batteryVoltage !== undefined) {
      resolved.batteryVoltage = msg.batteryVoltage;
    }
    else {
      resolved.batteryVoltage = 0.0
    }

    if (msg.speedSound !== undefined) {
      resolved.speedSound = msg.speedSound;
    }
    else {
      resolved.speedSound = 0.0
    }

    if (msg.pressure !== undefined) {
      resolved.pressure = msg.pressure;
    }
    else {
      resolved.pressure = 0.0
    }

    if (msg.temp !== undefined) {
      resolved.temp = msg.temp;
    }
    else {
      resolved.temp = 0.0
    }

    return resolved;
    }
};

module.exports = Dvl;
