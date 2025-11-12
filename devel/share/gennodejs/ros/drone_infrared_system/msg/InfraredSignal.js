// Auto-generated. Do not edit!

// (in-package drone_infrared_system.msg)


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

class InfraredSignal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.robot_name = null;
      this.transmission_power = null;
      this.wavelength = null;
      this.source_pose = null;
      this.received_power = null;
      this.estimated_distance = null;
      this.emitter_id = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('robot_name')) {
        this.robot_name = initObj.robot_name
      }
      else {
        this.robot_name = '';
      }
      if (initObj.hasOwnProperty('transmission_power')) {
        this.transmission_power = initObj.transmission_power
      }
      else {
        this.transmission_power = 0.0;
      }
      if (initObj.hasOwnProperty('wavelength')) {
        this.wavelength = initObj.wavelength
      }
      else {
        this.wavelength = 0.0;
      }
      if (initObj.hasOwnProperty('source_pose')) {
        this.source_pose = initObj.source_pose
      }
      else {
        this.source_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('received_power')) {
        this.received_power = initObj.received_power
      }
      else {
        this.received_power = 0.0;
      }
      if (initObj.hasOwnProperty('estimated_distance')) {
        this.estimated_distance = initObj.estimated_distance
      }
      else {
        this.estimated_distance = 0.0;
      }
      if (initObj.hasOwnProperty('emitter_id')) {
        this.emitter_id = initObj.emitter_id
      }
      else {
        this.emitter_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InfraredSignal
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [robot_name]
    bufferOffset = _serializer.string(obj.robot_name, buffer, bufferOffset);
    // Serialize message field [transmission_power]
    bufferOffset = _serializer.float64(obj.transmission_power, buffer, bufferOffset);
    // Serialize message field [wavelength]
    bufferOffset = _serializer.float64(obj.wavelength, buffer, bufferOffset);
    // Serialize message field [source_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.source_pose, buffer, bufferOffset);
    // Serialize message field [received_power]
    bufferOffset = _serializer.float64(obj.received_power, buffer, bufferOffset);
    // Serialize message field [estimated_distance]
    bufferOffset = _serializer.float64(obj.estimated_distance, buffer, bufferOffset);
    // Serialize message field [emitter_id]
    bufferOffset = _serializer.string(obj.emitter_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InfraredSignal
    let len;
    let data = new InfraredSignal(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [robot_name]
    data.robot_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [transmission_power]
    data.transmission_power = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [wavelength]
    data.wavelength = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [source_pose]
    data.source_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [received_power]
    data.received_power = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [estimated_distance]
    data.estimated_distance = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [emitter_id]
    data.emitter_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.robot_name.length;
    length += object.emitter_id.length;
    return length + 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone_infrared_system/InfraredSignal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '229d00d508a2c64dc70d2cc763410fb1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string robot_name
    float64 transmission_power
    float64 wavelength
    geometry_msgs/Pose source_pose
    float64 received_power
    float64 estimated_distance
    string emitter_id
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
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InfraredSignal(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.robot_name !== undefined) {
      resolved.robot_name = msg.robot_name;
    }
    else {
      resolved.robot_name = ''
    }

    if (msg.transmission_power !== undefined) {
      resolved.transmission_power = msg.transmission_power;
    }
    else {
      resolved.transmission_power = 0.0
    }

    if (msg.wavelength !== undefined) {
      resolved.wavelength = msg.wavelength;
    }
    else {
      resolved.wavelength = 0.0
    }

    if (msg.source_pose !== undefined) {
      resolved.source_pose = geometry_msgs.msg.Pose.Resolve(msg.source_pose)
    }
    else {
      resolved.source_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.received_power !== undefined) {
      resolved.received_power = msg.received_power;
    }
    else {
      resolved.received_power = 0.0
    }

    if (msg.estimated_distance !== undefined) {
      resolved.estimated_distance = msg.estimated_distance;
    }
    else {
      resolved.estimated_distance = 0.0
    }

    if (msg.emitter_id !== undefined) {
      resolved.emitter_id = msg.emitter_id;
    }
    else {
      resolved.emitter_id = ''
    }

    return resolved;
    }
};

module.exports = InfraredSignal;
