// Auto-generated. Do not edit!

// (in-package iris_ir_sitl.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class InfraredDetection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.receiver_id = null;
      this.detected_ids = null;
      this.distances = null;
      this.strengths = null;
      this.timestamp = null;
    }
    else {
      if (initObj.hasOwnProperty('receiver_id')) {
        this.receiver_id = initObj.receiver_id
      }
      else {
        this.receiver_id = '';
      }
      if (initObj.hasOwnProperty('detected_ids')) {
        this.detected_ids = initObj.detected_ids
      }
      else {
        this.detected_ids = [];
      }
      if (initObj.hasOwnProperty('distances')) {
        this.distances = initObj.distances
      }
      else {
        this.distances = [];
      }
      if (initObj.hasOwnProperty('strengths')) {
        this.strengths = initObj.strengths
      }
      else {
        this.strengths = [];
      }
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InfraredDetection
    // Serialize message field [receiver_id]
    bufferOffset = _serializer.string(obj.receiver_id, buffer, bufferOffset);
    // Serialize message field [detected_ids]
    bufferOffset = _arraySerializer.string(obj.detected_ids, buffer, bufferOffset, null);
    // Serialize message field [distances]
    bufferOffset = _arraySerializer.float64(obj.distances, buffer, bufferOffset, null);
    // Serialize message field [strengths]
    bufferOffset = _arraySerializer.float64(obj.strengths, buffer, bufferOffset, null);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InfraredDetection
    let len;
    let data = new InfraredDetection(null);
    // Deserialize message field [receiver_id]
    data.receiver_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [detected_ids]
    data.detected_ids = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [distances]
    data.distances = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [strengths]
    data.strengths = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.receiver_id.length;
    object.detected_ids.forEach((val) => {
      length += 4 + val.length;
    });
    length += 8 * object.distances.length;
    length += 8 * object.strengths.length;
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'iris_ir_sitl/InfraredDetection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '16244eb9ef16a053d5fdf4543baba6bc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string receiver_id
    string[] detected_ids
    float64[] distances
    float64[] strengths
    float64 timestamp
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InfraredDetection(null);
    if (msg.receiver_id !== undefined) {
      resolved.receiver_id = msg.receiver_id;
    }
    else {
      resolved.receiver_id = ''
    }

    if (msg.detected_ids !== undefined) {
      resolved.detected_ids = msg.detected_ids;
    }
    else {
      resolved.detected_ids = []
    }

    if (msg.distances !== undefined) {
      resolved.distances = msg.distances;
    }
    else {
      resolved.distances = []
    }

    if (msg.strengths !== undefined) {
      resolved.strengths = msg.strengths;
    }
    else {
      resolved.strengths = []
    }

    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    return resolved;
    }
};

module.exports = InfraredDetection;
