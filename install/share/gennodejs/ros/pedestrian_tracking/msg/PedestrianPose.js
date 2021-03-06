// Auto-generated. Do not edit!

// (in-package pedestrian_tracking.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PedestrianPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pedID = null;
      this.frameID = null;
      this.x = null;
      this.y = null;
    }
    else {
      if (initObj.hasOwnProperty('pedID')) {
        this.pedID = initObj.pedID
      }
      else {
        this.pedID = 0;
      }
      if (initObj.hasOwnProperty('frameID')) {
        this.frameID = initObj.frameID
      }
      else {
        this.frameID = 0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PedestrianPose
    // Serialize message field [pedID]
    bufferOffset = _serializer.uint32(obj.pedID, buffer, bufferOffset);
    // Serialize message field [frameID]
    bufferOffset = _serializer.uint32(obj.frameID, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PedestrianPose
    let len;
    let data = new PedestrianPose(null);
    // Deserialize message field [pedID]
    data.pedID = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [frameID]
    data.frameID = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pedestrian_tracking/PedestrianPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8c6b22c503c8ea4c695da904ced715cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 pedID
    uint32 frameID
    float64 x
    float64 y
    
    #undecided data type
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PedestrianPose(null);
    if (msg.pedID !== undefined) {
      resolved.pedID = msg.pedID;
    }
    else {
      resolved.pedID = 0
    }

    if (msg.frameID !== undefined) {
      resolved.frameID = msg.frameID;
    }
    else {
      resolved.frameID = 0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    return resolved;
    }
};

module.exports = PedestrianPose;
