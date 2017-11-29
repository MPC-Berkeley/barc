// Auto-generated. Do not edit!

// (in-package barc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Encoder {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.FL = null;
      this.FR = null;
      this.BL = null;
      this.BR = null;
    }
    else {
      if (initObj.hasOwnProperty('FL')) {
        this.FL = initObj.FL
      }
      else {
        this.FL = 0.0;
      }
      if (initObj.hasOwnProperty('FR')) {
        this.FR = initObj.FR
      }
      else {
        this.FR = 0.0;
      }
      if (initObj.hasOwnProperty('BL')) {
        this.BL = initObj.BL
      }
      else {
        this.BL = 0.0;
      }
      if (initObj.hasOwnProperty('BR')) {
        this.BR = initObj.BR
      }
      else {
        this.BR = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Encoder
    // Serialize message field [FL]
    bufferOffset = _serializer.float32(obj.FL, buffer, bufferOffset);
    // Serialize message field [FR]
    bufferOffset = _serializer.float32(obj.FR, buffer, bufferOffset);
    // Serialize message field [BL]
    bufferOffset = _serializer.float32(obj.BL, buffer, bufferOffset);
    // Serialize message field [BR]
    bufferOffset = _serializer.float32(obj.BR, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Encoder
    let len;
    let data = new Encoder(null);
    // Deserialize message field [FL]
    data.FL = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [FR]
    data.FR = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [BL]
    data.BL = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [BR]
    data.BR = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'barc/Encoder';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '32ae9320a3544d34b8e4e844525161ab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 FL 
    float32 FR
    float32 BL
    float32 BR
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Encoder(null);
    if (msg.FL !== undefined) {
      resolved.FL = msg.FL;
    }
    else {
      resolved.FL = 0.0
    }

    if (msg.FR !== undefined) {
      resolved.FR = msg.FR;
    }
    else {
      resolved.FR = 0.0
    }

    if (msg.BL !== undefined) {
      resolved.BL = msg.BL;
    }
    else {
      resolved.BL = 0.0
    }

    if (msg.BR !== undefined) {
      resolved.BR = msg.BR;
    }
    else {
      resolved.BR = 0.0
    }

    return resolved;
    }
};

module.exports = Encoder;
