// Auto-generated. Do not edit!

// (in-package simulator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class eZ_DynBkMdl {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.s = null;
      this.ey = null;
      this.epsi = null;
      this.v_x = null;
      this.v_y = null;
      this.psi_dot = null;
    }
    else {
      if (initObj.hasOwnProperty('s')) {
        this.s = initObj.s
      }
      else {
        this.s = 0.0;
      }
      if (initObj.hasOwnProperty('ey')) {
        this.ey = initObj.ey
      }
      else {
        this.ey = 0.0;
      }
      if (initObj.hasOwnProperty('epsi')) {
        this.epsi = initObj.epsi
      }
      else {
        this.epsi = 0.0;
      }
      if (initObj.hasOwnProperty('v_x')) {
        this.v_x = initObj.v_x
      }
      else {
        this.v_x = 0.0;
      }
      if (initObj.hasOwnProperty('v_y')) {
        this.v_y = initObj.v_y
      }
      else {
        this.v_y = 0.0;
      }
      if (initObj.hasOwnProperty('psi_dot')) {
        this.psi_dot = initObj.psi_dot
      }
      else {
        this.psi_dot = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type eZ_DynBkMdl
    // Serialize message field [s]
    bufferOffset = _serializer.float32(obj.s, buffer, bufferOffset);
    // Serialize message field [ey]
    bufferOffset = _serializer.float32(obj.ey, buffer, bufferOffset);
    // Serialize message field [epsi]
    bufferOffset = _serializer.float32(obj.epsi, buffer, bufferOffset);
    // Serialize message field [v_x]
    bufferOffset = _serializer.float32(obj.v_x, buffer, bufferOffset);
    // Serialize message field [v_y]
    bufferOffset = _serializer.float32(obj.v_y, buffer, bufferOffset);
    // Serialize message field [psi_dot]
    bufferOffset = _serializer.float32(obj.psi_dot, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type eZ_DynBkMdl
    let len;
    let data = new eZ_DynBkMdl(null);
    // Deserialize message field [s]
    data.s = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ey]
    data.ey = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [epsi]
    data.epsi = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [v_x]
    data.v_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [v_y]
    data.v_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [psi_dot]
    data.psi_dot = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'simulator/eZ_DynBkMdl';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '19de479b90a68da73ee59a1fb6a50755';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 s
    float32 ey
    float32 epsi
    float32 v_x
    float32 v_y
    float32 psi_dot
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new eZ_DynBkMdl(null);
    if (msg.s !== undefined) {
      resolved.s = msg.s;
    }
    else {
      resolved.s = 0.0
    }

    if (msg.ey !== undefined) {
      resolved.ey = msg.ey;
    }
    else {
      resolved.ey = 0.0
    }

    if (msg.epsi !== undefined) {
      resolved.epsi = msg.epsi;
    }
    else {
      resolved.epsi = 0.0
    }

    if (msg.v_x !== undefined) {
      resolved.v_x = msg.v_x;
    }
    else {
      resolved.v_x = 0.0
    }

    if (msg.v_y !== undefined) {
      resolved.v_y = msg.v_y;
    }
    else {
      resolved.v_y = 0.0
    }

    if (msg.psi_dot !== undefined) {
      resolved.psi_dot = msg.psi_dot;
    }
    else {
      resolved.psi_dot = 0.0
    }

    return resolved;
    }
};

module.exports = eZ_DynBkMdl;
