// Auto-generated. Do not edit!

// (in-package data_service.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CustomSignal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.signal = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = '';
      }
      if (initObj.hasOwnProperty('signal')) {
        this.signal = initObj.signal
      }
      else {
        this.signal = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CustomSignal
    // Serialize message field [id]
    bufferOffset = _serializer.string(obj.id, buffer, bufferOffset);
    // Serialize message field [signal]
    bufferOffset = _serializer.string(obj.signal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CustomSignal
    let len;
    let data = new CustomSignal(null);
    // Deserialize message field [id]
    data.id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [signal]
    data.signal = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.id.length;
    length += object.signal.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'data_service/CustomSignal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '952635fef7c9f8f6266bdfc127ea52f9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string id
    string signal
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CustomSignal(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = ''
    }

    if (msg.signal !== undefined) {
      resolved.signal = msg.signal;
    }
    else {
      resolved.signal = ''
    }

    return resolved;
    }
};

module.exports = CustomSignal;
