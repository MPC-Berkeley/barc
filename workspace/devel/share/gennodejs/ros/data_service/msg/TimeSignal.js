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

class TimeSignal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.timestamps = null;
      this.signal = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('timestamps')) {
        this.timestamps = initObj.timestamps
      }
      else {
        this.timestamps = [];
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
    // Serializes a message object of type TimeSignal
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [timestamps]
    bufferOffset = _arraySerializer.float64(obj.timestamps, buffer, bufferOffset, null);
    // Serialize message field [signal]
    bufferOffset = _serializer.string(obj.signal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TimeSignal
    let len;
    let data = new TimeSignal(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [timestamps]
    data.timestamps = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [signal]
    data.signal = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.name.length;
    length += 8 * object.timestamps.length;
    length += object.signal.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'data_service/TimeSignal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '242915a951390ccd66bdffda0979a29d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string name
    float64[] timestamps
    string signal
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TimeSignal(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.timestamps !== undefined) {
      resolved.timestamps = msg.timestamps;
    }
    else {
      resolved.timestamps = []
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

module.exports = TimeSignal;
