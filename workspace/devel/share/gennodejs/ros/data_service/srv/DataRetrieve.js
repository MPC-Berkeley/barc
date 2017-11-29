// Auto-generated. Do not edit!

// (in-package data_service.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let CustomSignal = require('../msg/CustomSignal.js');

//-----------------------------------------------------------

class DataRetrieveRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.is_time = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = '';
      }
      if (initObj.hasOwnProperty('is_time')) {
        this.is_time = initObj.is_time
      }
      else {
        this.is_time = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DataRetrieveRequest
    // Serialize message field [id]
    bufferOffset = _serializer.string(obj.id, buffer, bufferOffset);
    // Serialize message field [is_time]
    bufferOffset = _serializer.bool(obj.is_time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DataRetrieveRequest
    let len;
    let data = new DataRetrieveRequest(null);
    // Deserialize message field [id]
    data.id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [is_time]
    data.is_time = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.id.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'data_service/DataRetrieveRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '75d582623a3182fcdab95d52c5168aba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string id
    bool is_time
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DataRetrieveRequest(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = ''
    }

    if (msg.is_time !== undefined) {
      resolved.is_time = msg.is_time;
    }
    else {
      resolved.is_time = false
    }

    return resolved;
    }
};

class DataRetrieveResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.signal = null;
    }
    else {
      if (initObj.hasOwnProperty('signal')) {
        this.signal = initObj.signal
      }
      else {
        this.signal = new CustomSignal();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DataRetrieveResponse
    // Serialize message field [signal]
    bufferOffset = CustomSignal.serialize(obj.signal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DataRetrieveResponse
    let len;
    let data = new DataRetrieveResponse(null);
    // Deserialize message field [signal]
    data.signal = CustomSignal.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += CustomSignal.getMessageSize(object.signal);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'data_service/DataRetrieveResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ed8e16fd675a2c47988d9dbfca608537';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CustomSignal signal
    
    ================================================================================
    MSG: data_service/CustomSignal
    string id
    string signal
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DataRetrieveResponse(null);
    if (msg.signal !== undefined) {
      resolved.signal = CustomSignal.Resolve(msg.signal)
    }
    else {
      resolved.signal = new CustomSignal()
    }

    return resolved;
    }
};

module.exports = {
  Request: DataRetrieveRequest,
  Response: DataRetrieveResponse,
  md5sum() { return 'f58abbeb8edc41005a341386540c1d0b'; },
  datatype() { return 'data_service/DataRetrieve'; }
};
