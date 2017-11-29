// Auto-generated. Do not edit!

// (in-package data_service.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TimeSignal = require('../msg/TimeSignal.js');
let CustomSignal = require('../msg/CustomSignal.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class DataForwardRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time_signal = null;
      this.custom_signal = null;
      this.experiment_name = null;
    }
    else {
      if (initObj.hasOwnProperty('time_signal')) {
        this.time_signal = initObj.time_signal
      }
      else {
        this.time_signal = new TimeSignal();
      }
      if (initObj.hasOwnProperty('custom_signal')) {
        this.custom_signal = initObj.custom_signal
      }
      else {
        this.custom_signal = new CustomSignal();
      }
      if (initObj.hasOwnProperty('experiment_name')) {
        this.experiment_name = initObj.experiment_name
      }
      else {
        this.experiment_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DataForwardRequest
    // Serialize message field [time_signal]
    bufferOffset = TimeSignal.serialize(obj.time_signal, buffer, bufferOffset);
    // Serialize message field [custom_signal]
    bufferOffset = CustomSignal.serialize(obj.custom_signal, buffer, bufferOffset);
    // Serialize message field [experiment_name]
    bufferOffset = _serializer.string(obj.experiment_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DataForwardRequest
    let len;
    let data = new DataForwardRequest(null);
    // Deserialize message field [time_signal]
    data.time_signal = TimeSignal.deserialize(buffer, bufferOffset);
    // Deserialize message field [custom_signal]
    data.custom_signal = CustomSignal.deserialize(buffer, bufferOffset);
    // Deserialize message field [experiment_name]
    data.experiment_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += TimeSignal.getMessageSize(object.time_signal);
    length += CustomSignal.getMessageSize(object.custom_signal);
    length += object.experiment_name.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'data_service/DataForwardRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '66879b37a6a1d948514d4b887d4a03a9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    TimeSignal time_signal
    CustomSignal custom_signal
    string experiment_name
    
    ================================================================================
    MSG: data_service/TimeSignal
    string name
    float64[] timestamps
    string signal
    
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
    const resolved = new DataForwardRequest(null);
    if (msg.time_signal !== undefined) {
      resolved.time_signal = TimeSignal.Resolve(msg.time_signal)
    }
    else {
      resolved.time_signal = new TimeSignal()
    }

    if (msg.custom_signal !== undefined) {
      resolved.custom_signal = CustomSignal.Resolve(msg.custom_signal)
    }
    else {
      resolved.custom_signal = new CustomSignal()
    }

    if (msg.experiment_name !== undefined) {
      resolved.experiment_name = msg.experiment_name;
    }
    else {
      resolved.experiment_name = ''
    }

    return resolved;
    }
};

class DataForwardResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.response = null;
    }
    else {
      if (initObj.hasOwnProperty('response')) {
        this.response = initObj.response
      }
      else {
        this.response = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DataForwardResponse
    // Serialize message field [response]
    bufferOffset = _serializer.string(obj.response, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DataForwardResponse
    let len;
    let data = new DataForwardResponse(null);
    // Deserialize message field [response]
    data.response = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.response.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'data_service/DataForwardResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6de314e2dc76fbff2b6244a6ad70b68d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string response
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DataForwardResponse(null);
    if (msg.response !== undefined) {
      resolved.response = msg.response;
    }
    else {
      resolved.response = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: DataForwardRequest,
  Response: DataForwardResponse,
  md5sum() { return 'f8d627aa29376505cccbe1058f0ed9d2'; },
  datatype() { return 'data_service/DataForward'; }
};
