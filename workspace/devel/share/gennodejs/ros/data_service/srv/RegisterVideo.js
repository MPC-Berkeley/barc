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


//-----------------------------------------------------------

class RegisterVideoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.experiment = null;
      this.path = null;
    }
    else {
      if (initObj.hasOwnProperty('experiment')) {
        this.experiment = initObj.experiment
      }
      else {
        this.experiment = '';
      }
      if (initObj.hasOwnProperty('path')) {
        this.path = initObj.path
      }
      else {
        this.path = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RegisterVideoRequest
    // Serialize message field [experiment]
    bufferOffset = _serializer.string(obj.experiment, buffer, bufferOffset);
    // Serialize message field [path]
    bufferOffset = _serializer.string(obj.path, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RegisterVideoRequest
    let len;
    let data = new RegisterVideoRequest(null);
    // Deserialize message field [experiment]
    data.experiment = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [path]
    data.path = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.experiment.length;
    length += object.path.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'data_service/RegisterVideoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'baad6537a8fac26f3bc46b1b86efbaac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string experiment
    string path
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RegisterVideoRequest(null);
    if (msg.experiment !== undefined) {
      resolved.experiment = msg.experiment;
    }
    else {
      resolved.experiment = ''
    }

    if (msg.path !== undefined) {
      resolved.path = msg.path;
    }
    else {
      resolved.path = ''
    }

    return resolved;
    }
};

class RegisterVideoResponse {
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
    // Serializes a message object of type RegisterVideoResponse
    // Serialize message field [response]
    bufferOffset = _serializer.string(obj.response, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RegisterVideoResponse
    let len;
    let data = new RegisterVideoResponse(null);
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
    return 'data_service/RegisterVideoResponse';
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
    const resolved = new RegisterVideoResponse(null);
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
  Request: RegisterVideoRequest,
  Response: RegisterVideoResponse,
  md5sum() { return '252bc0f1ff2ebd98b97d212fe2c3fb57'; },
  datatype() { return 'data_service/RegisterVideo'; }
};
