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

class RegisterExperimentRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.experiment = null;
    }
    else {
      if (initObj.hasOwnProperty('experiment')) {
        this.experiment = initObj.experiment
      }
      else {
        this.experiment = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RegisterExperimentRequest
    // Serialize message field [experiment]
    bufferOffset = _serializer.string(obj.experiment, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RegisterExperimentRequest
    let len;
    let data = new RegisterExperimentRequest(null);
    // Deserialize message field [experiment]
    data.experiment = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.experiment.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'data_service/RegisterExperimentRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd5af7b7e6b332ce1133c33bfeadc80af';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string experiment
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RegisterExperimentRequest(null);
    if (msg.experiment !== undefined) {
      resolved.experiment = msg.experiment;
    }
    else {
      resolved.experiment = ''
    }

    return resolved;
    }
};

class RegisterExperimentResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.experiment_id = null;
    }
    else {
      if (initObj.hasOwnProperty('experiment_id')) {
        this.experiment_id = initObj.experiment_id
      }
      else {
        this.experiment_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RegisterExperimentResponse
    // Serialize message field [experiment_id]
    bufferOffset = _serializer.int32(obj.experiment_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RegisterExperimentResponse
    let len;
    let data = new RegisterExperimentResponse(null);
    // Deserialize message field [experiment_id]
    data.experiment_id = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'data_service/RegisterExperimentResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9bc044b3b8998aa44c7343356570fb87';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 experiment_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RegisterExperimentResponse(null);
    if (msg.experiment_id !== undefined) {
      resolved.experiment_id = msg.experiment_id;
    }
    else {
      resolved.experiment_id = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: RegisterExperimentRequest,
  Response: RegisterExperimentResponse,
  md5sum() { return '23efeb8a860ffcd264ebd586a8c57078'; },
  datatype() { return 'data_service/RegisterExperiment'; }
};
