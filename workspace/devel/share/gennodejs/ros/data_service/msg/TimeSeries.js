// Auto-generated. Do not edit!

// (in-package data_service.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TimeData = require('./TimeData.js');

//-----------------------------------------------------------

class TimeSeries {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.series = null;
    }
    else {
      if (initObj.hasOwnProperty('series')) {
        this.series = initObj.series
      }
      else {
        this.series = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TimeSeries
    // Serialize message field [series]
    // Serialize the length for message field [series]
    bufferOffset = _serializer.uint32(obj.series.length, buffer, bufferOffset);
    obj.series.forEach((val) => {
      bufferOffset = TimeData.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TimeSeries
    let len;
    let data = new TimeSeries(null);
    // Deserialize message field [series]
    // Deserialize array length for message field [series]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.series = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.series[i] = TimeData.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.series.forEach((val) => {
      length += TimeData.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'data_service/TimeSeries';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ecf556e37ef3615e6c95590390588636';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    TimeData[] series
    
    ================================================================================
    MSG: data_service/TimeData
    float64 timestamp
    float64[] value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TimeSeries(null);
    if (msg.series !== undefined) {
      resolved.series = new Array(msg.series.length);
      for (let i = 0; i < resolved.series.length; ++i) {
        resolved.series[i] = TimeData.Resolve(msg.series[i]);
      }
    }
    else {
      resolved.series = []
    }

    return resolved;
    }
};

module.exports = TimeSeries;
