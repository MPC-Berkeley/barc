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

class ECU {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.motor = null;
      this.servo = null;
    }
    else {
      if (initObj.hasOwnProperty('motor')) {
        this.motor = initObj.motor
      }
      else {
        this.motor = 0.0;
      }
      if (initObj.hasOwnProperty('servo')) {
        this.servo = initObj.servo
      }
      else {
        this.servo = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ECU
    // Serialize message field [motor]
    bufferOffset = _serializer.float32(obj.motor, buffer, bufferOffset);
    // Serialize message field [servo]
    bufferOffset = _serializer.float32(obj.servo, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ECU
    let len;
    let data = new ECU(null);
    // Deserialize message field [motor]
    data.motor = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [servo]
    data.servo = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'barc/ECU';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e60fd3690167c0df782fc50cceb5ce82';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This is a message to hold data for the ECU (electronic control unit)
    #
    # Units may vary depending on the topic
    # The motor controls the speeds of the vehicle through an input torque. (For input force, divide by radius of tire) 
    # The servo controls the steering angle
    #
    # For modeling and state estimation, motors units are [N], and servo units are [rad]
    # For actuator signals, both have units of PWM angle [deg]. This relates to the duty cycle
    float32 motor 
    float32 servo
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ECU(null);
    if (msg.motor !== undefined) {
      resolved.motor = msg.motor;
    }
    else {
      resolved.motor = 0.0
    }

    if (msg.servo !== undefined) {
      resolved.servo = msg.servo;
    }
    else {
      resolved.servo = 0.0
    }

    return resolved;
    }
};

module.exports = ECU;
