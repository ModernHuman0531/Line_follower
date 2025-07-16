// Auto-generated. Do not edit!

// (in-package lane_follower.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MotorPWM_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left_pwm = null;
      this.right_pwm = null;
    }
    else {
      if (initObj.hasOwnProperty('left_pwm')) {
        this.left_pwm = initObj.left_pwm
      }
      else {
        this.left_pwm = 0;
      }
      if (initObj.hasOwnProperty('right_pwm')) {
        this.right_pwm = initObj.right_pwm
      }
      else {
        this.right_pwm = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotorPWM_msg
    // Serialize message field [left_pwm]
    bufferOffset = _serializer.int16(obj.left_pwm, buffer, bufferOffset);
    // Serialize message field [right_pwm]
    bufferOffset = _serializer.int16(obj.right_pwm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotorPWM_msg
    let len;
    let data = new MotorPWM_msg(null);
    // Deserialize message field [left_pwm]
    data.left_pwm = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [right_pwm]
    data.right_pwm = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lane_follower/MotorPWM_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1bbcb2731ff8485f26f1b76809762437';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 left_pwm
    int16 right_pwm
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotorPWM_msg(null);
    if (msg.left_pwm !== undefined) {
      resolved.left_pwm = msg.left_pwm;
    }
    else {
      resolved.left_pwm = 0
    }

    if (msg.right_pwm !== undefined) {
      resolved.right_pwm = msg.right_pwm;
    }
    else {
      resolved.right_pwm = 0
    }

    return resolved;
    }
};

module.exports = MotorPWM_msg;
