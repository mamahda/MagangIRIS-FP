// Auto-generated. Do not edit!

// (in-package FP_Magang.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PC2BS {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bola_x = null;
      this.bola_y = null;
      this.motor1 = null;
      this.motor2 = null;
      this.motor3 = null;
    }
    else {
      if (initObj.hasOwnProperty('bola_x')) {
        this.bola_x = initObj.bola_x
      }
      else {
        this.bola_x = 0.0;
      }
      if (initObj.hasOwnProperty('bola_y')) {
        this.bola_y = initObj.bola_y
      }
      else {
        this.bola_y = 0.0;
      }
      if (initObj.hasOwnProperty('motor1')) {
        this.motor1 = initObj.motor1
      }
      else {
        this.motor1 = 0.0;
      }
      if (initObj.hasOwnProperty('motor2')) {
        this.motor2 = initObj.motor2
      }
      else {
        this.motor2 = 0.0;
      }
      if (initObj.hasOwnProperty('motor3')) {
        this.motor3 = initObj.motor3
      }
      else {
        this.motor3 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PC2BS
    // Serialize message field [bola_x]
    bufferOffset = _serializer.float32(obj.bola_x, buffer, bufferOffset);
    // Serialize message field [bola_y]
    bufferOffset = _serializer.float32(obj.bola_y, buffer, bufferOffset);
    // Serialize message field [motor1]
    bufferOffset = _serializer.float32(obj.motor1, buffer, bufferOffset);
    // Serialize message field [motor2]
    bufferOffset = _serializer.float32(obj.motor2, buffer, bufferOffset);
    // Serialize message field [motor3]
    bufferOffset = _serializer.float32(obj.motor3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PC2BS
    let len;
    let data = new PC2BS(null);
    // Deserialize message field [bola_x]
    data.bola_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bola_y]
    data.bola_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [motor1]
    data.motor1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [motor2]
    data.motor2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [motor3]
    data.motor3 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'FP_Magang/PC2BS';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '414e07ab141b8d50059180f242899e66';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 bola_x
    float32 bola_y
    float32 motor1
    float32 motor2
    float32 motor3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PC2BS(null);
    if (msg.bola_x !== undefined) {
      resolved.bola_x = msg.bola_x;
    }
    else {
      resolved.bola_x = 0.0
    }

    if (msg.bola_y !== undefined) {
      resolved.bola_y = msg.bola_y;
    }
    else {
      resolved.bola_y = 0.0
    }

    if (msg.motor1 !== undefined) {
      resolved.motor1 = msg.motor1;
    }
    else {
      resolved.motor1 = 0.0
    }

    if (msg.motor2 !== undefined) {
      resolved.motor2 = msg.motor2;
    }
    else {
      resolved.motor2 = 0.0
    }

    if (msg.motor3 !== undefined) {
      resolved.motor3 = msg.motor3;
    }
    else {
      resolved.motor3 = 0.0
    }

    return resolved;
    }
};

module.exports = PC2BS;
