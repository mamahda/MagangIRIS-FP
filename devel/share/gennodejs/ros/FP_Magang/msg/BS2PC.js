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

class BS2PC {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
      this.tujuan_x = null;
      this.tujuan_y = null;
      this.enc_left = null;
      this.enc_right = null;
      this.th = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0.0;
      }
      if (initObj.hasOwnProperty('tujuan_x')) {
        this.tujuan_x = initObj.tujuan_x
      }
      else {
        this.tujuan_x = 0.0;
      }
      if (initObj.hasOwnProperty('tujuan_y')) {
        this.tujuan_y = initObj.tujuan_y
      }
      else {
        this.tujuan_y = 0.0;
      }
      if (initObj.hasOwnProperty('enc_left')) {
        this.enc_left = initObj.enc_left
      }
      else {
        this.enc_left = 0.0;
      }
      if (initObj.hasOwnProperty('enc_right')) {
        this.enc_right = initObj.enc_right
      }
      else {
        this.enc_right = 0.0;
      }
      if (initObj.hasOwnProperty('th')) {
        this.th = initObj.th
      }
      else {
        this.th = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BS2PC
    // Serialize message field [status]
    bufferOffset = _serializer.float32(obj.status, buffer, bufferOffset);
    // Serialize message field [tujuan_x]
    bufferOffset = _serializer.float32(obj.tujuan_x, buffer, bufferOffset);
    // Serialize message field [tujuan_y]
    bufferOffset = _serializer.float32(obj.tujuan_y, buffer, bufferOffset);
    // Serialize message field [enc_left]
    bufferOffset = _serializer.float32(obj.enc_left, buffer, bufferOffset);
    // Serialize message field [enc_right]
    bufferOffset = _serializer.float32(obj.enc_right, buffer, bufferOffset);
    // Serialize message field [th]
    bufferOffset = _serializer.float32(obj.th, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BS2PC
    let len;
    let data = new BS2PC(null);
    // Deserialize message field [status]
    data.status = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tujuan_x]
    data.tujuan_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tujuan_y]
    data.tujuan_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [enc_left]
    data.enc_left = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [enc_right]
    data.enc_right = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [th]
    data.th = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'FP_Magang/BS2PC';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6713ba3a23152bdfb17a6636f933f589';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 status
    float32 tujuan_x
    float32 tujuan_y
    float32 enc_left
    float32 enc_right
    float32 th
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BS2PC(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0.0
    }

    if (msg.tujuan_x !== undefined) {
      resolved.tujuan_x = msg.tujuan_x;
    }
    else {
      resolved.tujuan_x = 0.0
    }

    if (msg.tujuan_y !== undefined) {
      resolved.tujuan_y = msg.tujuan_y;
    }
    else {
      resolved.tujuan_y = 0.0
    }

    if (msg.enc_left !== undefined) {
      resolved.enc_left = msg.enc_left;
    }
    else {
      resolved.enc_left = 0.0
    }

    if (msg.enc_right !== undefined) {
      resolved.enc_right = msg.enc_right;
    }
    else {
      resolved.enc_right = 0.0
    }

    if (msg.th !== undefined) {
      resolved.th = msg.th;
    }
    else {
      resolved.th = 0.0
    }

    return resolved;
    }
};

module.exports = BS2PC;
