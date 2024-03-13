// Auto-generated. Do not edit!

// (in-package mavros_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class GimbalManagerCameraTrackRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.x = null;
      this.y = null;
      this.radius = null;
      this.top_left_x = null;
      this.top_left_y = null;
      this.bottom_right_x = null;
      this.bottom_right_y = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('radius')) {
        this.radius = initObj.radius
      }
      else {
        this.radius = 0.0;
      }
      if (initObj.hasOwnProperty('top_left_x')) {
        this.top_left_x = initObj.top_left_x
      }
      else {
        this.top_left_x = 0.0;
      }
      if (initObj.hasOwnProperty('top_left_y')) {
        this.top_left_y = initObj.top_left_y
      }
      else {
        this.top_left_y = 0.0;
      }
      if (initObj.hasOwnProperty('bottom_right_x')) {
        this.bottom_right_x = initObj.bottom_right_x
      }
      else {
        this.bottom_right_x = 0.0;
      }
      if (initObj.hasOwnProperty('bottom_right_y')) {
        this.bottom_right_y = initObj.bottom_right_y
      }
      else {
        this.bottom_right_y = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GimbalManagerCameraTrackRequest
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [x]
    bufferOffset = _serializer.float32(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float32(obj.y, buffer, bufferOffset);
    // Serialize message field [radius]
    bufferOffset = _serializer.float32(obj.radius, buffer, bufferOffset);
    // Serialize message field [top_left_x]
    bufferOffset = _serializer.float32(obj.top_left_x, buffer, bufferOffset);
    // Serialize message field [top_left_y]
    bufferOffset = _serializer.float32(obj.top_left_y, buffer, bufferOffset);
    // Serialize message field [bottom_right_x]
    bufferOffset = _serializer.float32(obj.bottom_right_x, buffer, bufferOffset);
    // Serialize message field [bottom_right_y]
    bufferOffset = _serializer.float32(obj.bottom_right_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GimbalManagerCameraTrackRequest
    let len;
    let data = new GimbalManagerCameraTrackRequest(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [x]
    data.x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [radius]
    data.radius = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [top_left_x]
    data.top_left_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [top_left_y]
    data.top_left_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bottom_right_x]
    data.bottom_right_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [bottom_right_y]
    data.bottom_right_y = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 29;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mavros_msgs/GimbalManagerCameraTrackRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ba0d64325a3002df0189bd5ebdeecac8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MAVLink commands: CAMERA_TRACK_POINT, CAMERA_TRACK_RECTANGLE, CAMERA_STOP_TRACKING
    # https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_POINT
    # https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_RECTANGLE
    # https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_STOP_TRACKING
    
    uint8 mode      # enumerator to indicate camera track mode setting - see CAMERA_TRACK_MODE
    #CAMERA_TRACK_MODE
    uint8 CAMERA_TRACK_MODE_POINT = 0           # If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking. [CAMERA_TRACK_POINT]
    uint8 CAMERA_TRACK_MODE_RECTANGLE = 1       # If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking. [CAMERA_TRACK_RECTANGLE]
    uint8 CAMERA_TRACK_MODE_STOP_TRACKING = 2   # Stops ongoing tracking. [CAMERA_STOP_TRACKING]
    
    #For CAMERA_TRACK_POINT
    float32 x       # Point to track x value (normalized 0..1, 0 is left, 1 is right).
    float32 y       # Point to track y value (normalized 0..1, 0 is top, 1 is bottom).
    float32 radius  # Point radius (normalized 0..1, 0 is image left, 1 is image right).
    
    #For CAMERA_TRACK_RECTANGLE
    float32 top_left_x      # Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
    float32 top_left_y      # Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
    float32 bottom_right_x  # Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
    float32 bottom_right_y  # Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
    
    #CAMERA_STOP_TRACKING doesn't take extra parameters
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GimbalManagerCameraTrackRequest(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.radius !== undefined) {
      resolved.radius = msg.radius;
    }
    else {
      resolved.radius = 0.0
    }

    if (msg.top_left_x !== undefined) {
      resolved.top_left_x = msg.top_left_x;
    }
    else {
      resolved.top_left_x = 0.0
    }

    if (msg.top_left_y !== undefined) {
      resolved.top_left_y = msg.top_left_y;
    }
    else {
      resolved.top_left_y = 0.0
    }

    if (msg.bottom_right_x !== undefined) {
      resolved.bottom_right_x = msg.bottom_right_x;
    }
    else {
      resolved.bottom_right_x = 0.0
    }

    if (msg.bottom_right_y !== undefined) {
      resolved.bottom_right_y = msg.bottom_right_y;
    }
    else {
      resolved.bottom_right_y = 0.0
    }

    return resolved;
    }
};

// Constants for message
GimbalManagerCameraTrackRequest.Constants = {
  CAMERA_TRACK_MODE_POINT: 0,
  CAMERA_TRACK_MODE_RECTANGLE: 1,
  CAMERA_TRACK_MODE_STOP_TRACKING: 2,
}

class GimbalManagerCameraTrackResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GimbalManagerCameraTrackResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [result]
    bufferOffset = _serializer.uint8(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GimbalManagerCameraTrackResponse
    let len;
    let data = new GimbalManagerCameraTrackResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [result]
    data.result = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mavros_msgs/GimbalManagerCameraTrackResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1cd894375e4e3d2861d2222772894fdb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    # raw result returned by COMMAND_ACK
    uint8 result
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GimbalManagerCameraTrackResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: GimbalManagerCameraTrackRequest,
  Response: GimbalManagerCameraTrackResponse,
  md5sum() { return 'b75e6fe30849c24c4458c6930214e491'; },
  datatype() { return 'mavros_msgs/GimbalManagerCameraTrack'; }
};
