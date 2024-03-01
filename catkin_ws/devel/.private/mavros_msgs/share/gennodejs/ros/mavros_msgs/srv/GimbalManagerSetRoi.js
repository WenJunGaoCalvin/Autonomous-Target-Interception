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

class GimbalManagerSetRoiRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.gimbal_device_id = null;
      this.latitude = null;
      this.longitude = null;
      this.altitude = null;
      this.pitch_offset = null;
      this.roll_offset = null;
      this.yaw_offset = null;
      this.sysid = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('gimbal_device_id')) {
        this.gimbal_device_id = initObj.gimbal_device_id
      }
      else {
        this.gimbal_device_id = 0;
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_offset')) {
        this.pitch_offset = initObj.pitch_offset
      }
      else {
        this.pitch_offset = 0.0;
      }
      if (initObj.hasOwnProperty('roll_offset')) {
        this.roll_offset = initObj.roll_offset
      }
      else {
        this.roll_offset = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_offset')) {
        this.yaw_offset = initObj.yaw_offset
      }
      else {
        this.yaw_offset = 0.0;
      }
      if (initObj.hasOwnProperty('sysid')) {
        this.sysid = initObj.sysid
      }
      else {
        this.sysid = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GimbalManagerSetRoiRequest
    // Serialize message field [mode]
    bufferOffset = _serializer.uint8(obj.mode, buffer, bufferOffset);
    // Serialize message field [gimbal_device_id]
    bufferOffset = _serializer.uint8(obj.gimbal_device_id, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float32(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float32(obj.longitude, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float32(obj.altitude, buffer, bufferOffset);
    // Serialize message field [pitch_offset]
    bufferOffset = _serializer.float32(obj.pitch_offset, buffer, bufferOffset);
    // Serialize message field [roll_offset]
    bufferOffset = _serializer.float32(obj.roll_offset, buffer, bufferOffset);
    // Serialize message field [yaw_offset]
    bufferOffset = _serializer.float32(obj.yaw_offset, buffer, bufferOffset);
    // Serialize message field [sysid]
    bufferOffset = _serializer.uint8(obj.sysid, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GimbalManagerSetRoiRequest
    let len;
    let data = new GimbalManagerSetRoiRequest(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [gimbal_device_id]
    data.gimbal_device_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_offset]
    data.pitch_offset = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll_offset]
    data.roll_offset = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw_offset]
    data.yaw_offset = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [sysid]
    data.sysid = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 27;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mavros_msgs/GimbalManagerSetRoiRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f789d901bb3e0753c3a47a0c94b4993f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MAVLink commands: DO_SET_ROI_LOCATION, DO_SET_ROI_WPNEXT_OFFSET, DO_SET_ROI_SYSID, DO_SET_ROI_NONE
    # https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_LOCATION
    # https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET
    # https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_SYSID
    # https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_NONE
    
    uint8 mode              # enumerator to indicate ROI mode setting - see ROI_MODE
    #ROI_MODE
    uint8 ROI_MODE_LOCATION = 0         # Sets the region of interest (ROI) to a location. [DO_SET_ROI_LOCATION]
    uint8 ROI_MODE_WP_NEXT_OFFSET = 1   # Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. [DO_SET_ROI_WPNEXT_OFFSET]
    uint8 ROI_MODE_SYSID = 2            # Mount tracks system with specified system ID [DO_SET_ROI_SYSID]
    uint8 ROI_MODE_NONE = 3             # Cancels any previous ROI setting and returns vehicle to defaults [DO_SET_ROI_NONE]
    
    uint8 gimbal_device_id  # Component ID of gimbal device to address 
                            # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
                            # components. Send command multiple times for more than
                            # one gimbal (but not all gimbals).  Default Mavlink gimbal 
                            # device ids: 154, 171-175
    
    #For ROI_MODE_LOCATION
    float32 latitude
    float32 longitude
    float32 altitude        # Meters
    
    #For ROI_MODE_WP_NEXT_OFFSET
    float32 pitch_offset    # Pitch offset from next waypoint, positive pitching up
    float32 roll_offset     # Roll offset from next waypoint, positive rolling to the right
    float32 yaw_offset      # Yaw offset from next waypoint, positive yawing to the right
    
    #For ROI_MODE_SYSID
    uint8 sysid             # System ID to track (min: 1, max: 255)
    
    #ROI_MODE_NONE doesn't take extra parameters
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GimbalManagerSetRoiRequest(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.gimbal_device_id !== undefined) {
      resolved.gimbal_device_id = msg.gimbal_device_id;
    }
    else {
      resolved.gimbal_device_id = 0
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    if (msg.pitch_offset !== undefined) {
      resolved.pitch_offset = msg.pitch_offset;
    }
    else {
      resolved.pitch_offset = 0.0
    }

    if (msg.roll_offset !== undefined) {
      resolved.roll_offset = msg.roll_offset;
    }
    else {
      resolved.roll_offset = 0.0
    }

    if (msg.yaw_offset !== undefined) {
      resolved.yaw_offset = msg.yaw_offset;
    }
    else {
      resolved.yaw_offset = 0.0
    }

    if (msg.sysid !== undefined) {
      resolved.sysid = msg.sysid;
    }
    else {
      resolved.sysid = 0
    }

    return resolved;
    }
};

// Constants for message
GimbalManagerSetRoiRequest.Constants = {
  ROI_MODE_LOCATION: 0,
  ROI_MODE_WP_NEXT_OFFSET: 1,
  ROI_MODE_SYSID: 2,
  ROI_MODE_NONE: 3,
}

class GimbalManagerSetRoiResponse {
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
    // Serializes a message object of type GimbalManagerSetRoiResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [result]
    bufferOffset = _serializer.uint8(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GimbalManagerSetRoiResponse
    let len;
    let data = new GimbalManagerSetRoiResponse(null);
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
    return 'mavros_msgs/GimbalManagerSetRoiResponse';
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
    const resolved = new GimbalManagerSetRoiResponse(null);
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
  Request: GimbalManagerSetRoiRequest,
  Response: GimbalManagerSetRoiResponse,
  md5sum() { return '0796c6a6cf0e18b0d829681b29f0828a'; },
  datatype() { return 'mavros_msgs/GimbalManagerSetRoi'; }
};
