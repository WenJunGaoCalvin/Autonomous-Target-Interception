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

class GimbalManagerConfigureRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sysid_primary = null;
      this.compid_primary = null;
      this.sysid_secondary = null;
      this.compid_secondary = null;
      this.gimbal_device_id = null;
    }
    else {
      if (initObj.hasOwnProperty('sysid_primary')) {
        this.sysid_primary = initObj.sysid_primary
      }
      else {
        this.sysid_primary = 0;
      }
      if (initObj.hasOwnProperty('compid_primary')) {
        this.compid_primary = initObj.compid_primary
      }
      else {
        this.compid_primary = 0;
      }
      if (initObj.hasOwnProperty('sysid_secondary')) {
        this.sysid_secondary = initObj.sysid_secondary
      }
      else {
        this.sysid_secondary = 0;
      }
      if (initObj.hasOwnProperty('compid_secondary')) {
        this.compid_secondary = initObj.compid_secondary
      }
      else {
        this.compid_secondary = 0;
      }
      if (initObj.hasOwnProperty('gimbal_device_id')) {
        this.gimbal_device_id = initObj.gimbal_device_id
      }
      else {
        this.gimbal_device_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GimbalManagerConfigureRequest
    // Serialize message field [sysid_primary]
    bufferOffset = _serializer.int16(obj.sysid_primary, buffer, bufferOffset);
    // Serialize message field [compid_primary]
    bufferOffset = _serializer.int16(obj.compid_primary, buffer, bufferOffset);
    // Serialize message field [sysid_secondary]
    bufferOffset = _serializer.int16(obj.sysid_secondary, buffer, bufferOffset);
    // Serialize message field [compid_secondary]
    bufferOffset = _serializer.int16(obj.compid_secondary, buffer, bufferOffset);
    // Serialize message field [gimbal_device_id]
    bufferOffset = _serializer.uint8(obj.gimbal_device_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GimbalManagerConfigureRequest
    let len;
    let data = new GimbalManagerConfigureRequest(null);
    // Deserialize message field [sysid_primary]
    data.sysid_primary = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [compid_primary]
    data.compid_primary = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [sysid_secondary]
    data.sysid_secondary = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [compid_secondary]
    data.compid_secondary = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [gimbal_device_id]
    data.gimbal_device_id = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mavros_msgs/GimbalManagerConfigureRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '68c32f3cd41fdfa83d7b863c5f5f2cb5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MAVLink command: DO_GIMBAL_MANAGER_CONFIGURE
    # https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
    # Note: default MAV_COMP_ID_ONBOARD_COMPUTER = 191, see MAV_COMPONENT documentation
    # https://mavlink.io/en/messages/common.html#MAV_COMPONENT
    
    int16 sysid_primary      # Sysid for primary control (0: no one in control, 
                            # -1: leave unchanged, -2: set itself in control 
                            # (for missions where the own sysid is still unknown), 
                            # -3: remove control if currently in control).
    int16 compid_primary     # Compid for primary control (0: no one in control, 
                            # -1: leave unchanged, -2: set itself in control 
                            # (for missions where the own sysid is still unknown), 
                            # -3: remove control if currently in control).
    int16 sysid_secondary    # Sysid for secondary control (0: no one in control, 
                            # -1: leave unchanged, -2: set itself in control 
                            # (for missions where the own sysid is still unknown), 
                            # -3: remove control if currently in control).
    int16 compid_secondary   # Compid for secondary control (0: no one in control, 
                            # -1: leave unchanged, -2: set itself in control 
                            # (for missions where the own sysid is still unknown), 
                            # -3: remove control if currently in control).
    
    uint8 gimbal_device_id  # Component ID of gimbal device to address 
                            # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
                            # components. Send command multiple times for more than
                            # one gimbal (but not all gimbals).
                            # Note: Default Mavlink gimbal device ids: 154, 171-175
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GimbalManagerConfigureRequest(null);
    if (msg.sysid_primary !== undefined) {
      resolved.sysid_primary = msg.sysid_primary;
    }
    else {
      resolved.sysid_primary = 0
    }

    if (msg.compid_primary !== undefined) {
      resolved.compid_primary = msg.compid_primary;
    }
    else {
      resolved.compid_primary = 0
    }

    if (msg.sysid_secondary !== undefined) {
      resolved.sysid_secondary = msg.sysid_secondary;
    }
    else {
      resolved.sysid_secondary = 0
    }

    if (msg.compid_secondary !== undefined) {
      resolved.compid_secondary = msg.compid_secondary;
    }
    else {
      resolved.compid_secondary = 0
    }

    if (msg.gimbal_device_id !== undefined) {
      resolved.gimbal_device_id = msg.gimbal_device_id;
    }
    else {
      resolved.gimbal_device_id = 0
    }

    return resolved;
    }
};

class GimbalManagerConfigureResponse {
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
    // Serializes a message object of type GimbalManagerConfigureResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [result]
    bufferOffset = _serializer.uint8(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GimbalManagerConfigureResponse
    let len;
    let data = new GimbalManagerConfigureResponse(null);
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
    return 'mavros_msgs/GimbalManagerConfigureResponse';
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
    const resolved = new GimbalManagerConfigureResponse(null);
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
  Request: GimbalManagerConfigureRequest,
  Response: GimbalManagerConfigureResponse,
  md5sum() { return '33077e29a7a8218a30def4cf603efdb4'; },
  datatype() { return 'mavros_msgs/GimbalManagerConfigure'; }
};
