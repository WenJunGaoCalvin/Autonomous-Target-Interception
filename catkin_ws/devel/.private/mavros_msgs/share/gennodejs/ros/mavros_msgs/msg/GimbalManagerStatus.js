// Auto-generated. Do not edit!

// (in-package mavros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class GimbalManagerStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.flags = null;
      this.gimbal_device_id = null;
      this.sysid_primary = null;
      this.compid_primary = null;
      this.sysid_secondary = null;
      this.compid_secondary = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('flags')) {
        this.flags = initObj.flags
      }
      else {
        this.flags = 0;
      }
      if (initObj.hasOwnProperty('gimbal_device_id')) {
        this.gimbal_device_id = initObj.gimbal_device_id
      }
      else {
        this.gimbal_device_id = 0;
      }
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GimbalManagerStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [flags]
    bufferOffset = _serializer.uint32(obj.flags, buffer, bufferOffset);
    // Serialize message field [gimbal_device_id]
    bufferOffset = _serializer.uint8(obj.gimbal_device_id, buffer, bufferOffset);
    // Serialize message field [sysid_primary]
    bufferOffset = _serializer.uint8(obj.sysid_primary, buffer, bufferOffset);
    // Serialize message field [compid_primary]
    bufferOffset = _serializer.uint8(obj.compid_primary, buffer, bufferOffset);
    // Serialize message field [sysid_secondary]
    bufferOffset = _serializer.uint8(obj.sysid_secondary, buffer, bufferOffset);
    // Serialize message field [compid_secondary]
    bufferOffset = _serializer.uint8(obj.compid_secondary, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GimbalManagerStatus
    let len;
    let data = new GimbalManagerStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [flags]
    data.flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [gimbal_device_id]
    data.gimbal_device_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sysid_primary]
    data.sysid_primary = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [compid_primary]
    data.compid_primary = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sysid_secondary]
    data.sysid_secondary = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [compid_secondary]
    data.compid_secondary = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mavros_msgs/GimbalManagerStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc335d636c5417cf82ed6184846e75a1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MAVLink message: GIMBAL_MANAGER_STATUS
    # https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS
    
    std_msgs/Header header
    
    uint32 flags                # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS
    #GIMBAL_MANAGER_FLAGS
    uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT
    uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
    uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
    uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
    uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK
    
    uint8 gimbal_device_id # Gimbal device ID that this gimbal manager is responsible for.
    
    uint8 sysid_primary     # System ID of MAVLink component with primary control, 0 for none.
    uint8 compid_primary    # Component ID of MAVLink component with primary control, 0 for none.
    uint8 sysid_secondary   # System ID of MAVLink component with secondary control, 0 for none.
    uint8 compid_secondary  # Component ID of MAVLink component with secondary control, 0 for none.
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GimbalManagerStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.flags !== undefined) {
      resolved.flags = msg.flags;
    }
    else {
      resolved.flags = 0
    }

    if (msg.gimbal_device_id !== undefined) {
      resolved.gimbal_device_id = msg.gimbal_device_id;
    }
    else {
      resolved.gimbal_device_id = 0
    }

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

    return resolved;
    }
};

// Constants for message
GimbalManagerStatus.Constants = {
  GIMBAL_MANAGER_FLAGS_RETRACT: 1,
  GIMBAL_MANAGER_FLAGS_NEUTRAL: 2,
  GIMBAL_MANAGER_FLAGS_ROLL_LOCK: 4,
  GIMBAL_MANAGER_FLAGS_PITCH_LOCK: 8,
  GIMBAL_MANAGER_FLAGS_YAW_LOCK: 16,
}

module.exports = GimbalManagerStatus;
