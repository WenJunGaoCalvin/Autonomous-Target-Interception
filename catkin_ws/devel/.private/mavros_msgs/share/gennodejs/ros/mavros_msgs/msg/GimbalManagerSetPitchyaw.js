// Auto-generated. Do not edit!

// (in-package mavros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class GimbalManagerSetPitchyaw {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_system = null;
      this.target_component = null;
      this.flags = null;
      this.gimbal_device_id = null;
      this.pitch = null;
      this.yaw = null;
      this.pitch_rate = null;
      this.yaw_rate = null;
    }
    else {
      if (initObj.hasOwnProperty('target_system')) {
        this.target_system = initObj.target_system
      }
      else {
        this.target_system = 0;
      }
      if (initObj.hasOwnProperty('target_component')) {
        this.target_component = initObj.target_component
      }
      else {
        this.target_component = 0;
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
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_rate')) {
        this.pitch_rate = initObj.pitch_rate
      }
      else {
        this.pitch_rate = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_rate')) {
        this.yaw_rate = initObj.yaw_rate
      }
      else {
        this.yaw_rate = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GimbalManagerSetPitchyaw
    // Serialize message field [target_system]
    bufferOffset = _serializer.uint8(obj.target_system, buffer, bufferOffset);
    // Serialize message field [target_component]
    bufferOffset = _serializer.uint8(obj.target_component, buffer, bufferOffset);
    // Serialize message field [flags]
    bufferOffset = _serializer.uint32(obj.flags, buffer, bufferOffset);
    // Serialize message field [gimbal_device_id]
    bufferOffset = _serializer.uint8(obj.gimbal_device_id, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float32(obj.pitch, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [pitch_rate]
    bufferOffset = _serializer.float32(obj.pitch_rate, buffer, bufferOffset);
    // Serialize message field [yaw_rate]
    bufferOffset = _serializer.float32(obj.yaw_rate, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GimbalManagerSetPitchyaw
    let len;
    let data = new GimbalManagerSetPitchyaw(null);
    // Deserialize message field [target_system]
    data.target_system = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [target_component]
    data.target_component = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [flags]
    data.flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [gimbal_device_id]
    data.gimbal_device_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_rate]
    data.pitch_rate = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw_rate]
    data.yaw_rate = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 23;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mavros_msgs/GimbalManagerSetPitchyaw';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1167a6666ca24e39fb8a2101fd672d21';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MAVLink message: GIMBAL_MANAGER_SET_PITCHYAW
    # https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_PITCHYAW
    # Note that this message structure is identical also to GIMBAL_MANAGER_SET_MANUAL_CONTROL and is 
    # reused as such by the plugin
    # https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_MANUAL_CONTROL
    
    uint8 target_system         # System ID
    uint8 target_component      # Component ID
    
    uint32 flags                # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS
    #GIMBAL_MANAGER_FLAGS
    uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT
    uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
    uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
    uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
    uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK
    
    uint8 gimbal_device_id  # Component ID of gimbal device to address 
                            # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
                            # components. Send command multiple times for more than
                            # one gimbal (but not all gimbals).  Default Mavlink gimbal 
                            # device ids: 154, 171-175
    
    float32 pitch # Pitch angle (positive: up, negative: down, NaN to be ignored).
    float32 yaw # Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).
    float32 pitch_rate # Pitch angular rate (positive: up, negative: down, NaN to be ignored).
    float32 yaw_rate # Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GimbalManagerSetPitchyaw(null);
    if (msg.target_system !== undefined) {
      resolved.target_system = msg.target_system;
    }
    else {
      resolved.target_system = 0
    }

    if (msg.target_component !== undefined) {
      resolved.target_component = msg.target_component;
    }
    else {
      resolved.target_component = 0
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

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.pitch_rate !== undefined) {
      resolved.pitch_rate = msg.pitch_rate;
    }
    else {
      resolved.pitch_rate = 0.0
    }

    if (msg.yaw_rate !== undefined) {
      resolved.yaw_rate = msg.yaw_rate;
    }
    else {
      resolved.yaw_rate = 0.0
    }

    return resolved;
    }
};

// Constants for message
GimbalManagerSetPitchyaw.Constants = {
  GIMBAL_MANAGER_FLAGS_RETRACT: 1,
  GIMBAL_MANAGER_FLAGS_NEUTRAL: 2,
  GIMBAL_MANAGER_FLAGS_ROLL_LOCK: 4,
  GIMBAL_MANAGER_FLAGS_PITCH_LOCK: 8,
  GIMBAL_MANAGER_FLAGS_YAW_LOCK: 16,
}

module.exports = GimbalManagerSetPitchyaw;
