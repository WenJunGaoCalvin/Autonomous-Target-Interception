// Auto-generated. Do not edit!

// (in-package mavros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class GimbalManagerSetAttitude {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_system = null;
      this.target_component = null;
      this.flags = null;
      this.gimbal_device_id = null;
      this.q = null;
      this.angular_velocity_x = null;
      this.angular_velocity_y = null;
      this.angular_velocity_z = null;
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
      if (initObj.hasOwnProperty('q')) {
        this.q = initObj.q
      }
      else {
        this.q = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('angular_velocity_x')) {
        this.angular_velocity_x = initObj.angular_velocity_x
      }
      else {
        this.angular_velocity_x = 0.0;
      }
      if (initObj.hasOwnProperty('angular_velocity_y')) {
        this.angular_velocity_y = initObj.angular_velocity_y
      }
      else {
        this.angular_velocity_y = 0.0;
      }
      if (initObj.hasOwnProperty('angular_velocity_z')) {
        this.angular_velocity_z = initObj.angular_velocity_z
      }
      else {
        this.angular_velocity_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GimbalManagerSetAttitude
    // Serialize message field [target_system]
    bufferOffset = _serializer.uint8(obj.target_system, buffer, bufferOffset);
    // Serialize message field [target_component]
    bufferOffset = _serializer.uint8(obj.target_component, buffer, bufferOffset);
    // Serialize message field [flags]
    bufferOffset = _serializer.uint32(obj.flags, buffer, bufferOffset);
    // Serialize message field [gimbal_device_id]
    bufferOffset = _serializer.uint8(obj.gimbal_device_id, buffer, bufferOffset);
    // Serialize message field [q]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.q, buffer, bufferOffset);
    // Serialize message field [angular_velocity_x]
    bufferOffset = _serializer.float32(obj.angular_velocity_x, buffer, bufferOffset);
    // Serialize message field [angular_velocity_y]
    bufferOffset = _serializer.float32(obj.angular_velocity_y, buffer, bufferOffset);
    // Serialize message field [angular_velocity_z]
    bufferOffset = _serializer.float32(obj.angular_velocity_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GimbalManagerSetAttitude
    let len;
    let data = new GimbalManagerSetAttitude(null);
    // Deserialize message field [target_system]
    data.target_system = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [target_component]
    data.target_component = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [flags]
    data.flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [gimbal_device_id]
    data.gimbal_device_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [q]
    data.q = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [angular_velocity_x]
    data.angular_velocity_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular_velocity_y]
    data.angular_velocity_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular_velocity_z]
    data.angular_velocity_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 51;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mavros_msgs/GimbalManagerSetAttitude';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '81ba34c5366dd06af9dda84c4cca3bd7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MAVLink message: GIMBAL_MANAGER_SET_ATTITUDE
    # https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_ATTITUDE
    
    uint8 target_system         # System ID
    uint8 target_component      # Component ID
    
    uint32 flags                # High level gimbal manager flags to use (bitwise) - See GIMBAL_MANAGER_FLAGS
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
    
    geometry_msgs/Quaternion q # Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
    float32 angular_velocity_x # X component of angular velocity, positive is rolling to the right, NaN to be ignored.
    float32 angular_velocity_y # Y component of angular velocity, positive is pitching up, NaN to be ignored.
    float32 angular_velocity_z # Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GimbalManagerSetAttitude(null);
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

    if (msg.q !== undefined) {
      resolved.q = geometry_msgs.msg.Quaternion.Resolve(msg.q)
    }
    else {
      resolved.q = new geometry_msgs.msg.Quaternion()
    }

    if (msg.angular_velocity_x !== undefined) {
      resolved.angular_velocity_x = msg.angular_velocity_x;
    }
    else {
      resolved.angular_velocity_x = 0.0
    }

    if (msg.angular_velocity_y !== undefined) {
      resolved.angular_velocity_y = msg.angular_velocity_y;
    }
    else {
      resolved.angular_velocity_y = 0.0
    }

    if (msg.angular_velocity_z !== undefined) {
      resolved.angular_velocity_z = msg.angular_velocity_z;
    }
    else {
      resolved.angular_velocity_z = 0.0
    }

    return resolved;
    }
};

// Constants for message
GimbalManagerSetAttitude.Constants = {
  GIMBAL_MANAGER_FLAGS_RETRACT: 1,
  GIMBAL_MANAGER_FLAGS_NEUTRAL: 2,
  GIMBAL_MANAGER_FLAGS_ROLL_LOCK: 4,
  GIMBAL_MANAGER_FLAGS_PITCH_LOCK: 8,
  GIMBAL_MANAGER_FLAGS_YAW_LOCK: 16,
}

module.exports = GimbalManagerSetAttitude;
