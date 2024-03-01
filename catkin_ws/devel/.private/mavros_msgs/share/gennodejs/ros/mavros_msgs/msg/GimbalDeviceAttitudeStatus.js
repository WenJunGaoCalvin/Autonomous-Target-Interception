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
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class GimbalDeviceAttitudeStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.target_system = null;
      this.target_component = null;
      this.flags = null;
      this.q = null;
      this.angular_velocity_x = null;
      this.angular_velocity_y = null;
      this.angular_velocity_z = null;
      this.failure_flags = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
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
      if (initObj.hasOwnProperty('failure_flags')) {
        this.failure_flags = initObj.failure_flags
      }
      else {
        this.failure_flags = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GimbalDeviceAttitudeStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [target_system]
    bufferOffset = _serializer.uint8(obj.target_system, buffer, bufferOffset);
    // Serialize message field [target_component]
    bufferOffset = _serializer.uint8(obj.target_component, buffer, bufferOffset);
    // Serialize message field [flags]
    bufferOffset = _serializer.uint16(obj.flags, buffer, bufferOffset);
    // Serialize message field [q]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.q, buffer, bufferOffset);
    // Serialize message field [angular_velocity_x]
    bufferOffset = _serializer.float32(obj.angular_velocity_x, buffer, bufferOffset);
    // Serialize message field [angular_velocity_y]
    bufferOffset = _serializer.float32(obj.angular_velocity_y, buffer, bufferOffset);
    // Serialize message field [angular_velocity_z]
    bufferOffset = _serializer.float32(obj.angular_velocity_z, buffer, bufferOffset);
    // Serialize message field [failure_flags]
    bufferOffset = _serializer.uint32(obj.failure_flags, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GimbalDeviceAttitudeStatus
    let len;
    let data = new GimbalDeviceAttitudeStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_system]
    data.target_system = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [target_component]
    data.target_component = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [flags]
    data.flags = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [q]
    data.q = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [angular_velocity_x]
    data.angular_velocity_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular_velocity_y]
    data.angular_velocity_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angular_velocity_z]
    data.angular_velocity_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [failure_flags]
    data.failure_flags = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mavros_msgs/GimbalDeviceAttitudeStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1fa324570393b1fbb776c5ff6e00764e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MAVLink message: GIMBAL_DEVICE_ATTITUDE_STATUS
    # https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS
    
    std_msgs/Header header
    
    uint8 target_system             # System ID
    uint8 target_component          # Component ID
    
    uint16 flags                    # Current gimbal flags set (bitwise) - See GIMBAL_DEVICE_FLAGS
    #GIMBAL_DEVICE_FLAGS
    uint16 GIMBAL_DEVICE_FLAGS_RETRACT = 1      # Set to retracted safe position (no stabilization), takes presedence over all other flags.
    uint16 GIMBAL_DEVICE_FLAGS_NEUTRAL = 2      # Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly forward-facing and horizontal (pitch=yaw=0) but may be any orientation.
    uint16 GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4    # Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal.
    uint16 GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8   # Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default.
    uint16 GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16    # Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle).
    
    geometry_msgs/Quaternion q      # Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
    float32 angular_velocity_x      # X component of angular velocity (NaN if unknown)
    float32 angular_velocity_y      # Y component of angular velocity (NaN if unknown)
    float32 angular_velocity_z      # Z component of angular velocity (NaN if unknown)
    
    uint32 failure_flags            # Failure flags (0 for no failure) (bitwise) - See GIMBAL_DEVICE_ERROR_FLAGS
    #GIMBAL_DEVICE_ERROR_FLAGS
    uint32 GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT = 1          # Gimbal device is limited by hardware roll limit.
    uint32 GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT = 2         # Gimbal device is limited by hardware pitch limit.
    uint32 GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT = 4           # Gimbal device is limited by hardware yaw limit.
    uint32 GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR = 8          # There is an error with the gimbal encoders.
    uint32 GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR = 16           # There is an error with the gimbal power source.
    uint32 GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR = 32           # There is an error with the gimbal motor's.
    uint32 GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR = 64        # There is an error with the gimbal's software.
    uint32 GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR = 128          # There is an error with the gimbal's communication.
    uint32 GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING = 256  # Gimbal is currently calibrating.
    
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
    const resolved = new GimbalDeviceAttitudeStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

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

    if (msg.failure_flags !== undefined) {
      resolved.failure_flags = msg.failure_flags;
    }
    else {
      resolved.failure_flags = 0
    }

    return resolved;
    }
};

// Constants for message
GimbalDeviceAttitudeStatus.Constants = {
  GIMBAL_DEVICE_FLAGS_RETRACT: 1,
  GIMBAL_DEVICE_FLAGS_NEUTRAL: 2,
  GIMBAL_DEVICE_FLAGS_ROLL_LOCK: 4,
  GIMBAL_DEVICE_FLAGS_PITCH_LOCK: 8,
  GIMBAL_DEVICE_FLAGS_YAW_LOCK: 16,
  GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT: 1,
  GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT: 2,
  GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT: 4,
  GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR: 8,
  GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR: 16,
  GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR: 32,
  GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR: 64,
  GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR: 128,
  GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING: 256,
}

module.exports = GimbalDeviceAttitudeStatus;
