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

class GimbalDeviceInformation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.vendor_name = null;
      this.model_name = null;
      this.custom_name = null;
      this.firmware_version = null;
      this.hardware_version = null;
      this.uid = null;
      this.cap_flags = null;
      this.custom_cap_flags = null;
      this.roll_min = null;
      this.roll_max = null;
      this.pitch_min = null;
      this.pitch_max = null;
      this.yaw_min = null;
      this.yaw_max = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('vendor_name')) {
        this.vendor_name = initObj.vendor_name
      }
      else {
        this.vendor_name = '';
      }
      if (initObj.hasOwnProperty('model_name')) {
        this.model_name = initObj.model_name
      }
      else {
        this.model_name = '';
      }
      if (initObj.hasOwnProperty('custom_name')) {
        this.custom_name = initObj.custom_name
      }
      else {
        this.custom_name = '';
      }
      if (initObj.hasOwnProperty('firmware_version')) {
        this.firmware_version = initObj.firmware_version
      }
      else {
        this.firmware_version = 0;
      }
      if (initObj.hasOwnProperty('hardware_version')) {
        this.hardware_version = initObj.hardware_version
      }
      else {
        this.hardware_version = 0;
      }
      if (initObj.hasOwnProperty('uid')) {
        this.uid = initObj.uid
      }
      else {
        this.uid = 0;
      }
      if (initObj.hasOwnProperty('cap_flags')) {
        this.cap_flags = initObj.cap_flags
      }
      else {
        this.cap_flags = 0;
      }
      if (initObj.hasOwnProperty('custom_cap_flags')) {
        this.custom_cap_flags = initObj.custom_cap_flags
      }
      else {
        this.custom_cap_flags = 0;
      }
      if (initObj.hasOwnProperty('roll_min')) {
        this.roll_min = initObj.roll_min
      }
      else {
        this.roll_min = 0.0;
      }
      if (initObj.hasOwnProperty('roll_max')) {
        this.roll_max = initObj.roll_max
      }
      else {
        this.roll_max = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_min')) {
        this.pitch_min = initObj.pitch_min
      }
      else {
        this.pitch_min = 0.0;
      }
      if (initObj.hasOwnProperty('pitch_max')) {
        this.pitch_max = initObj.pitch_max
      }
      else {
        this.pitch_max = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_min')) {
        this.yaw_min = initObj.yaw_min
      }
      else {
        this.yaw_min = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_max')) {
        this.yaw_max = initObj.yaw_max
      }
      else {
        this.yaw_max = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GimbalDeviceInformation
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [vendor_name]
    bufferOffset = _serializer.string(obj.vendor_name, buffer, bufferOffset);
    // Serialize message field [model_name]
    bufferOffset = _serializer.string(obj.model_name, buffer, bufferOffset);
    // Serialize message field [custom_name]
    bufferOffset = _serializer.string(obj.custom_name, buffer, bufferOffset);
    // Serialize message field [firmware_version]
    bufferOffset = _serializer.uint32(obj.firmware_version, buffer, bufferOffset);
    // Serialize message field [hardware_version]
    bufferOffset = _serializer.uint32(obj.hardware_version, buffer, bufferOffset);
    // Serialize message field [uid]
    bufferOffset = _serializer.uint64(obj.uid, buffer, bufferOffset);
    // Serialize message field [cap_flags]
    bufferOffset = _serializer.uint32(obj.cap_flags, buffer, bufferOffset);
    // Serialize message field [custom_cap_flags]
    bufferOffset = _serializer.uint16(obj.custom_cap_flags, buffer, bufferOffset);
    // Serialize message field [roll_min]
    bufferOffset = _serializer.float32(obj.roll_min, buffer, bufferOffset);
    // Serialize message field [roll_max]
    bufferOffset = _serializer.float32(obj.roll_max, buffer, bufferOffset);
    // Serialize message field [pitch_min]
    bufferOffset = _serializer.float32(obj.pitch_min, buffer, bufferOffset);
    // Serialize message field [pitch_max]
    bufferOffset = _serializer.float32(obj.pitch_max, buffer, bufferOffset);
    // Serialize message field [yaw_min]
    bufferOffset = _serializer.float32(obj.yaw_min, buffer, bufferOffset);
    // Serialize message field [yaw_max]
    bufferOffset = _serializer.float32(obj.yaw_max, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GimbalDeviceInformation
    let len;
    let data = new GimbalDeviceInformation(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [vendor_name]
    data.vendor_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [model_name]
    data.model_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [custom_name]
    data.custom_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [firmware_version]
    data.firmware_version = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [hardware_version]
    data.hardware_version = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [uid]
    data.uid = _deserializer.uint64(buffer, bufferOffset);
    // Deserialize message field [cap_flags]
    data.cap_flags = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [custom_cap_flags]
    data.custom_cap_flags = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [roll_min]
    data.roll_min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [roll_max]
    data.roll_max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_min]
    data.pitch_min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch_max]
    data.pitch_max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw_min]
    data.yaw_min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw_max]
    data.yaw_max = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.vendor_name);
    length += _getByteLength(object.model_name);
    length += _getByteLength(object.custom_name);
    return length + 58;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mavros_msgs/GimbalDeviceInformation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cf9937985a9347aa15974b9c7eb2fa20';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # MAVLink message: GIMBAL_DEVICE_INFORMATION
    # https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION
    
    std_msgs/Header header
    
    string vendor_name # Name of the gimbal vendor.
    string model_name # Name of the gimbal model.
    string custom_name # Custom name of the gimbal given to it by the user.
    uint32 firmware_version # Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
    uint32 hardware_version # Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).
    uint64 uid # UID of gimbal hardware (0 if unknown).
    
    uint32 cap_flags # Bitmap of gimbal capability flags - see GIMBAL_DEVICE_CAP_FLAGS
    #GIMBAL_DEVICE_CAP_FLAGS
    uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT = 1 # Gimbal device supports a retracted position
    uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL = 2 # Gimbal device supports a horizontal, forward looking position, stabilized
    uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS = 4 # Gimbal device supports rotating around roll axis.
    uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW = 8 # Gimbal device supports to follow a roll angle relative to the vehicle
    uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK = 16 # Gimbal device supports locking to an roll angle (generally that's the default with roll stabilized)
    uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS = 32 # Gimbal device supports rotating around pitch axis.
    uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW = 64 # Gimbal device supports to follow a pitch angle relative to the vehicle
    uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK = 128 # Gimbal device supports locking to an pitch angle (generally that's the default with pitch stabilized)
    uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS = 256 # Gimbal device supports rotating around yaw axis.
    uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW = 512 # Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default)
    uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK = 1024 # Gimbal device supports locking to an absolute heading (often this is an option available)
    uint32 GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048 # Gimbal device supports yawing/panning infinetely (e.g. using slip disk).
    
    uint16 custom_cap_flags # Bitmap for use for gimbal-specific capability flags.
    float32 roll_min # Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
    float32 roll_max # Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
    float32 pitch_min # Minimum pitch angle (positive: up, negative: down)
    float32 pitch_max # Maximum pitch angle (positive: up, negative: down)
    float32 yaw_min # Minimum yaw angle (positive: to the right, negative: to the left)
    float32 yaw_max # Maximum yaw angle (positive: to the right, negative: to the left)
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
    const resolved = new GimbalDeviceInformation(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.vendor_name !== undefined) {
      resolved.vendor_name = msg.vendor_name;
    }
    else {
      resolved.vendor_name = ''
    }

    if (msg.model_name !== undefined) {
      resolved.model_name = msg.model_name;
    }
    else {
      resolved.model_name = ''
    }

    if (msg.custom_name !== undefined) {
      resolved.custom_name = msg.custom_name;
    }
    else {
      resolved.custom_name = ''
    }

    if (msg.firmware_version !== undefined) {
      resolved.firmware_version = msg.firmware_version;
    }
    else {
      resolved.firmware_version = 0
    }

    if (msg.hardware_version !== undefined) {
      resolved.hardware_version = msg.hardware_version;
    }
    else {
      resolved.hardware_version = 0
    }

    if (msg.uid !== undefined) {
      resolved.uid = msg.uid;
    }
    else {
      resolved.uid = 0
    }

    if (msg.cap_flags !== undefined) {
      resolved.cap_flags = msg.cap_flags;
    }
    else {
      resolved.cap_flags = 0
    }

    if (msg.custom_cap_flags !== undefined) {
      resolved.custom_cap_flags = msg.custom_cap_flags;
    }
    else {
      resolved.custom_cap_flags = 0
    }

    if (msg.roll_min !== undefined) {
      resolved.roll_min = msg.roll_min;
    }
    else {
      resolved.roll_min = 0.0
    }

    if (msg.roll_max !== undefined) {
      resolved.roll_max = msg.roll_max;
    }
    else {
      resolved.roll_max = 0.0
    }

    if (msg.pitch_min !== undefined) {
      resolved.pitch_min = msg.pitch_min;
    }
    else {
      resolved.pitch_min = 0.0
    }

    if (msg.pitch_max !== undefined) {
      resolved.pitch_max = msg.pitch_max;
    }
    else {
      resolved.pitch_max = 0.0
    }

    if (msg.yaw_min !== undefined) {
      resolved.yaw_min = msg.yaw_min;
    }
    else {
      resolved.yaw_min = 0.0
    }

    if (msg.yaw_max !== undefined) {
      resolved.yaw_max = msg.yaw_max;
    }
    else {
      resolved.yaw_max = 0.0
    }

    return resolved;
    }
};

// Constants for message
GimbalDeviceInformation.Constants = {
  GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT: 1,
  GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL: 2,
  GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS: 4,
  GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW: 8,
  GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK: 16,
  GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS: 32,
  GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW: 64,
  GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK: 128,
  GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS: 256,
  GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW: 512,
  GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK: 1024,
  GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW: 2048,
}

module.exports = GimbalDeviceInformation;
