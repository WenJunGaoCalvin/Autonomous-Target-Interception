; Auto-generated. Do not edit!


(cl:in-package mavros_msgs-msg)


;//! \htmlinclude GimbalDeviceAttitudeStatus.msg.html

(cl:defclass <GimbalDeviceAttitudeStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (target_system
    :reader target_system
    :initarg :target_system
    :type cl:fixnum
    :initform 0)
   (target_component
    :reader target_component
    :initarg :target_component
    :type cl:fixnum
    :initform 0)
   (flags
    :reader flags
    :initarg :flags
    :type cl:fixnum
    :initform 0)
   (q
    :reader q
    :initarg :q
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (angular_velocity_x
    :reader angular_velocity_x
    :initarg :angular_velocity_x
    :type cl:float
    :initform 0.0)
   (angular_velocity_y
    :reader angular_velocity_y
    :initarg :angular_velocity_y
    :type cl:float
    :initform 0.0)
   (angular_velocity_z
    :reader angular_velocity_z
    :initarg :angular_velocity_z
    :type cl:float
    :initform 0.0)
   (failure_flags
    :reader failure_flags
    :initarg :failure_flags
    :type cl:integer
    :initform 0))
)

(cl:defclass GimbalDeviceAttitudeStatus (<GimbalDeviceAttitudeStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalDeviceAttitudeStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalDeviceAttitudeStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-msg:<GimbalDeviceAttitudeStatus> is deprecated: use mavros_msgs-msg:GimbalDeviceAttitudeStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GimbalDeviceAttitudeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:header-val is deprecated.  Use mavros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'target_system-val :lambda-list '(m))
(cl:defmethod target_system-val ((m <GimbalDeviceAttitudeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:target_system-val is deprecated.  Use mavros_msgs-msg:target_system instead.")
  (target_system m))

(cl:ensure-generic-function 'target_component-val :lambda-list '(m))
(cl:defmethod target_component-val ((m <GimbalDeviceAttitudeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:target_component-val is deprecated.  Use mavros_msgs-msg:target_component instead.")
  (target_component m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <GimbalDeviceAttitudeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:flags-val is deprecated.  Use mavros_msgs-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'q-val :lambda-list '(m))
(cl:defmethod q-val ((m <GimbalDeviceAttitudeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:q-val is deprecated.  Use mavros_msgs-msg:q instead.")
  (q m))

(cl:ensure-generic-function 'angular_velocity_x-val :lambda-list '(m))
(cl:defmethod angular_velocity_x-val ((m <GimbalDeviceAttitudeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:angular_velocity_x-val is deprecated.  Use mavros_msgs-msg:angular_velocity_x instead.")
  (angular_velocity_x m))

(cl:ensure-generic-function 'angular_velocity_y-val :lambda-list '(m))
(cl:defmethod angular_velocity_y-val ((m <GimbalDeviceAttitudeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:angular_velocity_y-val is deprecated.  Use mavros_msgs-msg:angular_velocity_y instead.")
  (angular_velocity_y m))

(cl:ensure-generic-function 'angular_velocity_z-val :lambda-list '(m))
(cl:defmethod angular_velocity_z-val ((m <GimbalDeviceAttitudeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:angular_velocity_z-val is deprecated.  Use mavros_msgs-msg:angular_velocity_z instead.")
  (angular_velocity_z m))

(cl:ensure-generic-function 'failure_flags-val :lambda-list '(m))
(cl:defmethod failure_flags-val ((m <GimbalDeviceAttitudeStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:failure_flags-val is deprecated.  Use mavros_msgs-msg:failure_flags instead.")
  (failure_flags m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GimbalDeviceAttitudeStatus>)))
    "Constants for message type '<GimbalDeviceAttitudeStatus>"
  '((:GIMBAL_DEVICE_FLAGS_RETRACT . 1)
    (:GIMBAL_DEVICE_FLAGS_NEUTRAL . 2)
    (:GIMBAL_DEVICE_FLAGS_ROLL_LOCK . 4)
    (:GIMBAL_DEVICE_FLAGS_PITCH_LOCK . 8)
    (:GIMBAL_DEVICE_FLAGS_YAW_LOCK . 16)
    (:GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT . 1)
    (:GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT . 2)
    (:GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT . 4)
    (:GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR . 8)
    (:GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR . 16)
    (:GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR . 32)
    (:GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR . 64)
    (:GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR . 128)
    (:GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING . 256))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GimbalDeviceAttitudeStatus)))
    "Constants for message type 'GimbalDeviceAttitudeStatus"
  '((:GIMBAL_DEVICE_FLAGS_RETRACT . 1)
    (:GIMBAL_DEVICE_FLAGS_NEUTRAL . 2)
    (:GIMBAL_DEVICE_FLAGS_ROLL_LOCK . 4)
    (:GIMBAL_DEVICE_FLAGS_PITCH_LOCK . 8)
    (:GIMBAL_DEVICE_FLAGS_YAW_LOCK . 16)
    (:GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT . 1)
    (:GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT . 2)
    (:GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT . 4)
    (:GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR . 8)
    (:GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR . 16)
    (:GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR . 32)
    (:GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR . 64)
    (:GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR . 128)
    (:GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING . 256))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalDeviceAttitudeStatus>) ostream)
  "Serializes a message object of type '<GimbalDeviceAttitudeStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_system)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_component)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'q) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular_velocity_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular_velocity_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angular_velocity_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'failure_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'failure_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'failure_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'failure_flags)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalDeviceAttitudeStatus>) istream)
  "Deserializes a message object of type '<GimbalDeviceAttitudeStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_system)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_component)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'q) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_velocity_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_velocity_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular_velocity_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'failure_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'failure_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'failure_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'failure_flags)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalDeviceAttitudeStatus>)))
  "Returns string type for a message object of type '<GimbalDeviceAttitudeStatus>"
  "mavros_msgs/GimbalDeviceAttitudeStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalDeviceAttitudeStatus)))
  "Returns string type for a message object of type 'GimbalDeviceAttitudeStatus"
  "mavros_msgs/GimbalDeviceAttitudeStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalDeviceAttitudeStatus>)))
  "Returns md5sum for a message object of type '<GimbalDeviceAttitudeStatus>"
  "1fa324570393b1fbb776c5ff6e00764e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalDeviceAttitudeStatus)))
  "Returns md5sum for a message object of type 'GimbalDeviceAttitudeStatus"
  "1fa324570393b1fbb776c5ff6e00764e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalDeviceAttitudeStatus>)))
  "Returns full string definition for message of type '<GimbalDeviceAttitudeStatus>"
  (cl:format cl:nil "# MAVLink message: GIMBAL_DEVICE_ATTITUDE_STATUS~%# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS~%~%std_msgs/Header header~%~%uint8 target_system             # System ID~%uint8 target_component          # Component ID~%~%uint16 flags                    # Current gimbal flags set (bitwise) - See GIMBAL_DEVICE_FLAGS~%#GIMBAL_DEVICE_FLAGS~%uint16 GIMBAL_DEVICE_FLAGS_RETRACT = 1      # Set to retracted safe position (no stabilization), takes presedence over all other flags.~%uint16 GIMBAL_DEVICE_FLAGS_NEUTRAL = 2      # Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly forward-facing and horizontal (pitch=yaw=0) but may be any orientation.~%uint16 GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4    # Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal.~%uint16 GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8   # Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default.~%uint16 GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16    # Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle).~%~%geometry_msgs/Quaternion q      # Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)~%float32 angular_velocity_x      # X component of angular velocity (NaN if unknown)~%float32 angular_velocity_y      # Y component of angular velocity (NaN if unknown)~%float32 angular_velocity_z      # Z component of angular velocity (NaN if unknown)~%~%uint32 failure_flags            # Failure flags (0 for no failure) (bitwise) - See GIMBAL_DEVICE_ERROR_FLAGS~%#GIMBAL_DEVICE_ERROR_FLAGS~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT = 1          # Gimbal device is limited by hardware roll limit.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT = 2         # Gimbal device is limited by hardware pitch limit.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT = 4           # Gimbal device is limited by hardware yaw limit.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR = 8          # There is an error with the gimbal encoders.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR = 16           # There is an error with the gimbal power source.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR = 32           # There is an error with the gimbal motor's.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR = 64        # There is an error with the gimbal's software.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR = 128          # There is an error with the gimbal's communication.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING = 256  # Gimbal is currently calibrating.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalDeviceAttitudeStatus)))
  "Returns full string definition for message of type 'GimbalDeviceAttitudeStatus"
  (cl:format cl:nil "# MAVLink message: GIMBAL_DEVICE_ATTITUDE_STATUS~%# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS~%~%std_msgs/Header header~%~%uint8 target_system             # System ID~%uint8 target_component          # Component ID~%~%uint16 flags                    # Current gimbal flags set (bitwise) - See GIMBAL_DEVICE_FLAGS~%#GIMBAL_DEVICE_FLAGS~%uint16 GIMBAL_DEVICE_FLAGS_RETRACT = 1      # Set to retracted safe position (no stabilization), takes presedence over all other flags.~%uint16 GIMBAL_DEVICE_FLAGS_NEUTRAL = 2      # Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly forward-facing and horizontal (pitch=yaw=0) but may be any orientation.~%uint16 GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4    # Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal.~%uint16 GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8   # Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default.~%uint16 GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16    # Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle).~%~%geometry_msgs/Quaternion q      # Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)~%float32 angular_velocity_x      # X component of angular velocity (NaN if unknown)~%float32 angular_velocity_y      # Y component of angular velocity (NaN if unknown)~%float32 angular_velocity_z      # Z component of angular velocity (NaN if unknown)~%~%uint32 failure_flags            # Failure flags (0 for no failure) (bitwise) - See GIMBAL_DEVICE_ERROR_FLAGS~%#GIMBAL_DEVICE_ERROR_FLAGS~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT = 1          # Gimbal device is limited by hardware roll limit.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT = 2         # Gimbal device is limited by hardware pitch limit.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT = 4           # Gimbal device is limited by hardware yaw limit.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR = 8          # There is an error with the gimbal encoders.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR = 16           # There is an error with the gimbal power source.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR = 32           # There is an error with the gimbal motor's.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR = 64        # There is an error with the gimbal's software.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR = 128          # There is an error with the gimbal's communication.~%uint32 GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING = 256  # Gimbal is currently calibrating.~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalDeviceAttitudeStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     2
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'q))
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalDeviceAttitudeStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalDeviceAttitudeStatus
    (cl:cons ':header (header msg))
    (cl:cons ':target_system (target_system msg))
    (cl:cons ':target_component (target_component msg))
    (cl:cons ':flags (flags msg))
    (cl:cons ':q (q msg))
    (cl:cons ':angular_velocity_x (angular_velocity_x msg))
    (cl:cons ':angular_velocity_y (angular_velocity_y msg))
    (cl:cons ':angular_velocity_z (angular_velocity_z msg))
    (cl:cons ':failure_flags (failure_flags msg))
))
