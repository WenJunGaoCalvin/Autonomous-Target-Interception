; Auto-generated. Do not edit!


(cl:in-package mavros_msgs-msg)


;//! \htmlinclude GimbalManagerSetPitchyaw.msg.html

(cl:defclass <GimbalManagerSetPitchyaw> (roslisp-msg-protocol:ros-message)
  ((target_system
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
    :type cl:integer
    :initform 0)
   (gimbal_device_id
    :reader gimbal_device_id
    :initarg :gimbal_device_id
    :type cl:fixnum
    :initform 0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (pitch_rate
    :reader pitch_rate
    :initarg :pitch_rate
    :type cl:float
    :initform 0.0)
   (yaw_rate
    :reader yaw_rate
    :initarg :yaw_rate
    :type cl:float
    :initform 0.0))
)

(cl:defclass GimbalManagerSetPitchyaw (<GimbalManagerSetPitchyaw>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalManagerSetPitchyaw>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalManagerSetPitchyaw)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-msg:<GimbalManagerSetPitchyaw> is deprecated: use mavros_msgs-msg:GimbalManagerSetPitchyaw instead.")))

(cl:ensure-generic-function 'target_system-val :lambda-list '(m))
(cl:defmethod target_system-val ((m <GimbalManagerSetPitchyaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:target_system-val is deprecated.  Use mavros_msgs-msg:target_system instead.")
  (target_system m))

(cl:ensure-generic-function 'target_component-val :lambda-list '(m))
(cl:defmethod target_component-val ((m <GimbalManagerSetPitchyaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:target_component-val is deprecated.  Use mavros_msgs-msg:target_component instead.")
  (target_component m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <GimbalManagerSetPitchyaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:flags-val is deprecated.  Use mavros_msgs-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'gimbal_device_id-val :lambda-list '(m))
(cl:defmethod gimbal_device_id-val ((m <GimbalManagerSetPitchyaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:gimbal_device_id-val is deprecated.  Use mavros_msgs-msg:gimbal_device_id instead.")
  (gimbal_device_id m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <GimbalManagerSetPitchyaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:pitch-val is deprecated.  Use mavros_msgs-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <GimbalManagerSetPitchyaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:yaw-val is deprecated.  Use mavros_msgs-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'pitch_rate-val :lambda-list '(m))
(cl:defmethod pitch_rate-val ((m <GimbalManagerSetPitchyaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:pitch_rate-val is deprecated.  Use mavros_msgs-msg:pitch_rate instead.")
  (pitch_rate m))

(cl:ensure-generic-function 'yaw_rate-val :lambda-list '(m))
(cl:defmethod yaw_rate-val ((m <GimbalManagerSetPitchyaw>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:yaw_rate-val is deprecated.  Use mavros_msgs-msg:yaw_rate instead.")
  (yaw_rate m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GimbalManagerSetPitchyaw>)))
    "Constants for message type '<GimbalManagerSetPitchyaw>"
  '((:GIMBAL_MANAGER_FLAGS_RETRACT . 1)
    (:GIMBAL_MANAGER_FLAGS_NEUTRAL . 2)
    (:GIMBAL_MANAGER_FLAGS_ROLL_LOCK . 4)
    (:GIMBAL_MANAGER_FLAGS_PITCH_LOCK . 8)
    (:GIMBAL_MANAGER_FLAGS_YAW_LOCK . 16))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GimbalManagerSetPitchyaw)))
    "Constants for message type 'GimbalManagerSetPitchyaw"
  '((:GIMBAL_MANAGER_FLAGS_RETRACT . 1)
    (:GIMBAL_MANAGER_FLAGS_NEUTRAL . 2)
    (:GIMBAL_MANAGER_FLAGS_ROLL_LOCK . 4)
    (:GIMBAL_MANAGER_FLAGS_PITCH_LOCK . 8)
    (:GIMBAL_MANAGER_FLAGS_YAW_LOCK . 16))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalManagerSetPitchyaw>) ostream)
  "Serializes a message object of type '<GimbalManagerSetPitchyaw>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_system)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_component)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gimbal_device_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalManagerSetPitchyaw>) istream)
  "Deserializes a message object of type '<GimbalManagerSetPitchyaw>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_system)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'target_component)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gimbal_device_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_rate) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalManagerSetPitchyaw>)))
  "Returns string type for a message object of type '<GimbalManagerSetPitchyaw>"
  "mavros_msgs/GimbalManagerSetPitchyaw")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerSetPitchyaw)))
  "Returns string type for a message object of type 'GimbalManagerSetPitchyaw"
  "mavros_msgs/GimbalManagerSetPitchyaw")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalManagerSetPitchyaw>)))
  "Returns md5sum for a message object of type '<GimbalManagerSetPitchyaw>"
  "1167a6666ca24e39fb8a2101fd672d21")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalManagerSetPitchyaw)))
  "Returns md5sum for a message object of type 'GimbalManagerSetPitchyaw"
  "1167a6666ca24e39fb8a2101fd672d21")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalManagerSetPitchyaw>)))
  "Returns full string definition for message of type '<GimbalManagerSetPitchyaw>"
  (cl:format cl:nil "# MAVLink message: GIMBAL_MANAGER_SET_PITCHYAW~%# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_PITCHYAW~%# Note that this message structure is identical also to GIMBAL_MANAGER_SET_MANUAL_CONTROL and is ~%# reused as such by the plugin~%# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_MANUAL_CONTROL~%~%uint8 target_system         # System ID~%uint8 target_component      # Component ID~%~%uint32 flags                # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS~%#GIMBAL_MANAGER_FLAGS~%uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT~%uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL~%uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK~%uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK~%uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK~%~%uint8 gimbal_device_id  # Component ID of gimbal device to address ~%                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device~%                        # components. Send command multiple times for more than~%                        # one gimbal (but not all gimbals).  Default Mavlink gimbal ~%                        # device ids: 154, 171-175~%~%float32 pitch # Pitch angle (positive: up, negative: down, NaN to be ignored).~%float32 yaw # Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).~%float32 pitch_rate # Pitch angular rate (positive: up, negative: down, NaN to be ignored).~%float32 yaw_rate # Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalManagerSetPitchyaw)))
  "Returns full string definition for message of type 'GimbalManagerSetPitchyaw"
  (cl:format cl:nil "# MAVLink message: GIMBAL_MANAGER_SET_PITCHYAW~%# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_PITCHYAW~%# Note that this message structure is identical also to GIMBAL_MANAGER_SET_MANUAL_CONTROL and is ~%# reused as such by the plugin~%# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_MANUAL_CONTROL~%~%uint8 target_system         # System ID~%uint8 target_component      # Component ID~%~%uint32 flags                # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS~%#GIMBAL_MANAGER_FLAGS~%uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT~%uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL~%uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK~%uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK~%uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK~%~%uint8 gimbal_device_id  # Component ID of gimbal device to address ~%                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device~%                        # components. Send command multiple times for more than~%                        # one gimbal (but not all gimbals).  Default Mavlink gimbal ~%                        # device ids: 154, 171-175~%~%float32 pitch # Pitch angle (positive: up, negative: down, NaN to be ignored).~%float32 yaw # Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).~%float32 pitch_rate # Pitch angular rate (positive: up, negative: down, NaN to be ignored).~%float32 yaw_rate # Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalManagerSetPitchyaw>))
  (cl:+ 0
     1
     1
     4
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalManagerSetPitchyaw>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalManagerSetPitchyaw
    (cl:cons ':target_system (target_system msg))
    (cl:cons ':target_component (target_component msg))
    (cl:cons ':flags (flags msg))
    (cl:cons ':gimbal_device_id (gimbal_device_id msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':pitch_rate (pitch_rate msg))
    (cl:cons ':yaw_rate (yaw_rate msg))
))
