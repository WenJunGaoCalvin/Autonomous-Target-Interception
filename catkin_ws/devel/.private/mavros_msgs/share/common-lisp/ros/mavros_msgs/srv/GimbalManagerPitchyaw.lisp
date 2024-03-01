; Auto-generated. Do not edit!


(cl:in-package mavros_msgs-srv)


;//! \htmlinclude GimbalManagerPitchyaw-request.msg.html

(cl:defclass <GimbalManagerPitchyaw-request> (roslisp-msg-protocol:ros-message)
  ((pitch
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
    :initform 0.0)
   (flags
    :reader flags
    :initarg :flags
    :type cl:integer
    :initform 0)
   (gimbal_device_id
    :reader gimbal_device_id
    :initarg :gimbal_device_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GimbalManagerPitchyaw-request (<GimbalManagerPitchyaw-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalManagerPitchyaw-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalManagerPitchyaw-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-srv:<GimbalManagerPitchyaw-request> is deprecated: use mavros_msgs-srv:GimbalManagerPitchyaw-request instead.")))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <GimbalManagerPitchyaw-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:pitch-val is deprecated.  Use mavros_msgs-srv:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <GimbalManagerPitchyaw-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:yaw-val is deprecated.  Use mavros_msgs-srv:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'pitch_rate-val :lambda-list '(m))
(cl:defmethod pitch_rate-val ((m <GimbalManagerPitchyaw-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:pitch_rate-val is deprecated.  Use mavros_msgs-srv:pitch_rate instead.")
  (pitch_rate m))

(cl:ensure-generic-function 'yaw_rate-val :lambda-list '(m))
(cl:defmethod yaw_rate-val ((m <GimbalManagerPitchyaw-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:yaw_rate-val is deprecated.  Use mavros_msgs-srv:yaw_rate instead.")
  (yaw_rate m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <GimbalManagerPitchyaw-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:flags-val is deprecated.  Use mavros_msgs-srv:flags instead.")
  (flags m))

(cl:ensure-generic-function 'gimbal_device_id-val :lambda-list '(m))
(cl:defmethod gimbal_device_id-val ((m <GimbalManagerPitchyaw-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:gimbal_device_id-val is deprecated.  Use mavros_msgs-srv:gimbal_device_id instead.")
  (gimbal_device_id m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GimbalManagerPitchyaw-request>)))
    "Constants for message type '<GimbalManagerPitchyaw-request>"
  '((:GIMBAL_MANAGER_FLAGS_RETRACT . 1)
    (:GIMBAL_MANAGER_FLAGS_NEUTRAL . 2)
    (:GIMBAL_MANAGER_FLAGS_ROLL_LOCK . 4)
    (:GIMBAL_MANAGER_FLAGS_PITCH_LOCK . 8)
    (:GIMBAL_MANAGER_FLAGS_YAW_LOCK . 16))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GimbalManagerPitchyaw-request)))
    "Constants for message type 'GimbalManagerPitchyaw-request"
  '((:GIMBAL_MANAGER_FLAGS_RETRACT . 1)
    (:GIMBAL_MANAGER_FLAGS_NEUTRAL . 2)
    (:GIMBAL_MANAGER_FLAGS_ROLL_LOCK . 4)
    (:GIMBAL_MANAGER_FLAGS_PITCH_LOCK . 8)
    (:GIMBAL_MANAGER_FLAGS_YAW_LOCK . 16))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalManagerPitchyaw-request>) ostream)
  "Serializes a message object of type '<GimbalManagerPitchyaw-request>"
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
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gimbal_device_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalManagerPitchyaw-request>) istream)
  "Deserializes a message object of type '<GimbalManagerPitchyaw-request>"
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
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gimbal_device_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalManagerPitchyaw-request>)))
  "Returns string type for a service object of type '<GimbalManagerPitchyaw-request>"
  "mavros_msgs/GimbalManagerPitchyawRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerPitchyaw-request)))
  "Returns string type for a service object of type 'GimbalManagerPitchyaw-request"
  "mavros_msgs/GimbalManagerPitchyawRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalManagerPitchyaw-request>)))
  "Returns md5sum for a message object of type '<GimbalManagerPitchyaw-request>"
  "64c6b1297e0d0b5bed3f6fb79c8c51aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalManagerPitchyaw-request)))
  "Returns md5sum for a message object of type 'GimbalManagerPitchyaw-request"
  "64c6b1297e0d0b5bed3f6fb79c8c51aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalManagerPitchyaw-request>)))
  "Returns full string definition for message of type '<GimbalManagerPitchyaw-request>"
  (cl:format cl:nil "# MAVLink commands: DO_GIMBAL_MANAGER_PITCHYAW~%# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW~%~%~%float32 pitch # Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode). (-180 to 180 deg)~%float32 yaw # Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode). (-180 to 180 deg)~%float32 pitch_rate # Pitch rate (positive to pitch up). (deg/s)~%float32 yaw_rate # Yaw rate (positive to yaw to the right). (deg/s)~%~%uint32 flags # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS~%#GIMBAL_MANAGER_FLAGS~%uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT~%uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL~%uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK~%uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK~%uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK~%~%uint8 gimbal_device_id  # Component ID of gimbal device to address ~%                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device~%                        # components. Send command multiple times for more than~%                        # one gimbal (but not all gimbals).  Default Mavlink gimbal ~%                        # device ids: 154, 171-175~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalManagerPitchyaw-request)))
  "Returns full string definition for message of type 'GimbalManagerPitchyaw-request"
  (cl:format cl:nil "# MAVLink commands: DO_GIMBAL_MANAGER_PITCHYAW~%# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW~%~%~%float32 pitch # Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode). (-180 to 180 deg)~%float32 yaw # Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode). (-180 to 180 deg)~%float32 pitch_rate # Pitch rate (positive to pitch up). (deg/s)~%float32 yaw_rate # Yaw rate (positive to yaw to the right). (deg/s)~%~%uint32 flags # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS~%#GIMBAL_MANAGER_FLAGS~%uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT~%uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL~%uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK~%uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK~%uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK~%~%uint8 gimbal_device_id  # Component ID of gimbal device to address ~%                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device~%                        # components. Send command multiple times for more than~%                        # one gimbal (but not all gimbals).  Default Mavlink gimbal ~%                        # device ids: 154, 171-175~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalManagerPitchyaw-request>))
  (cl:+ 0
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalManagerPitchyaw-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalManagerPitchyaw-request
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':pitch_rate (pitch_rate msg))
    (cl:cons ':yaw_rate (yaw_rate msg))
    (cl:cons ':flags (flags msg))
    (cl:cons ':gimbal_device_id (gimbal_device_id msg))
))
;//! \htmlinclude GimbalManagerPitchyaw-response.msg.html

(cl:defclass <GimbalManagerPitchyaw-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GimbalManagerPitchyaw-response (<GimbalManagerPitchyaw-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalManagerPitchyaw-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalManagerPitchyaw-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-srv:<GimbalManagerPitchyaw-response> is deprecated: use mavros_msgs-srv:GimbalManagerPitchyaw-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GimbalManagerPitchyaw-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:success-val is deprecated.  Use mavros_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <GimbalManagerPitchyaw-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:result-val is deprecated.  Use mavros_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalManagerPitchyaw-response>) ostream)
  "Serializes a message object of type '<GimbalManagerPitchyaw-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalManagerPitchyaw-response>) istream)
  "Deserializes a message object of type '<GimbalManagerPitchyaw-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalManagerPitchyaw-response>)))
  "Returns string type for a service object of type '<GimbalManagerPitchyaw-response>"
  "mavros_msgs/GimbalManagerPitchyawResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerPitchyaw-response)))
  "Returns string type for a service object of type 'GimbalManagerPitchyaw-response"
  "mavros_msgs/GimbalManagerPitchyawResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalManagerPitchyaw-response>)))
  "Returns md5sum for a message object of type '<GimbalManagerPitchyaw-response>"
  "64c6b1297e0d0b5bed3f6fb79c8c51aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalManagerPitchyaw-response)))
  "Returns md5sum for a message object of type 'GimbalManagerPitchyaw-response"
  "64c6b1297e0d0b5bed3f6fb79c8c51aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalManagerPitchyaw-response>)))
  "Returns full string definition for message of type '<GimbalManagerPitchyaw-response>"
  (cl:format cl:nil "bool success~%# raw result returned by COMMAND_ACK~%uint8 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalManagerPitchyaw-response)))
  "Returns full string definition for message of type 'GimbalManagerPitchyaw-response"
  (cl:format cl:nil "bool success~%# raw result returned by COMMAND_ACK~%uint8 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalManagerPitchyaw-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalManagerPitchyaw-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalManagerPitchyaw-response
    (cl:cons ':success (success msg))
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GimbalManagerPitchyaw)))
  'GimbalManagerPitchyaw-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GimbalManagerPitchyaw)))
  'GimbalManagerPitchyaw-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerPitchyaw)))
  "Returns string type for a service object of type '<GimbalManagerPitchyaw>"
  "mavros_msgs/GimbalManagerPitchyaw")