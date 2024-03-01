; Auto-generated. Do not edit!


(cl:in-package mavros_msgs-msg)


;//! \htmlinclude GimbalManagerStatus.msg.html

(cl:defclass <GimbalManagerStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
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
   (sysid_primary
    :reader sysid_primary
    :initarg :sysid_primary
    :type cl:fixnum
    :initform 0)
   (compid_primary
    :reader compid_primary
    :initarg :compid_primary
    :type cl:fixnum
    :initform 0)
   (sysid_secondary
    :reader sysid_secondary
    :initarg :sysid_secondary
    :type cl:fixnum
    :initform 0)
   (compid_secondary
    :reader compid_secondary
    :initarg :compid_secondary
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GimbalManagerStatus (<GimbalManagerStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalManagerStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalManagerStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-msg:<GimbalManagerStatus> is deprecated: use mavros_msgs-msg:GimbalManagerStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GimbalManagerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:header-val is deprecated.  Use mavros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <GimbalManagerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:flags-val is deprecated.  Use mavros_msgs-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'gimbal_device_id-val :lambda-list '(m))
(cl:defmethod gimbal_device_id-val ((m <GimbalManagerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:gimbal_device_id-val is deprecated.  Use mavros_msgs-msg:gimbal_device_id instead.")
  (gimbal_device_id m))

(cl:ensure-generic-function 'sysid_primary-val :lambda-list '(m))
(cl:defmethod sysid_primary-val ((m <GimbalManagerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:sysid_primary-val is deprecated.  Use mavros_msgs-msg:sysid_primary instead.")
  (sysid_primary m))

(cl:ensure-generic-function 'compid_primary-val :lambda-list '(m))
(cl:defmethod compid_primary-val ((m <GimbalManagerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:compid_primary-val is deprecated.  Use mavros_msgs-msg:compid_primary instead.")
  (compid_primary m))

(cl:ensure-generic-function 'sysid_secondary-val :lambda-list '(m))
(cl:defmethod sysid_secondary-val ((m <GimbalManagerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:sysid_secondary-val is deprecated.  Use mavros_msgs-msg:sysid_secondary instead.")
  (sysid_secondary m))

(cl:ensure-generic-function 'compid_secondary-val :lambda-list '(m))
(cl:defmethod compid_secondary-val ((m <GimbalManagerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:compid_secondary-val is deprecated.  Use mavros_msgs-msg:compid_secondary instead.")
  (compid_secondary m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GimbalManagerStatus>)))
    "Constants for message type '<GimbalManagerStatus>"
  '((:GIMBAL_MANAGER_FLAGS_RETRACT . 1)
    (:GIMBAL_MANAGER_FLAGS_NEUTRAL . 2)
    (:GIMBAL_MANAGER_FLAGS_ROLL_LOCK . 4)
    (:GIMBAL_MANAGER_FLAGS_PITCH_LOCK . 8)
    (:GIMBAL_MANAGER_FLAGS_YAW_LOCK . 16))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GimbalManagerStatus)))
    "Constants for message type 'GimbalManagerStatus"
  '((:GIMBAL_MANAGER_FLAGS_RETRACT . 1)
    (:GIMBAL_MANAGER_FLAGS_NEUTRAL . 2)
    (:GIMBAL_MANAGER_FLAGS_ROLL_LOCK . 4)
    (:GIMBAL_MANAGER_FLAGS_PITCH_LOCK . 8)
    (:GIMBAL_MANAGER_FLAGS_YAW_LOCK . 16))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalManagerStatus>) ostream)
  "Serializes a message object of type '<GimbalManagerStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gimbal_device_id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sysid_primary)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'compid_primary)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sysid_secondary)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'compid_secondary)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalManagerStatus>) istream)
  "Deserializes a message object of type '<GimbalManagerStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gimbal_device_id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sysid_primary)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'compid_primary)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sysid_secondary)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'compid_secondary)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalManagerStatus>)))
  "Returns string type for a message object of type '<GimbalManagerStatus>"
  "mavros_msgs/GimbalManagerStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerStatus)))
  "Returns string type for a message object of type 'GimbalManagerStatus"
  "mavros_msgs/GimbalManagerStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalManagerStatus>)))
  "Returns md5sum for a message object of type '<GimbalManagerStatus>"
  "fc335d636c5417cf82ed6184846e75a1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalManagerStatus)))
  "Returns md5sum for a message object of type 'GimbalManagerStatus"
  "fc335d636c5417cf82ed6184846e75a1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalManagerStatus>)))
  "Returns full string definition for message of type '<GimbalManagerStatus>"
  (cl:format cl:nil "# MAVLink message: GIMBAL_MANAGER_STATUS~%# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS~%~%std_msgs/Header header~%~%uint32 flags                # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS~%#GIMBAL_MANAGER_FLAGS~%uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT~%uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL~%uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK~%uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK~%uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK~%~%uint8 gimbal_device_id # Gimbal device ID that this gimbal manager is responsible for.~%~%uint8 sysid_primary     # System ID of MAVLink component with primary control, 0 for none.~%uint8 compid_primary    # Component ID of MAVLink component with primary control, 0 for none.~%uint8 sysid_secondary   # System ID of MAVLink component with secondary control, 0 for none.~%uint8 compid_secondary  # Component ID of MAVLink component with secondary control, 0 for none.~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalManagerStatus)))
  "Returns full string definition for message of type 'GimbalManagerStatus"
  (cl:format cl:nil "# MAVLink message: GIMBAL_MANAGER_STATUS~%# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS~%~%std_msgs/Header header~%~%uint32 flags                # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS~%#GIMBAL_MANAGER_FLAGS~%uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT~%uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL~%uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK~%uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK~%uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK~%~%uint8 gimbal_device_id # Gimbal device ID that this gimbal manager is responsible for.~%~%uint8 sysid_primary     # System ID of MAVLink component with primary control, 0 for none.~%uint8 compid_primary    # Component ID of MAVLink component with primary control, 0 for none.~%uint8 sysid_secondary   # System ID of MAVLink component with secondary control, 0 for none.~%uint8 compid_secondary  # Component ID of MAVLink component with secondary control, 0 for none.~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalManagerStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalManagerStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalManagerStatus
    (cl:cons ':header (header msg))
    (cl:cons ':flags (flags msg))
    (cl:cons ':gimbal_device_id (gimbal_device_id msg))
    (cl:cons ':sysid_primary (sysid_primary msg))
    (cl:cons ':compid_primary (compid_primary msg))
    (cl:cons ':sysid_secondary (sysid_secondary msg))
    (cl:cons ':compid_secondary (compid_secondary msg))
))
