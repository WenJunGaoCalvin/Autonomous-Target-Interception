; Auto-generated. Do not edit!


(cl:in-package mavros_msgs-srv)


;//! \htmlinclude GimbalManagerConfigure-request.msg.html

(cl:defclass <GimbalManagerConfigure-request> (roslisp-msg-protocol:ros-message)
  ((sysid_primary
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
    :initform 0)
   (gimbal_device_id
    :reader gimbal_device_id
    :initarg :gimbal_device_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GimbalManagerConfigure-request (<GimbalManagerConfigure-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalManagerConfigure-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalManagerConfigure-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-srv:<GimbalManagerConfigure-request> is deprecated: use mavros_msgs-srv:GimbalManagerConfigure-request instead.")))

(cl:ensure-generic-function 'sysid_primary-val :lambda-list '(m))
(cl:defmethod sysid_primary-val ((m <GimbalManagerConfigure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:sysid_primary-val is deprecated.  Use mavros_msgs-srv:sysid_primary instead.")
  (sysid_primary m))

(cl:ensure-generic-function 'compid_primary-val :lambda-list '(m))
(cl:defmethod compid_primary-val ((m <GimbalManagerConfigure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:compid_primary-val is deprecated.  Use mavros_msgs-srv:compid_primary instead.")
  (compid_primary m))

(cl:ensure-generic-function 'sysid_secondary-val :lambda-list '(m))
(cl:defmethod sysid_secondary-val ((m <GimbalManagerConfigure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:sysid_secondary-val is deprecated.  Use mavros_msgs-srv:sysid_secondary instead.")
  (sysid_secondary m))

(cl:ensure-generic-function 'compid_secondary-val :lambda-list '(m))
(cl:defmethod compid_secondary-val ((m <GimbalManagerConfigure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:compid_secondary-val is deprecated.  Use mavros_msgs-srv:compid_secondary instead.")
  (compid_secondary m))

(cl:ensure-generic-function 'gimbal_device_id-val :lambda-list '(m))
(cl:defmethod gimbal_device_id-val ((m <GimbalManagerConfigure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:gimbal_device_id-val is deprecated.  Use mavros_msgs-srv:gimbal_device_id instead.")
  (gimbal_device_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalManagerConfigure-request>) ostream)
  "Serializes a message object of type '<GimbalManagerConfigure-request>"
  (cl:let* ((signed (cl:slot-value msg 'sysid_primary)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'compid_primary)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'sysid_secondary)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'compid_secondary)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gimbal_device_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalManagerConfigure-request>) istream)
  "Deserializes a message object of type '<GimbalManagerConfigure-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sysid_primary) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'compid_primary) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sysid_secondary) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'compid_secondary) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gimbal_device_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalManagerConfigure-request>)))
  "Returns string type for a service object of type '<GimbalManagerConfigure-request>"
  "mavros_msgs/GimbalManagerConfigureRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerConfigure-request)))
  "Returns string type for a service object of type 'GimbalManagerConfigure-request"
  "mavros_msgs/GimbalManagerConfigureRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalManagerConfigure-request>)))
  "Returns md5sum for a message object of type '<GimbalManagerConfigure-request>"
  "33077e29a7a8218a30def4cf603efdb4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalManagerConfigure-request)))
  "Returns md5sum for a message object of type 'GimbalManagerConfigure-request"
  "33077e29a7a8218a30def4cf603efdb4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalManagerConfigure-request>)))
  "Returns full string definition for message of type '<GimbalManagerConfigure-request>"
  (cl:format cl:nil "# MAVLink command: DO_GIMBAL_MANAGER_CONFIGURE~%# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE~%# Note: default MAV_COMP_ID_ONBOARD_COMPUTER = 191, see MAV_COMPONENT documentation~%# https://mavlink.io/en/messages/common.html#MAV_COMPONENT~%~%int16 sysid_primary      # Sysid for primary control (0: no one in control, ~%                        # -1: leave unchanged, -2: set itself in control ~%                        # (for missions where the own sysid is still unknown), ~%                        # -3: remove control if currently in control).~%int16 compid_primary     # Compid for primary control (0: no one in control, ~%                        # -1: leave unchanged, -2: set itself in control ~%                        # (for missions where the own sysid is still unknown), ~%                        # -3: remove control if currently in control).~%int16 sysid_secondary    # Sysid for secondary control (0: no one in control, ~%                        # -1: leave unchanged, -2: set itself in control ~%                        # (for missions where the own sysid is still unknown), ~%                        # -3: remove control if currently in control).~%int16 compid_secondary   # Compid for secondary control (0: no one in control, ~%                        # -1: leave unchanged, -2: set itself in control ~%                        # (for missions where the own sysid is still unknown), ~%                        # -3: remove control if currently in control).~%~%uint8 gimbal_device_id  # Component ID of gimbal device to address ~%                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device~%                        # components. Send command multiple times for more than~%                        # one gimbal (but not all gimbals).~%                        # Note: Default Mavlink gimbal device ids: 154, 171-175~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalManagerConfigure-request)))
  "Returns full string definition for message of type 'GimbalManagerConfigure-request"
  (cl:format cl:nil "# MAVLink command: DO_GIMBAL_MANAGER_CONFIGURE~%# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE~%# Note: default MAV_COMP_ID_ONBOARD_COMPUTER = 191, see MAV_COMPONENT documentation~%# https://mavlink.io/en/messages/common.html#MAV_COMPONENT~%~%int16 sysid_primary      # Sysid for primary control (0: no one in control, ~%                        # -1: leave unchanged, -2: set itself in control ~%                        # (for missions where the own sysid is still unknown), ~%                        # -3: remove control if currently in control).~%int16 compid_primary     # Compid for primary control (0: no one in control, ~%                        # -1: leave unchanged, -2: set itself in control ~%                        # (for missions where the own sysid is still unknown), ~%                        # -3: remove control if currently in control).~%int16 sysid_secondary    # Sysid for secondary control (0: no one in control, ~%                        # -1: leave unchanged, -2: set itself in control ~%                        # (for missions where the own sysid is still unknown), ~%                        # -3: remove control if currently in control).~%int16 compid_secondary   # Compid for secondary control (0: no one in control, ~%                        # -1: leave unchanged, -2: set itself in control ~%                        # (for missions where the own sysid is still unknown), ~%                        # -3: remove control if currently in control).~%~%uint8 gimbal_device_id  # Component ID of gimbal device to address ~%                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device~%                        # components. Send command multiple times for more than~%                        # one gimbal (but not all gimbals).~%                        # Note: Default Mavlink gimbal device ids: 154, 171-175~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalManagerConfigure-request>))
  (cl:+ 0
     2
     2
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalManagerConfigure-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalManagerConfigure-request
    (cl:cons ':sysid_primary (sysid_primary msg))
    (cl:cons ':compid_primary (compid_primary msg))
    (cl:cons ':sysid_secondary (sysid_secondary msg))
    (cl:cons ':compid_secondary (compid_secondary msg))
    (cl:cons ':gimbal_device_id (gimbal_device_id msg))
))
;//! \htmlinclude GimbalManagerConfigure-response.msg.html

(cl:defclass <GimbalManagerConfigure-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GimbalManagerConfigure-response (<GimbalManagerConfigure-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalManagerConfigure-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalManagerConfigure-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-srv:<GimbalManagerConfigure-response> is deprecated: use mavros_msgs-srv:GimbalManagerConfigure-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GimbalManagerConfigure-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:success-val is deprecated.  Use mavros_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <GimbalManagerConfigure-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:result-val is deprecated.  Use mavros_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalManagerConfigure-response>) ostream)
  "Serializes a message object of type '<GimbalManagerConfigure-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalManagerConfigure-response>) istream)
  "Deserializes a message object of type '<GimbalManagerConfigure-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalManagerConfigure-response>)))
  "Returns string type for a service object of type '<GimbalManagerConfigure-response>"
  "mavros_msgs/GimbalManagerConfigureResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerConfigure-response)))
  "Returns string type for a service object of type 'GimbalManagerConfigure-response"
  "mavros_msgs/GimbalManagerConfigureResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalManagerConfigure-response>)))
  "Returns md5sum for a message object of type '<GimbalManagerConfigure-response>"
  "33077e29a7a8218a30def4cf603efdb4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalManagerConfigure-response)))
  "Returns md5sum for a message object of type 'GimbalManagerConfigure-response"
  "33077e29a7a8218a30def4cf603efdb4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalManagerConfigure-response>)))
  "Returns full string definition for message of type '<GimbalManagerConfigure-response>"
  (cl:format cl:nil "bool success~%# raw result returned by COMMAND_ACK~%uint8 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalManagerConfigure-response)))
  "Returns full string definition for message of type 'GimbalManagerConfigure-response"
  (cl:format cl:nil "bool success~%# raw result returned by COMMAND_ACK~%uint8 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalManagerConfigure-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalManagerConfigure-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalManagerConfigure-response
    (cl:cons ':success (success msg))
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GimbalManagerConfigure)))
  'GimbalManagerConfigure-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GimbalManagerConfigure)))
  'GimbalManagerConfigure-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerConfigure)))
  "Returns string type for a service object of type '<GimbalManagerConfigure>"
  "mavros_msgs/GimbalManagerConfigure")