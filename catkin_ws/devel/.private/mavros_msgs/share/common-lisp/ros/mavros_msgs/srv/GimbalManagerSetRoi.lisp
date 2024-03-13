; Auto-generated. Do not edit!


(cl:in-package mavros_msgs-srv)


;//! \htmlinclude GimbalManagerSetRoi-request.msg.html

(cl:defclass <GimbalManagerSetRoi-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (gimbal_device_id
    :reader gimbal_device_id
    :initarg :gimbal_device_id
    :type cl:fixnum
    :initform 0)
   (latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (altitude
    :reader altitude
    :initarg :altitude
    :type cl:float
    :initform 0.0)
   (pitch_offset
    :reader pitch_offset
    :initarg :pitch_offset
    :type cl:float
    :initform 0.0)
   (roll_offset
    :reader roll_offset
    :initarg :roll_offset
    :type cl:float
    :initform 0.0)
   (yaw_offset
    :reader yaw_offset
    :initarg :yaw_offset
    :type cl:float
    :initform 0.0)
   (sysid
    :reader sysid
    :initarg :sysid
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GimbalManagerSetRoi-request (<GimbalManagerSetRoi-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalManagerSetRoi-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalManagerSetRoi-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-srv:<GimbalManagerSetRoi-request> is deprecated: use mavros_msgs-srv:GimbalManagerSetRoi-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <GimbalManagerSetRoi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:mode-val is deprecated.  Use mavros_msgs-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'gimbal_device_id-val :lambda-list '(m))
(cl:defmethod gimbal_device_id-val ((m <GimbalManagerSetRoi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:gimbal_device_id-val is deprecated.  Use mavros_msgs-srv:gimbal_device_id instead.")
  (gimbal_device_id m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <GimbalManagerSetRoi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:latitude-val is deprecated.  Use mavros_msgs-srv:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <GimbalManagerSetRoi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:longitude-val is deprecated.  Use mavros_msgs-srv:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <GimbalManagerSetRoi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:altitude-val is deprecated.  Use mavros_msgs-srv:altitude instead.")
  (altitude m))

(cl:ensure-generic-function 'pitch_offset-val :lambda-list '(m))
(cl:defmethod pitch_offset-val ((m <GimbalManagerSetRoi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:pitch_offset-val is deprecated.  Use mavros_msgs-srv:pitch_offset instead.")
  (pitch_offset m))

(cl:ensure-generic-function 'roll_offset-val :lambda-list '(m))
(cl:defmethod roll_offset-val ((m <GimbalManagerSetRoi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:roll_offset-val is deprecated.  Use mavros_msgs-srv:roll_offset instead.")
  (roll_offset m))

(cl:ensure-generic-function 'yaw_offset-val :lambda-list '(m))
(cl:defmethod yaw_offset-val ((m <GimbalManagerSetRoi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:yaw_offset-val is deprecated.  Use mavros_msgs-srv:yaw_offset instead.")
  (yaw_offset m))

(cl:ensure-generic-function 'sysid-val :lambda-list '(m))
(cl:defmethod sysid-val ((m <GimbalManagerSetRoi-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:sysid-val is deprecated.  Use mavros_msgs-srv:sysid instead.")
  (sysid m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GimbalManagerSetRoi-request>)))
    "Constants for message type '<GimbalManagerSetRoi-request>"
  '((:ROI_MODE_LOCATION . 0)
    (:ROI_MODE_WP_NEXT_OFFSET . 1)
    (:ROI_MODE_SYSID . 2)
    (:ROI_MODE_NONE . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GimbalManagerSetRoi-request)))
    "Constants for message type 'GimbalManagerSetRoi-request"
  '((:ROI_MODE_LOCATION . 0)
    (:ROI_MODE_WP_NEXT_OFFSET . 1)
    (:ROI_MODE_SYSID . 2)
    (:ROI_MODE_NONE . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalManagerSetRoi-request>) ostream)
  "Serializes a message object of type '<GimbalManagerSetRoi-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gimbal_device_id)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'altitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sysid)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalManagerSetRoi-request>) istream)
  "Deserializes a message object of type '<GimbalManagerSetRoi-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gimbal_device_id)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_offset) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_offset) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_offset) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sysid)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalManagerSetRoi-request>)))
  "Returns string type for a service object of type '<GimbalManagerSetRoi-request>"
  "mavros_msgs/GimbalManagerSetRoiRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerSetRoi-request)))
  "Returns string type for a service object of type 'GimbalManagerSetRoi-request"
  "mavros_msgs/GimbalManagerSetRoiRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalManagerSetRoi-request>)))
  "Returns md5sum for a message object of type '<GimbalManagerSetRoi-request>"
  "0796c6a6cf0e18b0d829681b29f0828a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalManagerSetRoi-request)))
  "Returns md5sum for a message object of type 'GimbalManagerSetRoi-request"
  "0796c6a6cf0e18b0d829681b29f0828a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalManagerSetRoi-request>)))
  "Returns full string definition for message of type '<GimbalManagerSetRoi-request>"
  (cl:format cl:nil "# MAVLink commands: DO_SET_ROI_LOCATION, DO_SET_ROI_WPNEXT_OFFSET, DO_SET_ROI_SYSID, DO_SET_ROI_NONE~%# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_LOCATION~%# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET~%# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_SYSID~%# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_NONE~%~%uint8 mode              # enumerator to indicate ROI mode setting - see ROI_MODE~%#ROI_MODE~%uint8 ROI_MODE_LOCATION = 0         # Sets the region of interest (ROI) to a location. [DO_SET_ROI_LOCATION]~%uint8 ROI_MODE_WP_NEXT_OFFSET = 1   # Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. [DO_SET_ROI_WPNEXT_OFFSET]~%uint8 ROI_MODE_SYSID = 2            # Mount tracks system with specified system ID [DO_SET_ROI_SYSID]~%uint8 ROI_MODE_NONE = 3             # Cancels any previous ROI setting and returns vehicle to defaults [DO_SET_ROI_NONE]~%~%uint8 gimbal_device_id  # Component ID of gimbal device to address ~%                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device~%                        # components. Send command multiple times for more than~%                        # one gimbal (but not all gimbals).  Default Mavlink gimbal ~%                        # device ids: 154, 171-175~%~%#For ROI_MODE_LOCATION~%float32 latitude~%float32 longitude~%float32 altitude        # Meters~%~%#For ROI_MODE_WP_NEXT_OFFSET~%float32 pitch_offset    # Pitch offset from next waypoint, positive pitching up~%float32 roll_offset     # Roll offset from next waypoint, positive rolling to the right~%float32 yaw_offset      # Yaw offset from next waypoint, positive yawing to the right~%~%#For ROI_MODE_SYSID~%uint8 sysid             # System ID to track (min: 1, max: 255)~%~%#ROI_MODE_NONE doesn't take extra parameters~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalManagerSetRoi-request)))
  "Returns full string definition for message of type 'GimbalManagerSetRoi-request"
  (cl:format cl:nil "# MAVLink commands: DO_SET_ROI_LOCATION, DO_SET_ROI_WPNEXT_OFFSET, DO_SET_ROI_SYSID, DO_SET_ROI_NONE~%# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_LOCATION~%# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET~%# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_SYSID~%# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_NONE~%~%uint8 mode              # enumerator to indicate ROI mode setting - see ROI_MODE~%#ROI_MODE~%uint8 ROI_MODE_LOCATION = 0         # Sets the region of interest (ROI) to a location. [DO_SET_ROI_LOCATION]~%uint8 ROI_MODE_WP_NEXT_OFFSET = 1   # Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. [DO_SET_ROI_WPNEXT_OFFSET]~%uint8 ROI_MODE_SYSID = 2            # Mount tracks system with specified system ID [DO_SET_ROI_SYSID]~%uint8 ROI_MODE_NONE = 3             # Cancels any previous ROI setting and returns vehicle to defaults [DO_SET_ROI_NONE]~%~%uint8 gimbal_device_id  # Component ID of gimbal device to address ~%                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device~%                        # components. Send command multiple times for more than~%                        # one gimbal (but not all gimbals).  Default Mavlink gimbal ~%                        # device ids: 154, 171-175~%~%#For ROI_MODE_LOCATION~%float32 latitude~%float32 longitude~%float32 altitude        # Meters~%~%#For ROI_MODE_WP_NEXT_OFFSET~%float32 pitch_offset    # Pitch offset from next waypoint, positive pitching up~%float32 roll_offset     # Roll offset from next waypoint, positive rolling to the right~%float32 yaw_offset      # Yaw offset from next waypoint, positive yawing to the right~%~%#For ROI_MODE_SYSID~%uint8 sysid             # System ID to track (min: 1, max: 255)~%~%#ROI_MODE_NONE doesn't take extra parameters~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalManagerSetRoi-request>))
  (cl:+ 0
     1
     1
     4
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalManagerSetRoi-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalManagerSetRoi-request
    (cl:cons ':mode (mode msg))
    (cl:cons ':gimbal_device_id (gimbal_device_id msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':altitude (altitude msg))
    (cl:cons ':pitch_offset (pitch_offset msg))
    (cl:cons ':roll_offset (roll_offset msg))
    (cl:cons ':yaw_offset (yaw_offset msg))
    (cl:cons ':sysid (sysid msg))
))
;//! \htmlinclude GimbalManagerSetRoi-response.msg.html

(cl:defclass <GimbalManagerSetRoi-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GimbalManagerSetRoi-response (<GimbalManagerSetRoi-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalManagerSetRoi-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalManagerSetRoi-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-srv:<GimbalManagerSetRoi-response> is deprecated: use mavros_msgs-srv:GimbalManagerSetRoi-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GimbalManagerSetRoi-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:success-val is deprecated.  Use mavros_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <GimbalManagerSetRoi-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:result-val is deprecated.  Use mavros_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalManagerSetRoi-response>) ostream)
  "Serializes a message object of type '<GimbalManagerSetRoi-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalManagerSetRoi-response>) istream)
  "Deserializes a message object of type '<GimbalManagerSetRoi-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalManagerSetRoi-response>)))
  "Returns string type for a service object of type '<GimbalManagerSetRoi-response>"
  "mavros_msgs/GimbalManagerSetRoiResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerSetRoi-response)))
  "Returns string type for a service object of type 'GimbalManagerSetRoi-response"
  "mavros_msgs/GimbalManagerSetRoiResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalManagerSetRoi-response>)))
  "Returns md5sum for a message object of type '<GimbalManagerSetRoi-response>"
  "0796c6a6cf0e18b0d829681b29f0828a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalManagerSetRoi-response)))
  "Returns md5sum for a message object of type 'GimbalManagerSetRoi-response"
  "0796c6a6cf0e18b0d829681b29f0828a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalManagerSetRoi-response>)))
  "Returns full string definition for message of type '<GimbalManagerSetRoi-response>"
  (cl:format cl:nil "bool success~%# raw result returned by COMMAND_ACK~%uint8 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalManagerSetRoi-response)))
  "Returns full string definition for message of type 'GimbalManagerSetRoi-response"
  (cl:format cl:nil "bool success~%# raw result returned by COMMAND_ACK~%uint8 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalManagerSetRoi-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalManagerSetRoi-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalManagerSetRoi-response
    (cl:cons ':success (success msg))
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GimbalManagerSetRoi)))
  'GimbalManagerSetRoi-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GimbalManagerSetRoi)))
  'GimbalManagerSetRoi-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerSetRoi)))
  "Returns string type for a service object of type '<GimbalManagerSetRoi>"
  "mavros_msgs/GimbalManagerSetRoi")