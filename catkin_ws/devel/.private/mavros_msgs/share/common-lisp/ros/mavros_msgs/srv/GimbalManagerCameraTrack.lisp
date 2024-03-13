; Auto-generated. Do not edit!


(cl:in-package mavros_msgs-srv)


;//! \htmlinclude GimbalManagerCameraTrack-request.msg.html

(cl:defclass <GimbalManagerCameraTrack-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0)
   (top_left_x
    :reader top_left_x
    :initarg :top_left_x
    :type cl:float
    :initform 0.0)
   (top_left_y
    :reader top_left_y
    :initarg :top_left_y
    :type cl:float
    :initform 0.0)
   (bottom_right_x
    :reader bottom_right_x
    :initarg :bottom_right_x
    :type cl:float
    :initform 0.0)
   (bottom_right_y
    :reader bottom_right_y
    :initarg :bottom_right_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass GimbalManagerCameraTrack-request (<GimbalManagerCameraTrack-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalManagerCameraTrack-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalManagerCameraTrack-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-srv:<GimbalManagerCameraTrack-request> is deprecated: use mavros_msgs-srv:GimbalManagerCameraTrack-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <GimbalManagerCameraTrack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:mode-val is deprecated.  Use mavros_msgs-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <GimbalManagerCameraTrack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:x-val is deprecated.  Use mavros_msgs-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <GimbalManagerCameraTrack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:y-val is deprecated.  Use mavros_msgs-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <GimbalManagerCameraTrack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:radius-val is deprecated.  Use mavros_msgs-srv:radius instead.")
  (radius m))

(cl:ensure-generic-function 'top_left_x-val :lambda-list '(m))
(cl:defmethod top_left_x-val ((m <GimbalManagerCameraTrack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:top_left_x-val is deprecated.  Use mavros_msgs-srv:top_left_x instead.")
  (top_left_x m))

(cl:ensure-generic-function 'top_left_y-val :lambda-list '(m))
(cl:defmethod top_left_y-val ((m <GimbalManagerCameraTrack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:top_left_y-val is deprecated.  Use mavros_msgs-srv:top_left_y instead.")
  (top_left_y m))

(cl:ensure-generic-function 'bottom_right_x-val :lambda-list '(m))
(cl:defmethod bottom_right_x-val ((m <GimbalManagerCameraTrack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:bottom_right_x-val is deprecated.  Use mavros_msgs-srv:bottom_right_x instead.")
  (bottom_right_x m))

(cl:ensure-generic-function 'bottom_right_y-val :lambda-list '(m))
(cl:defmethod bottom_right_y-val ((m <GimbalManagerCameraTrack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:bottom_right_y-val is deprecated.  Use mavros_msgs-srv:bottom_right_y instead.")
  (bottom_right_y m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GimbalManagerCameraTrack-request>)))
    "Constants for message type '<GimbalManagerCameraTrack-request>"
  '((:CAMERA_TRACK_MODE_POINT . 0)
    (:CAMERA_TRACK_MODE_RECTANGLE . 1)
    (:CAMERA_TRACK_MODE_STOP_TRACKING . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GimbalManagerCameraTrack-request)))
    "Constants for message type 'GimbalManagerCameraTrack-request"
  '((:CAMERA_TRACK_MODE_POINT . 0)
    (:CAMERA_TRACK_MODE_RECTANGLE . 1)
    (:CAMERA_TRACK_MODE_STOP_TRACKING . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalManagerCameraTrack-request>) ostream)
  "Serializes a message object of type '<GimbalManagerCameraTrack-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'top_left_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'top_left_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bottom_right_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bottom_right_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalManagerCameraTrack-request>) istream)
  "Deserializes a message object of type '<GimbalManagerCameraTrack-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'top_left_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'top_left_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bottom_right_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bottom_right_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalManagerCameraTrack-request>)))
  "Returns string type for a service object of type '<GimbalManagerCameraTrack-request>"
  "mavros_msgs/GimbalManagerCameraTrackRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerCameraTrack-request)))
  "Returns string type for a service object of type 'GimbalManagerCameraTrack-request"
  "mavros_msgs/GimbalManagerCameraTrackRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalManagerCameraTrack-request>)))
  "Returns md5sum for a message object of type '<GimbalManagerCameraTrack-request>"
  "b75e6fe30849c24c4458c6930214e491")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalManagerCameraTrack-request)))
  "Returns md5sum for a message object of type 'GimbalManagerCameraTrack-request"
  "b75e6fe30849c24c4458c6930214e491")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalManagerCameraTrack-request>)))
  "Returns full string definition for message of type '<GimbalManagerCameraTrack-request>"
  (cl:format cl:nil "# MAVLink commands: CAMERA_TRACK_POINT, CAMERA_TRACK_RECTANGLE, CAMERA_STOP_TRACKING~%# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_POINT~%# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_RECTANGLE~%# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_STOP_TRACKING~%~%uint8 mode      # enumerator to indicate camera track mode setting - see CAMERA_TRACK_MODE~%#CAMERA_TRACK_MODE~%uint8 CAMERA_TRACK_MODE_POINT = 0           # If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking. [CAMERA_TRACK_POINT]~%uint8 CAMERA_TRACK_MODE_RECTANGLE = 1       # If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking. [CAMERA_TRACK_RECTANGLE]~%uint8 CAMERA_TRACK_MODE_STOP_TRACKING = 2   # Stops ongoing tracking. [CAMERA_STOP_TRACKING]~%~%#For CAMERA_TRACK_POINT~%float32 x       # Point to track x value (normalized 0..1, 0 is left, 1 is right).~%float32 y       # Point to track y value (normalized 0..1, 0 is top, 1 is bottom).~%float32 radius  # Point radius (normalized 0..1, 0 is image left, 1 is image right).~%~%#For CAMERA_TRACK_RECTANGLE~%float32 top_left_x      # Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).~%float32 top_left_y      # Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).~%float32 bottom_right_x  # Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).~%float32 bottom_right_y  # Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).~%~%#CAMERA_STOP_TRACKING doesn't take extra parameters~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalManagerCameraTrack-request)))
  "Returns full string definition for message of type 'GimbalManagerCameraTrack-request"
  (cl:format cl:nil "# MAVLink commands: CAMERA_TRACK_POINT, CAMERA_TRACK_RECTANGLE, CAMERA_STOP_TRACKING~%# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_POINT~%# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_RECTANGLE~%# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_STOP_TRACKING~%~%uint8 mode      # enumerator to indicate camera track mode setting - see CAMERA_TRACK_MODE~%#CAMERA_TRACK_MODE~%uint8 CAMERA_TRACK_MODE_POINT = 0           # If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking. [CAMERA_TRACK_POINT]~%uint8 CAMERA_TRACK_MODE_RECTANGLE = 1       # If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking. [CAMERA_TRACK_RECTANGLE]~%uint8 CAMERA_TRACK_MODE_STOP_TRACKING = 2   # Stops ongoing tracking. [CAMERA_STOP_TRACKING]~%~%#For CAMERA_TRACK_POINT~%float32 x       # Point to track x value (normalized 0..1, 0 is left, 1 is right).~%float32 y       # Point to track y value (normalized 0..1, 0 is top, 1 is bottom).~%float32 radius  # Point radius (normalized 0..1, 0 is image left, 1 is image right).~%~%#For CAMERA_TRACK_RECTANGLE~%float32 top_left_x      # Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).~%float32 top_left_y      # Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).~%float32 bottom_right_x  # Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).~%float32 bottom_right_y  # Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).~%~%#CAMERA_STOP_TRACKING doesn't take extra parameters~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalManagerCameraTrack-request>))
  (cl:+ 0
     1
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalManagerCameraTrack-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalManagerCameraTrack-request
    (cl:cons ':mode (mode msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':radius (radius msg))
    (cl:cons ':top_left_x (top_left_x msg))
    (cl:cons ':top_left_y (top_left_y msg))
    (cl:cons ':bottom_right_x (bottom_right_x msg))
    (cl:cons ':bottom_right_y (bottom_right_y msg))
))
;//! \htmlinclude GimbalManagerCameraTrack-response.msg.html

(cl:defclass <GimbalManagerCameraTrack-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass GimbalManagerCameraTrack-response (<GimbalManagerCameraTrack-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalManagerCameraTrack-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalManagerCameraTrack-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-srv:<GimbalManagerCameraTrack-response> is deprecated: use mavros_msgs-srv:GimbalManagerCameraTrack-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GimbalManagerCameraTrack-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:success-val is deprecated.  Use mavros_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <GimbalManagerCameraTrack-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:result-val is deprecated.  Use mavros_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalManagerCameraTrack-response>) ostream)
  "Serializes a message object of type '<GimbalManagerCameraTrack-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalManagerCameraTrack-response>) istream)
  "Deserializes a message object of type '<GimbalManagerCameraTrack-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalManagerCameraTrack-response>)))
  "Returns string type for a service object of type '<GimbalManagerCameraTrack-response>"
  "mavros_msgs/GimbalManagerCameraTrackResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerCameraTrack-response)))
  "Returns string type for a service object of type 'GimbalManagerCameraTrack-response"
  "mavros_msgs/GimbalManagerCameraTrackResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalManagerCameraTrack-response>)))
  "Returns md5sum for a message object of type '<GimbalManagerCameraTrack-response>"
  "b75e6fe30849c24c4458c6930214e491")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalManagerCameraTrack-response)))
  "Returns md5sum for a message object of type 'GimbalManagerCameraTrack-response"
  "b75e6fe30849c24c4458c6930214e491")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalManagerCameraTrack-response>)))
  "Returns full string definition for message of type '<GimbalManagerCameraTrack-response>"
  (cl:format cl:nil "bool success~%# raw result returned by COMMAND_ACK~%uint8 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalManagerCameraTrack-response)))
  "Returns full string definition for message of type 'GimbalManagerCameraTrack-response"
  (cl:format cl:nil "bool success~%# raw result returned by COMMAND_ACK~%uint8 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalManagerCameraTrack-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalManagerCameraTrack-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalManagerCameraTrack-response
    (cl:cons ':success (success msg))
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GimbalManagerCameraTrack)))
  'GimbalManagerCameraTrack-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GimbalManagerCameraTrack)))
  'GimbalManagerCameraTrack-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalManagerCameraTrack)))
  "Returns string type for a service object of type '<GimbalManagerCameraTrack>"
  "mavros_msgs/GimbalManagerCameraTrack")