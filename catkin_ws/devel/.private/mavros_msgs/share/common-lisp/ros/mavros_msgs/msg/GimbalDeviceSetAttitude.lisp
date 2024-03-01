; Auto-generated. Do not edit!


(cl:in-package mavros_msgs-msg)


;//! \htmlinclude GimbalDeviceSetAttitude.msg.html

(cl:defclass <GimbalDeviceSetAttitude> (roslisp-msg-protocol:ros-message)
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
    :initform 0.0))
)

(cl:defclass GimbalDeviceSetAttitude (<GimbalDeviceSetAttitude>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalDeviceSetAttitude>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalDeviceSetAttitude)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-msg:<GimbalDeviceSetAttitude> is deprecated: use mavros_msgs-msg:GimbalDeviceSetAttitude instead.")))

(cl:ensure-generic-function 'target_system-val :lambda-list '(m))
(cl:defmethod target_system-val ((m <GimbalDeviceSetAttitude>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:target_system-val is deprecated.  Use mavros_msgs-msg:target_system instead.")
  (target_system m))

(cl:ensure-generic-function 'target_component-val :lambda-list '(m))
(cl:defmethod target_component-val ((m <GimbalDeviceSetAttitude>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:target_component-val is deprecated.  Use mavros_msgs-msg:target_component instead.")
  (target_component m))

(cl:ensure-generic-function 'flags-val :lambda-list '(m))
(cl:defmethod flags-val ((m <GimbalDeviceSetAttitude>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:flags-val is deprecated.  Use mavros_msgs-msg:flags instead.")
  (flags m))

(cl:ensure-generic-function 'q-val :lambda-list '(m))
(cl:defmethod q-val ((m <GimbalDeviceSetAttitude>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:q-val is deprecated.  Use mavros_msgs-msg:q instead.")
  (q m))

(cl:ensure-generic-function 'angular_velocity_x-val :lambda-list '(m))
(cl:defmethod angular_velocity_x-val ((m <GimbalDeviceSetAttitude>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:angular_velocity_x-val is deprecated.  Use mavros_msgs-msg:angular_velocity_x instead.")
  (angular_velocity_x m))

(cl:ensure-generic-function 'angular_velocity_y-val :lambda-list '(m))
(cl:defmethod angular_velocity_y-val ((m <GimbalDeviceSetAttitude>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:angular_velocity_y-val is deprecated.  Use mavros_msgs-msg:angular_velocity_y instead.")
  (angular_velocity_y m))

(cl:ensure-generic-function 'angular_velocity_z-val :lambda-list '(m))
(cl:defmethod angular_velocity_z-val ((m <GimbalDeviceSetAttitude>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:angular_velocity_z-val is deprecated.  Use mavros_msgs-msg:angular_velocity_z instead.")
  (angular_velocity_z m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GimbalDeviceSetAttitude>)))
    "Constants for message type '<GimbalDeviceSetAttitude>"
  '((:GIMBAL_DEVICE_FLAGS_RETRACT . 1)
    (:GIMBAL_DEVICE_FLAGS_NEUTRAL . 2)
    (:GIMBAL_DEVICE_FLAGS_ROLL_LOCK . 4)
    (:GIMBAL_DEVICE_FLAGS_PITCH_LOCK . 8)
    (:GIMBAL_DEVICE_FLAGS_YAW_LOCK . 16))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GimbalDeviceSetAttitude)))
    "Constants for message type 'GimbalDeviceSetAttitude"
  '((:GIMBAL_DEVICE_FLAGS_RETRACT . 1)
    (:GIMBAL_DEVICE_FLAGS_NEUTRAL . 2)
    (:GIMBAL_DEVICE_FLAGS_ROLL_LOCK . 4)
    (:GIMBAL_DEVICE_FLAGS_PITCH_LOCK . 8)
    (:GIMBAL_DEVICE_FLAGS_YAW_LOCK . 16))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalDeviceSetAttitude>) ostream)
  "Serializes a message object of type '<GimbalDeviceSetAttitude>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalDeviceSetAttitude>) istream)
  "Deserializes a message object of type '<GimbalDeviceSetAttitude>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalDeviceSetAttitude>)))
  "Returns string type for a message object of type '<GimbalDeviceSetAttitude>"
  "mavros_msgs/GimbalDeviceSetAttitude")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalDeviceSetAttitude)))
  "Returns string type for a message object of type 'GimbalDeviceSetAttitude"
  "mavros_msgs/GimbalDeviceSetAttitude")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalDeviceSetAttitude>)))
  "Returns md5sum for a message object of type '<GimbalDeviceSetAttitude>"
  "87a55ddad0e67129d73063fdc10f2ffb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalDeviceSetAttitude)))
  "Returns md5sum for a message object of type 'GimbalDeviceSetAttitude"
  "87a55ddad0e67129d73063fdc10f2ffb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalDeviceSetAttitude>)))
  "Returns full string definition for message of type '<GimbalDeviceSetAttitude>"
  (cl:format cl:nil "# MAVLink message: GIMBAL_DEVICE_SET_ATTITUDE~%# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_SET_ATTITUDE~%~%uint8 target_system         # System ID~%uint8 target_component      # Component ID~%~%uint16 flags                # Low level gimbal flags (bitwise) - See GIMBAL_DEVICE_FLAGS~%#GIMBAL_DEVICE_FLAGS~%uint16 GIMBAL_DEVICE_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT~%uint16 GIMBAL_DEVICE_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL~%uint16 GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK~%uint16 GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK~%uint16 GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK~%~%geometry_msgs/Quaternion q # Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)~%float32 angular_velocity_x # X component of angular velocity, positive is rolling to the right, NaN to be ignored.~%float32 angular_velocity_y # Y component of angular velocity, positive is pitching up, NaN to be ignored.~%float32 angular_velocity_z # Z component of angular velocity, positive is yawing to the right, NaN to be ignored.~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalDeviceSetAttitude)))
  "Returns full string definition for message of type 'GimbalDeviceSetAttitude"
  (cl:format cl:nil "# MAVLink message: GIMBAL_DEVICE_SET_ATTITUDE~%# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_SET_ATTITUDE~%~%uint8 target_system         # System ID~%uint8 target_component      # Component ID~%~%uint16 flags                # Low level gimbal flags (bitwise) - See GIMBAL_DEVICE_FLAGS~%#GIMBAL_DEVICE_FLAGS~%uint16 GIMBAL_DEVICE_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT~%uint16 GIMBAL_DEVICE_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL~%uint16 GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK~%uint16 GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK~%uint16 GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK~%~%geometry_msgs/Quaternion q # Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)~%float32 angular_velocity_x # X component of angular velocity, positive is rolling to the right, NaN to be ignored.~%float32 angular_velocity_y # Y component of angular velocity, positive is pitching up, NaN to be ignored.~%float32 angular_velocity_z # Z component of angular velocity, positive is yawing to the right, NaN to be ignored.~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalDeviceSetAttitude>))
  (cl:+ 0
     1
     1
     2
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'q))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalDeviceSetAttitude>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalDeviceSetAttitude
    (cl:cons ':target_system (target_system msg))
    (cl:cons ':target_component (target_component msg))
    (cl:cons ':flags (flags msg))
    (cl:cons ':q (q msg))
    (cl:cons ':angular_velocity_x (angular_velocity_x msg))
    (cl:cons ':angular_velocity_y (angular_velocity_y msg))
    (cl:cons ':angular_velocity_z (angular_velocity_z msg))
))
