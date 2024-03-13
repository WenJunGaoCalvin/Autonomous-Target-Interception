; Auto-generated. Do not edit!


(cl:in-package mavros_msgs-msg)


;//! \htmlinclude GimbalDeviceInformation.msg.html

(cl:defclass <GimbalDeviceInformation> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (vendor_name
    :reader vendor_name
    :initarg :vendor_name
    :type cl:string
    :initform "")
   (model_name
    :reader model_name
    :initarg :model_name
    :type cl:string
    :initform "")
   (custom_name
    :reader custom_name
    :initarg :custom_name
    :type cl:string
    :initform "")
   (firmware_version
    :reader firmware_version
    :initarg :firmware_version
    :type cl:integer
    :initform 0)
   (hardware_version
    :reader hardware_version
    :initarg :hardware_version
    :type cl:integer
    :initform 0)
   (uid
    :reader uid
    :initarg :uid
    :type cl:integer
    :initform 0)
   (cap_flags
    :reader cap_flags
    :initarg :cap_flags
    :type cl:integer
    :initform 0)
   (custom_cap_flags
    :reader custom_cap_flags
    :initarg :custom_cap_flags
    :type cl:fixnum
    :initform 0)
   (roll_min
    :reader roll_min
    :initarg :roll_min
    :type cl:float
    :initform 0.0)
   (roll_max
    :reader roll_max
    :initarg :roll_max
    :type cl:float
    :initform 0.0)
   (pitch_min
    :reader pitch_min
    :initarg :pitch_min
    :type cl:float
    :initform 0.0)
   (pitch_max
    :reader pitch_max
    :initarg :pitch_max
    :type cl:float
    :initform 0.0)
   (yaw_min
    :reader yaw_min
    :initarg :yaw_min
    :type cl:float
    :initform 0.0)
   (yaw_max
    :reader yaw_max
    :initarg :yaw_max
    :type cl:float
    :initform 0.0))
)

(cl:defclass GimbalDeviceInformation (<GimbalDeviceInformation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalDeviceInformation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalDeviceInformation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-msg:<GimbalDeviceInformation> is deprecated: use mavros_msgs-msg:GimbalDeviceInformation instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:header-val is deprecated.  Use mavros_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'vendor_name-val :lambda-list '(m))
(cl:defmethod vendor_name-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:vendor_name-val is deprecated.  Use mavros_msgs-msg:vendor_name instead.")
  (vendor_name m))

(cl:ensure-generic-function 'model_name-val :lambda-list '(m))
(cl:defmethod model_name-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:model_name-val is deprecated.  Use mavros_msgs-msg:model_name instead.")
  (model_name m))

(cl:ensure-generic-function 'custom_name-val :lambda-list '(m))
(cl:defmethod custom_name-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:custom_name-val is deprecated.  Use mavros_msgs-msg:custom_name instead.")
  (custom_name m))

(cl:ensure-generic-function 'firmware_version-val :lambda-list '(m))
(cl:defmethod firmware_version-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:firmware_version-val is deprecated.  Use mavros_msgs-msg:firmware_version instead.")
  (firmware_version m))

(cl:ensure-generic-function 'hardware_version-val :lambda-list '(m))
(cl:defmethod hardware_version-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:hardware_version-val is deprecated.  Use mavros_msgs-msg:hardware_version instead.")
  (hardware_version m))

(cl:ensure-generic-function 'uid-val :lambda-list '(m))
(cl:defmethod uid-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:uid-val is deprecated.  Use mavros_msgs-msg:uid instead.")
  (uid m))

(cl:ensure-generic-function 'cap_flags-val :lambda-list '(m))
(cl:defmethod cap_flags-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:cap_flags-val is deprecated.  Use mavros_msgs-msg:cap_flags instead.")
  (cap_flags m))

(cl:ensure-generic-function 'custom_cap_flags-val :lambda-list '(m))
(cl:defmethod custom_cap_flags-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:custom_cap_flags-val is deprecated.  Use mavros_msgs-msg:custom_cap_flags instead.")
  (custom_cap_flags m))

(cl:ensure-generic-function 'roll_min-val :lambda-list '(m))
(cl:defmethod roll_min-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:roll_min-val is deprecated.  Use mavros_msgs-msg:roll_min instead.")
  (roll_min m))

(cl:ensure-generic-function 'roll_max-val :lambda-list '(m))
(cl:defmethod roll_max-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:roll_max-val is deprecated.  Use mavros_msgs-msg:roll_max instead.")
  (roll_max m))

(cl:ensure-generic-function 'pitch_min-val :lambda-list '(m))
(cl:defmethod pitch_min-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:pitch_min-val is deprecated.  Use mavros_msgs-msg:pitch_min instead.")
  (pitch_min m))

(cl:ensure-generic-function 'pitch_max-val :lambda-list '(m))
(cl:defmethod pitch_max-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:pitch_max-val is deprecated.  Use mavros_msgs-msg:pitch_max instead.")
  (pitch_max m))

(cl:ensure-generic-function 'yaw_min-val :lambda-list '(m))
(cl:defmethod yaw_min-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:yaw_min-val is deprecated.  Use mavros_msgs-msg:yaw_min instead.")
  (yaw_min m))

(cl:ensure-generic-function 'yaw_max-val :lambda-list '(m))
(cl:defmethod yaw_max-val ((m <GimbalDeviceInformation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-msg:yaw_max-val is deprecated.  Use mavros_msgs-msg:yaw_max instead.")
  (yaw_max m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GimbalDeviceInformation>)))
    "Constants for message type '<GimbalDeviceInformation>"
  '((:GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT . 1)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL . 2)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS . 4)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW . 8)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK . 16)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS . 32)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW . 64)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK . 128)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS . 256)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW . 512)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK . 1024)
    (:GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW . 2048))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GimbalDeviceInformation)))
    "Constants for message type 'GimbalDeviceInformation"
  '((:GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT . 1)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL . 2)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS . 4)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW . 8)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK . 16)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS . 32)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW . 64)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK . 128)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS . 256)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW . 512)
    (:GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK . 1024)
    (:GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW . 2048))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalDeviceInformation>) ostream)
  "Serializes a message object of type '<GimbalDeviceInformation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'vendor_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'vendor_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'custom_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'custom_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'firmware_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'firmware_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'firmware_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'firmware_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hardware_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hardware_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'hardware_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'hardware_version)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'uid)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cap_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cap_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cap_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cap_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_cap_flags)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_cap_flags)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw_max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalDeviceInformation>) istream)
  "Deserializes a message object of type '<GimbalDeviceInformation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vendor_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'vendor_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'custom_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'custom_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'firmware_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'firmware_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'firmware_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'firmware_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hardware_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hardware_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'hardware_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'hardware_version)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'uid)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cap_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cap_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cap_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cap_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'custom_cap_flags)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'custom_cap_flags)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_min) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll_max) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_min) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch_max) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_min) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw_max) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalDeviceInformation>)))
  "Returns string type for a message object of type '<GimbalDeviceInformation>"
  "mavros_msgs/GimbalDeviceInformation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalDeviceInformation)))
  "Returns string type for a message object of type 'GimbalDeviceInformation"
  "mavros_msgs/GimbalDeviceInformation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalDeviceInformation>)))
  "Returns md5sum for a message object of type '<GimbalDeviceInformation>"
  "cf9937985a9347aa15974b9c7eb2fa20")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalDeviceInformation)))
  "Returns md5sum for a message object of type 'GimbalDeviceInformation"
  "cf9937985a9347aa15974b9c7eb2fa20")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalDeviceInformation>)))
  "Returns full string definition for message of type '<GimbalDeviceInformation>"
  (cl:format cl:nil "# MAVLink message: GIMBAL_DEVICE_INFORMATION~%# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION~%~%std_msgs/Header header~%~%string vendor_name # Name of the gimbal vendor.~%string model_name # Name of the gimbal model.~%string custom_name # Custom name of the gimbal given to it by the user.~%uint32 firmware_version # Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).~%uint32 hardware_version # Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).~%uint64 uid # UID of gimbal hardware (0 if unknown).~%~%uint32 cap_flags # Bitmap of gimbal capability flags - see GIMBAL_DEVICE_CAP_FLAGS~%#GIMBAL_DEVICE_CAP_FLAGS~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT = 1 # Gimbal device supports a retracted position~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL = 2 # Gimbal device supports a horizontal, forward looking position, stabilized~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS = 4 # Gimbal device supports rotating around roll axis.~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW = 8 # Gimbal device supports to follow a roll angle relative to the vehicle~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK = 16 # Gimbal device supports locking to an roll angle (generally that's the default with roll stabilized)~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS = 32 # Gimbal device supports rotating around pitch axis.~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW = 64 # Gimbal device supports to follow a pitch angle relative to the vehicle~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK = 128 # Gimbal device supports locking to an pitch angle (generally that's the default with pitch stabilized)~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS = 256 # Gimbal device supports rotating around yaw axis.~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW = 512 # Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default)~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK = 1024 # Gimbal device supports locking to an absolute heading (often this is an option available)~%uint32 GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048 # Gimbal device supports yawing/panning infinetely (e.g. using slip disk).~%~%uint16 custom_cap_flags # Bitmap for use for gimbal-specific capability flags.~%float32 roll_min # Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)~%float32 roll_max # Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)~%float32 pitch_min # Minimum pitch angle (positive: up, negative: down)~%float32 pitch_max # Maximum pitch angle (positive: up, negative: down)~%float32 yaw_min # Minimum yaw angle (positive: to the right, negative: to the left)~%float32 yaw_max # Maximum yaw angle (positive: to the right, negative: to the left)~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalDeviceInformation)))
  "Returns full string definition for message of type 'GimbalDeviceInformation"
  (cl:format cl:nil "# MAVLink message: GIMBAL_DEVICE_INFORMATION~%# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION~%~%std_msgs/Header header~%~%string vendor_name # Name of the gimbal vendor.~%string model_name # Name of the gimbal model.~%string custom_name # Custom name of the gimbal given to it by the user.~%uint32 firmware_version # Version of the gimbal firmware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).~%uint32 hardware_version # Version of the gimbal hardware, encoded as: (Dev & 0xff) << 24 | (Patch & 0xff) << 16 | (Minor & 0xff) << 8 | (Major & 0xff).~%uint64 uid # UID of gimbal hardware (0 if unknown).~%~%uint32 cap_flags # Bitmap of gimbal capability flags - see GIMBAL_DEVICE_CAP_FLAGS~%#GIMBAL_DEVICE_CAP_FLAGS~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT = 1 # Gimbal device supports a retracted position~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL = 2 # Gimbal device supports a horizontal, forward looking position, stabilized~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS = 4 # Gimbal device supports rotating around roll axis.~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW = 8 # Gimbal device supports to follow a roll angle relative to the vehicle~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK = 16 # Gimbal device supports locking to an roll angle (generally that's the default with roll stabilized)~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS = 32 # Gimbal device supports rotating around pitch axis.~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW = 64 # Gimbal device supports to follow a pitch angle relative to the vehicle~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK = 128 # Gimbal device supports locking to an pitch angle (generally that's the default with pitch stabilized)~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS = 256 # Gimbal device supports rotating around yaw axis.~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW = 512 # Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default)~%uint32 GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK = 1024 # Gimbal device supports locking to an absolute heading (often this is an option available)~%uint32 GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048 # Gimbal device supports yawing/panning infinetely (e.g. using slip disk).~%~%uint16 custom_cap_flags # Bitmap for use for gimbal-specific capability flags.~%float32 roll_min # Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)~%float32 roll_max # Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)~%float32 pitch_min # Minimum pitch angle (positive: up, negative: down)~%float32 pitch_max # Maximum pitch angle (positive: up, negative: down)~%float32 yaw_min # Minimum yaw angle (positive: to the right, negative: to the left)~%float32 yaw_max # Maximum yaw angle (positive: to the right, negative: to the left)~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalDeviceInformation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'vendor_name))
     4 (cl:length (cl:slot-value msg 'model_name))
     4 (cl:length (cl:slot-value msg 'custom_name))
     4
     4
     8
     4
     2
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalDeviceInformation>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalDeviceInformation
    (cl:cons ':header (header msg))
    (cl:cons ':vendor_name (vendor_name msg))
    (cl:cons ':model_name (model_name msg))
    (cl:cons ':custom_name (custom_name msg))
    (cl:cons ':firmware_version (firmware_version msg))
    (cl:cons ':hardware_version (hardware_version msg))
    (cl:cons ':uid (uid msg))
    (cl:cons ':cap_flags (cap_flags msg))
    (cl:cons ':custom_cap_flags (custom_cap_flags msg))
    (cl:cons ':roll_min (roll_min msg))
    (cl:cons ':roll_max (roll_max msg))
    (cl:cons ':pitch_min (pitch_min msg))
    (cl:cons ':pitch_max (pitch_max msg))
    (cl:cons ':yaw_min (yaw_min msg))
    (cl:cons ':yaw_max (yaw_max msg))
))
