;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::GimbalDeviceInformation)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'GimbalDeviceInformation (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION")
  (make-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude GimbalDeviceInformation.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT*" (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(shadow '*GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT* (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(defconstant mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT* 1)
(intern "*GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL*" (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(shadow '*GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL* (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(defconstant mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL* 2)
(intern "*GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS*" (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(shadow '*GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS* (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(defconstant mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS* 4)
(intern "*GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW*" (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(shadow '*GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW* (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(defconstant mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW* 8)
(intern "*GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK*" (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(shadow '*GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK* (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(defconstant mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK* 16)
(intern "*GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS*" (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(shadow '*GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS* (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(defconstant mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS* 32)
(intern "*GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW*" (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(shadow '*GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW* (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(defconstant mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW* 64)
(intern "*GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK*" (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(shadow '*GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK* (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(defconstant mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK* 128)
(intern "*GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS*" (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(shadow '*GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS* (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(defconstant mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS* 256)
(intern "*GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW*" (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(shadow '*GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW* (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(defconstant mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW* 512)
(intern "*GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK*" (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(shadow '*GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK* (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(defconstant mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK* 1024)
(intern "*GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW*" (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(shadow '*GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW* (find-package "MAVROS_MSGS::GIMBALDEVICEINFORMATION"))
(defconstant mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW* 2048)

(defun mavros_msgs::GimbalDeviceInformation-to-symbol (const)
  (cond
        ((= const 1) 'mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT*)
        ((= const 2) 'mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL*)
        ((= const 4) 'mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS*)
        ((= const 8) 'mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW*)
        ((= const 16) 'mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK*)
        ((= const 32) 'mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS*)
        ((= const 64) 'mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW*)
        ((= const 128) 'mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK*)
        ((= const 256) 'mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS*)
        ((= const 512) 'mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW*)
        ((= const 1024) 'mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK*)
        ((= const 2048) 'mavros_msgs::GimbalDeviceInformation::*GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW*)
        (t nil)))

(defclass mavros_msgs::GimbalDeviceInformation
  :super ros::object
  :slots (_header _vendor_name _model_name _custom_name _firmware_version _hardware_version _uid _cap_flags _custom_cap_flags _roll_min _roll_max _pitch_min _pitch_max _yaw_min _yaw_max ))

(defmethod mavros_msgs::GimbalDeviceInformation
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:vendor_name __vendor_name) "")
    ((:model_name __model_name) "")
    ((:custom_name __custom_name) "")
    ((:firmware_version __firmware_version) 0)
    ((:hardware_version __hardware_version) 0)
    ((:uid __uid) 0)
    ((:cap_flags __cap_flags) 0)
    ((:custom_cap_flags __custom_cap_flags) 0)
    ((:roll_min __roll_min) 0.0)
    ((:roll_max __roll_max) 0.0)
    ((:pitch_min __pitch_min) 0.0)
    ((:pitch_max __pitch_max) 0.0)
    ((:yaw_min __yaw_min) 0.0)
    ((:yaw_max __yaw_max) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _vendor_name (string __vendor_name))
   (setq _model_name (string __model_name))
   (setq _custom_name (string __custom_name))
   (setq _firmware_version (round __firmware_version))
   (setq _hardware_version (round __hardware_version))
   (setq _uid (round __uid))
   (setq _cap_flags (round __cap_flags))
   (setq _custom_cap_flags (round __custom_cap_flags))
   (setq _roll_min (float __roll_min))
   (setq _roll_max (float __roll_max))
   (setq _pitch_min (float __pitch_min))
   (setq _pitch_max (float __pitch_max))
   (setq _yaw_min (float __yaw_min))
   (setq _yaw_max (float __yaw_max))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:vendor_name
   (&optional __vendor_name)
   (if __vendor_name (setq _vendor_name __vendor_name)) _vendor_name)
  (:model_name
   (&optional __model_name)
   (if __model_name (setq _model_name __model_name)) _model_name)
  (:custom_name
   (&optional __custom_name)
   (if __custom_name (setq _custom_name __custom_name)) _custom_name)
  (:firmware_version
   (&optional __firmware_version)
   (if __firmware_version (setq _firmware_version __firmware_version)) _firmware_version)
  (:hardware_version
   (&optional __hardware_version)
   (if __hardware_version (setq _hardware_version __hardware_version)) _hardware_version)
  (:uid
   (&optional __uid)
   (if __uid (setq _uid __uid)) _uid)
  (:cap_flags
   (&optional __cap_flags)
   (if __cap_flags (setq _cap_flags __cap_flags)) _cap_flags)
  (:custom_cap_flags
   (&optional __custom_cap_flags)
   (if __custom_cap_flags (setq _custom_cap_flags __custom_cap_flags)) _custom_cap_flags)
  (:roll_min
   (&optional __roll_min)
   (if __roll_min (setq _roll_min __roll_min)) _roll_min)
  (:roll_max
   (&optional __roll_max)
   (if __roll_max (setq _roll_max __roll_max)) _roll_max)
  (:pitch_min
   (&optional __pitch_min)
   (if __pitch_min (setq _pitch_min __pitch_min)) _pitch_min)
  (:pitch_max
   (&optional __pitch_max)
   (if __pitch_max (setq _pitch_max __pitch_max)) _pitch_max)
  (:yaw_min
   (&optional __yaw_min)
   (if __yaw_min (setq _yaw_min __yaw_min)) _yaw_min)
  (:yaw_max
   (&optional __yaw_max)
   (if __yaw_max (setq _yaw_max __yaw_max)) _yaw_max)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _vendor_name
    4 (length _vendor_name)
    ;; string _model_name
    4 (length _model_name)
    ;; string _custom_name
    4 (length _custom_name)
    ;; uint32 _firmware_version
    4
    ;; uint32 _hardware_version
    4
    ;; uint64 _uid
    8
    ;; uint32 _cap_flags
    4
    ;; uint16 _custom_cap_flags
    2
    ;; float32 _roll_min
    4
    ;; float32 _roll_max
    4
    ;; float32 _pitch_min
    4
    ;; float32 _pitch_max
    4
    ;; float32 _yaw_min
    4
    ;; float32 _yaw_max
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _vendor_name
       (write-long (length _vendor_name) s) (princ _vendor_name s)
     ;; string _model_name
       (write-long (length _model_name) s) (princ _model_name s)
     ;; string _custom_name
       (write-long (length _custom_name) s) (princ _custom_name s)
     ;; uint32 _firmware_version
       (write-long _firmware_version s)
     ;; uint32 _hardware_version
       (write-long _hardware_version s)
     ;; uint64 _uid
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _uid (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _uid) (= (length (_uid . bv)) 2)) ;; bignum
              (write-long (ash (elt (_uid . bv) 0) 0) s)
              (write-long (ash (elt (_uid . bv) 1) -1) s))
             ((and (class _uid) (= (length (_uid . bv)) 1)) ;; big1
              (write-long (elt (_uid . bv) 0) s)
              (write-long (if (>= _uid 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _uid s)(write-long (if (>= _uid 0) 0 #xffffffff) s)))
     ;; uint32 _cap_flags
       (write-long _cap_flags s)
     ;; uint16 _custom_cap_flags
       (write-word _custom_cap_flags s)
     ;; float32 _roll_min
       (sys::poke _roll_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll_max
       (sys::poke _roll_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_min
       (sys::poke _pitch_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_max
       (sys::poke _pitch_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_min
       (sys::poke _yaw_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_max
       (sys::poke _yaw_max (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _vendor_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _vendor_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _model_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _custom_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _custom_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _firmware_version
     (setq _firmware_version (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _hardware_version
     (setq _hardware_version (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint64 _uid
     
#+(or :alpha :irix6 :x86_64)
      (setf _uid (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _uid (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; uint32 _cap_flags
     (setq _cap_flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _custom_cap_flags
     (setq _custom_cap_flags (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _roll_min
     (setq _roll_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll_max
     (setq _roll_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_min
     (setq _pitch_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_max
     (setq _pitch_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_min
     (setq _yaw_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_max
     (setq _yaw_max (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mavros_msgs::GimbalDeviceInformation :md5sum-) "cf9937985a9347aa15974b9c7eb2fa20")
(setf (get mavros_msgs::GimbalDeviceInformation :datatype-) "mavros_msgs/GimbalDeviceInformation")
(setf (get mavros_msgs::GimbalDeviceInformation :definition-)
      "# MAVLink message: GIMBAL_DEVICE_INFORMATION
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

")



(provide :mavros_msgs/GimbalDeviceInformation "cf9937985a9347aa15974b9c7eb2fa20")


