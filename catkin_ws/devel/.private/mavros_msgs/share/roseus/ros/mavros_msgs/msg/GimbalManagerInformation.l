;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::GimbalManagerInformation)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'GimbalManagerInformation (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION")
  (make-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))

(in-package "ROS")
;;//! \htmlinclude GimbalManagerInformation.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT* 1)
(intern "*GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL* 2)
(intern "*GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS* 4)
(intern "*GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW* 8)
(intern "*GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK* 16)
(intern "*GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS* 32)
(intern "*GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW* 64)
(intern "*GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK* 128)
(intern "*GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS* 256)
(intern "*GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW* 512)
(intern "*GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK* 1024)
(intern "*GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW* 2048)
(intern "*GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL* 65536)
(intern "*GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL*" (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(shadow '*GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL* (find-package "MAVROS_MSGS::GIMBALMANAGERINFORMATION"))
(defconstant mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL* 131072)

(defun mavros_msgs::GimbalManagerInformation-to-symbol (const)
  (cond
        ((= const 1) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT*)
        ((= const 2) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL*)
        ((= const 4) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS*)
        ((= const 8) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW*)
        ((= const 16) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK*)
        ((= const 32) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS*)
        ((= const 64) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW*)
        ((= const 128) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK*)
        ((= const 256) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS*)
        ((= const 512) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW*)
        ((= const 1024) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK*)
        ((= const 2048) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW*)
        ((= const 65536) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL*)
        ((= const 131072) 'mavros_msgs::GimbalManagerInformation::*GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL*)
        (t nil)))

(defclass mavros_msgs::GimbalManagerInformation
  :super ros::object
  :slots (_header _cap_flags _gimbal_device_id _roll_min _roll_max _pitch_min _pitch_max _yaw_min _yaw_max ))

(defmethod mavros_msgs::GimbalManagerInformation
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cap_flags __cap_flags) 0)
    ((:gimbal_device_id __gimbal_device_id) 0)
    ((:roll_min __roll_min) 0.0)
    ((:roll_max __roll_max) 0.0)
    ((:pitch_min __pitch_min) 0.0)
    ((:pitch_max __pitch_max) 0.0)
    ((:yaw_min __yaw_min) 0.0)
    ((:yaw_max __yaw_max) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _cap_flags (round __cap_flags))
   (setq _gimbal_device_id (round __gimbal_device_id))
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
  (:cap_flags
   (&optional __cap_flags)
   (if __cap_flags (setq _cap_flags __cap_flags)) _cap_flags)
  (:gimbal_device_id
   (&optional __gimbal_device_id)
   (if __gimbal_device_id (setq _gimbal_device_id __gimbal_device_id)) _gimbal_device_id)
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
    ;; uint32 _cap_flags
    4
    ;; uint8 _gimbal_device_id
    1
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
     ;; uint32 _cap_flags
       (write-long _cap_flags s)
     ;; uint8 _gimbal_device_id
       (write-byte _gimbal_device_id s)
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
   ;; uint32 _cap_flags
     (setq _cap_flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _gimbal_device_id
     (setq _gimbal_device_id (sys::peek buf ptr- :char)) (incf ptr- 1)
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

(setf (get mavros_msgs::GimbalManagerInformation :md5sum-) "afbf4fbefec70c70e001795d309516f7")
(setf (get mavros_msgs::GimbalManagerInformation :datatype-) "mavros_msgs/GimbalManagerInformation")
(setf (get mavros_msgs::GimbalManagerInformation :definition-)
      "# MAVLink message: GIMBAL_MANAGER_INFORMATION
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION

std_msgs/Header header

uint32 cap_flags # Bitmap of gimbal capability flags - see GIMBAL_MANAGER_CAP_FLAGS
#GIMBAL_MANAGER_CAP_FLAGS
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT = 1 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT.
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL = 2 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL.
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS = 4 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS.
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW = 8 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW.
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK = 16 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK.
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS = 32 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS.
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW = 64 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW.
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK = 128 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK.
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS = 256 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS.
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW = 512 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW.
uint32 GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK = 1024 # Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK.
uint32 GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048 # Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW.
uint32 GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL = 65536 # Gimbal manager supports to point to a local position.
uint32 GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072 # Gimbal manager supports to point to a global latitude, longitude, altitude position.

uint8 gimbal_device_id # Gimbal device ID that this gimbal manager is responsible for.
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



(provide :mavros_msgs/GimbalManagerInformation "afbf4fbefec70c70e001795d309516f7")


