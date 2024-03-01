;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::GimbalDeviceAttitudeStatus)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'GimbalDeviceAttitudeStatus (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS")
  (make-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))

(in-package "ROS")
;;//! \htmlinclude GimbalDeviceAttitudeStatus.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*GIMBAL_DEVICE_FLAGS_RETRACT*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_FLAGS_RETRACT* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_FLAGS_RETRACT* 1)
(intern "*GIMBAL_DEVICE_FLAGS_NEUTRAL*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_FLAGS_NEUTRAL* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_FLAGS_NEUTRAL* 2)
(intern "*GIMBAL_DEVICE_FLAGS_ROLL_LOCK*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_FLAGS_ROLL_LOCK* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_FLAGS_ROLL_LOCK* 4)
(intern "*GIMBAL_DEVICE_FLAGS_PITCH_LOCK*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_FLAGS_PITCH_LOCK* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_FLAGS_PITCH_LOCK* 8)
(intern "*GIMBAL_DEVICE_FLAGS_YAW_LOCK*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_FLAGS_YAW_LOCK* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_FLAGS_YAW_LOCK* 16)
(intern "*GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT* 1)
(intern "*GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT* 2)
(intern "*GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT* 4)
(intern "*GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR* 8)
(intern "*GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR* 16)
(intern "*GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR* 32)
(intern "*GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR* 64)
(intern "*GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR* 128)
(intern "*GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING*" (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(shadow '*GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING* (find-package "MAVROS_MSGS::GIMBALDEVICEATTITUDESTATUS"))
(defconstant mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING* 256)

(defun mavros_msgs::GimbalDeviceAttitudeStatus-to-symbol (const)
  (cond
        ((= const 1) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_FLAGS_RETRACT*)
        ((= const 2) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_FLAGS_NEUTRAL*)
        ((= const 4) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_FLAGS_ROLL_LOCK*)
        ((= const 8) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_FLAGS_PITCH_LOCK*)
        ((= const 16) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_FLAGS_YAW_LOCK*)
        ((= const 1) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT*)
        ((= const 2) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT*)
        ((= const 4) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT*)
        ((= const 8) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR*)
        ((= const 16) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR*)
        ((= const 32) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR*)
        ((= const 64) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR*)
        ((= const 128) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR*)
        ((= const 256) 'mavros_msgs::GimbalDeviceAttitudeStatus::*GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING*)
        (t nil)))

(defclass mavros_msgs::GimbalDeviceAttitudeStatus
  :super ros::object
  :slots (_header _target_system _target_component _flags _q _angular_velocity_x _angular_velocity_y _angular_velocity_z _failure_flags ))

(defmethod mavros_msgs::GimbalDeviceAttitudeStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:target_system __target_system) 0)
    ((:target_component __target_component) 0)
    ((:flags __flags) 0)
    ((:q __q) (instance geometry_msgs::Quaternion :init))
    ((:angular_velocity_x __angular_velocity_x) 0.0)
    ((:angular_velocity_y __angular_velocity_y) 0.0)
    ((:angular_velocity_z __angular_velocity_z) 0.0)
    ((:failure_flags __failure_flags) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _target_system (round __target_system))
   (setq _target_component (round __target_component))
   (setq _flags (round __flags))
   (setq _q __q)
   (setq _angular_velocity_x (float __angular_velocity_x))
   (setq _angular_velocity_y (float __angular_velocity_y))
   (setq _angular_velocity_z (float __angular_velocity_z))
   (setq _failure_flags (round __failure_flags))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:target_system
   (&optional __target_system)
   (if __target_system (setq _target_system __target_system)) _target_system)
  (:target_component
   (&optional __target_component)
   (if __target_component (setq _target_component __target_component)) _target_component)
  (:flags
   (&optional __flags)
   (if __flags (setq _flags __flags)) _flags)
  (:q
   (&rest __q)
   (if (keywordp (car __q))
       (send* _q __q)
     (progn
       (if __q (setq _q (car __q)))
       _q)))
  (:angular_velocity_x
   (&optional __angular_velocity_x)
   (if __angular_velocity_x (setq _angular_velocity_x __angular_velocity_x)) _angular_velocity_x)
  (:angular_velocity_y
   (&optional __angular_velocity_y)
   (if __angular_velocity_y (setq _angular_velocity_y __angular_velocity_y)) _angular_velocity_y)
  (:angular_velocity_z
   (&optional __angular_velocity_z)
   (if __angular_velocity_z (setq _angular_velocity_z __angular_velocity_z)) _angular_velocity_z)
  (:failure_flags
   (&optional __failure_flags)
   (if __failure_flags (setq _failure_flags __failure_flags)) _failure_flags)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _target_system
    1
    ;; uint8 _target_component
    1
    ;; uint16 _flags
    2
    ;; geometry_msgs/Quaternion _q
    (send _q :serialization-length)
    ;; float32 _angular_velocity_x
    4
    ;; float32 _angular_velocity_y
    4
    ;; float32 _angular_velocity_z
    4
    ;; uint32 _failure_flags
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _target_system
       (write-byte _target_system s)
     ;; uint8 _target_component
       (write-byte _target_component s)
     ;; uint16 _flags
       (write-word _flags s)
     ;; geometry_msgs/Quaternion _q
       (send _q :serialize s)
     ;; float32 _angular_velocity_x
       (sys::poke _angular_velocity_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular_velocity_y
       (sys::poke _angular_velocity_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angular_velocity_z
       (sys::poke _angular_velocity_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _failure_flags
       (write-long _failure_flags s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _target_system
     (setq _target_system (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _target_component
     (setq _target_component (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _flags
     (setq _flags (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; geometry_msgs/Quaternion _q
     (send _q :deserialize buf ptr-) (incf ptr- (send _q :serialization-length))
   ;; float32 _angular_velocity_x
     (setq _angular_velocity_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular_velocity_y
     (setq _angular_velocity_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angular_velocity_z
     (setq _angular_velocity_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _failure_flags
     (setq _failure_flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mavros_msgs::GimbalDeviceAttitudeStatus :md5sum-) "1fa324570393b1fbb776c5ff6e00764e")
(setf (get mavros_msgs::GimbalDeviceAttitudeStatus :datatype-) "mavros_msgs/GimbalDeviceAttitudeStatus")
(setf (get mavros_msgs::GimbalDeviceAttitudeStatus :definition-)
      "# MAVLink message: GIMBAL_DEVICE_ATTITUDE_STATUS
# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_ATTITUDE_STATUS

std_msgs/Header header

uint8 target_system             # System ID
uint8 target_component          # Component ID

uint16 flags                    # Current gimbal flags set (bitwise) - See GIMBAL_DEVICE_FLAGS
#GIMBAL_DEVICE_FLAGS
uint16 GIMBAL_DEVICE_FLAGS_RETRACT = 1      # Set to retracted safe position (no stabilization), takes presedence over all other flags.
uint16 GIMBAL_DEVICE_FLAGS_NEUTRAL = 2      # Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly forward-facing and horizontal (pitch=yaw=0) but may be any orientation.
uint16 GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4    # Lock roll angle to absolute angle relative to horizon (not relative to drone). This is generally the default with a stabilizing gimbal.
uint16 GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8   # Lock pitch angle to absolute angle relative to horizon (not relative to drone). This is generally the default.
uint16 GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16    # Lock yaw angle to absolute angle relative to North (not relative to drone). If this flag is set, the quaternion is in the Earth frame with the x-axis pointing North (yaw absolute). If this flag is not set, the quaternion frame is in the Earth frame rotated so that the x-axis is pointing forward (yaw relative to vehicle).

geometry_msgs/Quaternion q      # Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
float32 angular_velocity_x      # X component of angular velocity (NaN if unknown)
float32 angular_velocity_y      # Y component of angular velocity (NaN if unknown)
float32 angular_velocity_z      # Z component of angular velocity (NaN if unknown)

uint32 failure_flags            # Failure flags (0 for no failure) (bitwise) - See GIMBAL_DEVICE_ERROR_FLAGS
#GIMBAL_DEVICE_ERROR_FLAGS
uint32 GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT = 1          # Gimbal device is limited by hardware roll limit.
uint32 GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT = 2         # Gimbal device is limited by hardware pitch limit.
uint32 GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT = 4           # Gimbal device is limited by hardware yaw limit.
uint32 GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR = 8          # There is an error with the gimbal encoders.
uint32 GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR = 16           # There is an error with the gimbal power source.
uint32 GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR = 32           # There is an error with the gimbal motor's.
uint32 GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR = 64        # There is an error with the gimbal's software.
uint32 GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR = 128          # There is an error with the gimbal's communication.
uint32 GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING = 256  # Gimbal is currently calibrating.

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

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :mavros_msgs/GimbalDeviceAttitudeStatus "1fa324570393b1fbb776c5ff6e00764e")


