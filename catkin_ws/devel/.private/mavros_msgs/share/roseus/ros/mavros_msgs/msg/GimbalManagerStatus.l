;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::GimbalManagerStatus)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'GimbalManagerStatus (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERSTATUS")
  (make-package "MAVROS_MSGS::GIMBALMANAGERSTATUS"))

(in-package "ROS")
;;//! \htmlinclude GimbalManagerStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*GIMBAL_MANAGER_FLAGS_RETRACT*" (find-package "MAVROS_MSGS::GIMBALMANAGERSTATUS"))
(shadow '*GIMBAL_MANAGER_FLAGS_RETRACT* (find-package "MAVROS_MSGS::GIMBALMANAGERSTATUS"))
(defconstant mavros_msgs::GimbalManagerStatus::*GIMBAL_MANAGER_FLAGS_RETRACT* 1)
(intern "*GIMBAL_MANAGER_FLAGS_NEUTRAL*" (find-package "MAVROS_MSGS::GIMBALMANAGERSTATUS"))
(shadow '*GIMBAL_MANAGER_FLAGS_NEUTRAL* (find-package "MAVROS_MSGS::GIMBALMANAGERSTATUS"))
(defconstant mavros_msgs::GimbalManagerStatus::*GIMBAL_MANAGER_FLAGS_NEUTRAL* 2)
(intern "*GIMBAL_MANAGER_FLAGS_ROLL_LOCK*" (find-package "MAVROS_MSGS::GIMBALMANAGERSTATUS"))
(shadow '*GIMBAL_MANAGER_FLAGS_ROLL_LOCK* (find-package "MAVROS_MSGS::GIMBALMANAGERSTATUS"))
(defconstant mavros_msgs::GimbalManagerStatus::*GIMBAL_MANAGER_FLAGS_ROLL_LOCK* 4)
(intern "*GIMBAL_MANAGER_FLAGS_PITCH_LOCK*" (find-package "MAVROS_MSGS::GIMBALMANAGERSTATUS"))
(shadow '*GIMBAL_MANAGER_FLAGS_PITCH_LOCK* (find-package "MAVROS_MSGS::GIMBALMANAGERSTATUS"))
(defconstant mavros_msgs::GimbalManagerStatus::*GIMBAL_MANAGER_FLAGS_PITCH_LOCK* 8)
(intern "*GIMBAL_MANAGER_FLAGS_YAW_LOCK*" (find-package "MAVROS_MSGS::GIMBALMANAGERSTATUS"))
(shadow '*GIMBAL_MANAGER_FLAGS_YAW_LOCK* (find-package "MAVROS_MSGS::GIMBALMANAGERSTATUS"))
(defconstant mavros_msgs::GimbalManagerStatus::*GIMBAL_MANAGER_FLAGS_YAW_LOCK* 16)

(defun mavros_msgs::GimbalManagerStatus-to-symbol (const)
  (cond
        ((= const 1) 'mavros_msgs::GimbalManagerStatus::*GIMBAL_MANAGER_FLAGS_RETRACT*)
        ((= const 2) 'mavros_msgs::GimbalManagerStatus::*GIMBAL_MANAGER_FLAGS_NEUTRAL*)
        ((= const 4) 'mavros_msgs::GimbalManagerStatus::*GIMBAL_MANAGER_FLAGS_ROLL_LOCK*)
        ((= const 8) 'mavros_msgs::GimbalManagerStatus::*GIMBAL_MANAGER_FLAGS_PITCH_LOCK*)
        ((= const 16) 'mavros_msgs::GimbalManagerStatus::*GIMBAL_MANAGER_FLAGS_YAW_LOCK*)
        (t nil)))

(defclass mavros_msgs::GimbalManagerStatus
  :super ros::object
  :slots (_header _flags _gimbal_device_id _sysid_primary _compid_primary _sysid_secondary _compid_secondary ))

(defmethod mavros_msgs::GimbalManagerStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:flags __flags) 0)
    ((:gimbal_device_id __gimbal_device_id) 0)
    ((:sysid_primary __sysid_primary) 0)
    ((:compid_primary __compid_primary) 0)
    ((:sysid_secondary __sysid_secondary) 0)
    ((:compid_secondary __compid_secondary) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _flags (round __flags))
   (setq _gimbal_device_id (round __gimbal_device_id))
   (setq _sysid_primary (round __sysid_primary))
   (setq _compid_primary (round __compid_primary))
   (setq _sysid_secondary (round __sysid_secondary))
   (setq _compid_secondary (round __compid_secondary))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:flags
   (&optional __flags)
   (if __flags (setq _flags __flags)) _flags)
  (:gimbal_device_id
   (&optional __gimbal_device_id)
   (if __gimbal_device_id (setq _gimbal_device_id __gimbal_device_id)) _gimbal_device_id)
  (:sysid_primary
   (&optional __sysid_primary)
   (if __sysid_primary (setq _sysid_primary __sysid_primary)) _sysid_primary)
  (:compid_primary
   (&optional __compid_primary)
   (if __compid_primary (setq _compid_primary __compid_primary)) _compid_primary)
  (:sysid_secondary
   (&optional __sysid_secondary)
   (if __sysid_secondary (setq _sysid_secondary __sysid_secondary)) _sysid_secondary)
  (:compid_secondary
   (&optional __compid_secondary)
   (if __compid_secondary (setq _compid_secondary __compid_secondary)) _compid_secondary)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _flags
    4
    ;; uint8 _gimbal_device_id
    1
    ;; uint8 _sysid_primary
    1
    ;; uint8 _compid_primary
    1
    ;; uint8 _sysid_secondary
    1
    ;; uint8 _compid_secondary
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _flags
       (write-long _flags s)
     ;; uint8 _gimbal_device_id
       (write-byte _gimbal_device_id s)
     ;; uint8 _sysid_primary
       (write-byte _sysid_primary s)
     ;; uint8 _compid_primary
       (write-byte _compid_primary s)
     ;; uint8 _sysid_secondary
       (write-byte _sysid_secondary s)
     ;; uint8 _compid_secondary
       (write-byte _compid_secondary s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _flags
     (setq _flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _gimbal_device_id
     (setq _gimbal_device_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sysid_primary
     (setq _sysid_primary (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _compid_primary
     (setq _compid_primary (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sysid_secondary
     (setq _sysid_secondary (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _compid_secondary
     (setq _compid_secondary (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mavros_msgs::GimbalManagerStatus :md5sum-) "fc335d636c5417cf82ed6184846e75a1")
(setf (get mavros_msgs::GimbalManagerStatus :datatype-) "mavros_msgs/GimbalManagerStatus")
(setf (get mavros_msgs::GimbalManagerStatus :definition-)
      "# MAVLink message: GIMBAL_MANAGER_STATUS
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_STATUS

std_msgs/Header header

uint32 flags                # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS
#GIMBAL_MANAGER_FLAGS
uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT
uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK

uint8 gimbal_device_id # Gimbal device ID that this gimbal manager is responsible for.

uint8 sysid_primary     # System ID of MAVLink component with primary control, 0 for none.
uint8 compid_primary    # Component ID of MAVLink component with primary control, 0 for none.
uint8 sysid_secondary   # System ID of MAVLink component with secondary control, 0 for none.
uint8 compid_secondary  # Component ID of MAVLink component with secondary control, 0 for none.
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



(provide :mavros_msgs/GimbalManagerStatus "fc335d636c5417cf82ed6184846e75a1")


