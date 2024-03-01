;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::GimbalManagerSetPitchyaw)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'GimbalManagerSetPitchyaw (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERSETPITCHYAW")
  (make-package "MAVROS_MSGS::GIMBALMANAGERSETPITCHYAW"))

(in-package "ROS")
;;//! \htmlinclude GimbalManagerSetPitchyaw.msg.html


(intern "*GIMBAL_MANAGER_FLAGS_RETRACT*" (find-package "MAVROS_MSGS::GIMBALMANAGERSETPITCHYAW"))
(shadow '*GIMBAL_MANAGER_FLAGS_RETRACT* (find-package "MAVROS_MSGS::GIMBALMANAGERSETPITCHYAW"))
(defconstant mavros_msgs::GimbalManagerSetPitchyaw::*GIMBAL_MANAGER_FLAGS_RETRACT* 1)
(intern "*GIMBAL_MANAGER_FLAGS_NEUTRAL*" (find-package "MAVROS_MSGS::GIMBALMANAGERSETPITCHYAW"))
(shadow '*GIMBAL_MANAGER_FLAGS_NEUTRAL* (find-package "MAVROS_MSGS::GIMBALMANAGERSETPITCHYAW"))
(defconstant mavros_msgs::GimbalManagerSetPitchyaw::*GIMBAL_MANAGER_FLAGS_NEUTRAL* 2)
(intern "*GIMBAL_MANAGER_FLAGS_ROLL_LOCK*" (find-package "MAVROS_MSGS::GIMBALMANAGERSETPITCHYAW"))
(shadow '*GIMBAL_MANAGER_FLAGS_ROLL_LOCK* (find-package "MAVROS_MSGS::GIMBALMANAGERSETPITCHYAW"))
(defconstant mavros_msgs::GimbalManagerSetPitchyaw::*GIMBAL_MANAGER_FLAGS_ROLL_LOCK* 4)
(intern "*GIMBAL_MANAGER_FLAGS_PITCH_LOCK*" (find-package "MAVROS_MSGS::GIMBALMANAGERSETPITCHYAW"))
(shadow '*GIMBAL_MANAGER_FLAGS_PITCH_LOCK* (find-package "MAVROS_MSGS::GIMBALMANAGERSETPITCHYAW"))
(defconstant mavros_msgs::GimbalManagerSetPitchyaw::*GIMBAL_MANAGER_FLAGS_PITCH_LOCK* 8)
(intern "*GIMBAL_MANAGER_FLAGS_YAW_LOCK*" (find-package "MAVROS_MSGS::GIMBALMANAGERSETPITCHYAW"))
(shadow '*GIMBAL_MANAGER_FLAGS_YAW_LOCK* (find-package "MAVROS_MSGS::GIMBALMANAGERSETPITCHYAW"))
(defconstant mavros_msgs::GimbalManagerSetPitchyaw::*GIMBAL_MANAGER_FLAGS_YAW_LOCK* 16)

(defun mavros_msgs::GimbalManagerSetPitchyaw-to-symbol (const)
  (cond
        ((= const 1) 'mavros_msgs::GimbalManagerSetPitchyaw::*GIMBAL_MANAGER_FLAGS_RETRACT*)
        ((= const 2) 'mavros_msgs::GimbalManagerSetPitchyaw::*GIMBAL_MANAGER_FLAGS_NEUTRAL*)
        ((= const 4) 'mavros_msgs::GimbalManagerSetPitchyaw::*GIMBAL_MANAGER_FLAGS_ROLL_LOCK*)
        ((= const 8) 'mavros_msgs::GimbalManagerSetPitchyaw::*GIMBAL_MANAGER_FLAGS_PITCH_LOCK*)
        ((= const 16) 'mavros_msgs::GimbalManagerSetPitchyaw::*GIMBAL_MANAGER_FLAGS_YAW_LOCK*)
        (t nil)))

(defclass mavros_msgs::GimbalManagerSetPitchyaw
  :super ros::object
  :slots (_target_system _target_component _flags _gimbal_device_id _pitch _yaw _pitch_rate _yaw_rate ))

(defmethod mavros_msgs::GimbalManagerSetPitchyaw
  (:init
   (&key
    ((:target_system __target_system) 0)
    ((:target_component __target_component) 0)
    ((:flags __flags) 0)
    ((:gimbal_device_id __gimbal_device_id) 0)
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:pitch_rate __pitch_rate) 0.0)
    ((:yaw_rate __yaw_rate) 0.0)
    )
   (send-super :init)
   (setq _target_system (round __target_system))
   (setq _target_component (round __target_component))
   (setq _flags (round __flags))
   (setq _gimbal_device_id (round __gimbal_device_id))
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _pitch_rate (float __pitch_rate))
   (setq _yaw_rate (float __yaw_rate))
   self)
  (:target_system
   (&optional __target_system)
   (if __target_system (setq _target_system __target_system)) _target_system)
  (:target_component
   (&optional __target_component)
   (if __target_component (setq _target_component __target_component)) _target_component)
  (:flags
   (&optional __flags)
   (if __flags (setq _flags __flags)) _flags)
  (:gimbal_device_id
   (&optional __gimbal_device_id)
   (if __gimbal_device_id (setq _gimbal_device_id __gimbal_device_id)) _gimbal_device_id)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:pitch_rate
   (&optional __pitch_rate)
   (if __pitch_rate (setq _pitch_rate __pitch_rate)) _pitch_rate)
  (:yaw_rate
   (&optional __yaw_rate)
   (if __yaw_rate (setq _yaw_rate __yaw_rate)) _yaw_rate)
  (:serialization-length
   ()
   (+
    ;; uint8 _target_system
    1
    ;; uint8 _target_component
    1
    ;; uint32 _flags
    4
    ;; uint8 _gimbal_device_id
    1
    ;; float32 _pitch
    4
    ;; float32 _yaw
    4
    ;; float32 _pitch_rate
    4
    ;; float32 _yaw_rate
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _target_system
       (write-byte _target_system s)
     ;; uint8 _target_component
       (write-byte _target_component s)
     ;; uint32 _flags
       (write-long _flags s)
     ;; uint8 _gimbal_device_id
       (write-byte _gimbal_device_id s)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_rate
       (sys::poke _pitch_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _target_system
     (setq _target_system (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _target_component
     (setq _target_component (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _flags
     (setq _flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _gimbal_device_id
     (setq _gimbal_device_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_rate
     (setq _pitch_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mavros_msgs::GimbalManagerSetPitchyaw :md5sum-) "1167a6666ca24e39fb8a2101fd672d21")
(setf (get mavros_msgs::GimbalManagerSetPitchyaw :datatype-) "mavros_msgs/GimbalManagerSetPitchyaw")
(setf (get mavros_msgs::GimbalManagerSetPitchyaw :definition-)
      "# MAVLink message: GIMBAL_MANAGER_SET_PITCHYAW
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_PITCHYAW
# Note that this message structure is identical also to GIMBAL_MANAGER_SET_MANUAL_CONTROL and is 
# reused as such by the plugin
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_SET_MANUAL_CONTROL

uint8 target_system         # System ID
uint8 target_component      # Component ID

uint32 flags                # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS
#GIMBAL_MANAGER_FLAGS
uint32 GIMBAL_MANAGER_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT
uint32 GIMBAL_MANAGER_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
uint32 GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
uint32 GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
uint32 GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK

uint8 gimbal_device_id  # Component ID of gimbal device to address 
                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
                        # components. Send command multiple times for more than
                        # one gimbal (but not all gimbals).  Default Mavlink gimbal 
                        # device ids: 154, 171-175

float32 pitch # Pitch angle (positive: up, negative: down, NaN to be ignored).
float32 yaw # Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).
float32 pitch_rate # Pitch angular rate (positive: up, negative: down, NaN to be ignored).
float32 yaw_rate # Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).
")



(provide :mavros_msgs/GimbalManagerSetPitchyaw "1167a6666ca24e39fb8a2101fd672d21")


