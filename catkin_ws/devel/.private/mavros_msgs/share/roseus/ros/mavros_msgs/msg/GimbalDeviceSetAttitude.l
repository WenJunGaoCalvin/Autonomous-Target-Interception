;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::GimbalDeviceSetAttitude)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'GimbalDeviceSetAttitude (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::GIMBALDEVICESETATTITUDE")
  (make-package "MAVROS_MSGS::GIMBALDEVICESETATTITUDE"))

(in-package "ROS")
;;//! \htmlinclude GimbalDeviceSetAttitude.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*GIMBAL_DEVICE_FLAGS_RETRACT*" (find-package "MAVROS_MSGS::GIMBALDEVICESETATTITUDE"))
(shadow '*GIMBAL_DEVICE_FLAGS_RETRACT* (find-package "MAVROS_MSGS::GIMBALDEVICESETATTITUDE"))
(defconstant mavros_msgs::GimbalDeviceSetAttitude::*GIMBAL_DEVICE_FLAGS_RETRACT* 1)
(intern "*GIMBAL_DEVICE_FLAGS_NEUTRAL*" (find-package "MAVROS_MSGS::GIMBALDEVICESETATTITUDE"))
(shadow '*GIMBAL_DEVICE_FLAGS_NEUTRAL* (find-package "MAVROS_MSGS::GIMBALDEVICESETATTITUDE"))
(defconstant mavros_msgs::GimbalDeviceSetAttitude::*GIMBAL_DEVICE_FLAGS_NEUTRAL* 2)
(intern "*GIMBAL_DEVICE_FLAGS_ROLL_LOCK*" (find-package "MAVROS_MSGS::GIMBALDEVICESETATTITUDE"))
(shadow '*GIMBAL_DEVICE_FLAGS_ROLL_LOCK* (find-package "MAVROS_MSGS::GIMBALDEVICESETATTITUDE"))
(defconstant mavros_msgs::GimbalDeviceSetAttitude::*GIMBAL_DEVICE_FLAGS_ROLL_LOCK* 4)
(intern "*GIMBAL_DEVICE_FLAGS_PITCH_LOCK*" (find-package "MAVROS_MSGS::GIMBALDEVICESETATTITUDE"))
(shadow '*GIMBAL_DEVICE_FLAGS_PITCH_LOCK* (find-package "MAVROS_MSGS::GIMBALDEVICESETATTITUDE"))
(defconstant mavros_msgs::GimbalDeviceSetAttitude::*GIMBAL_DEVICE_FLAGS_PITCH_LOCK* 8)
(intern "*GIMBAL_DEVICE_FLAGS_YAW_LOCK*" (find-package "MAVROS_MSGS::GIMBALDEVICESETATTITUDE"))
(shadow '*GIMBAL_DEVICE_FLAGS_YAW_LOCK* (find-package "MAVROS_MSGS::GIMBALDEVICESETATTITUDE"))
(defconstant mavros_msgs::GimbalDeviceSetAttitude::*GIMBAL_DEVICE_FLAGS_YAW_LOCK* 16)

(defun mavros_msgs::GimbalDeviceSetAttitude-to-symbol (const)
  (cond
        ((= const 1) 'mavros_msgs::GimbalDeviceSetAttitude::*GIMBAL_DEVICE_FLAGS_RETRACT*)
        ((= const 2) 'mavros_msgs::GimbalDeviceSetAttitude::*GIMBAL_DEVICE_FLAGS_NEUTRAL*)
        ((= const 4) 'mavros_msgs::GimbalDeviceSetAttitude::*GIMBAL_DEVICE_FLAGS_ROLL_LOCK*)
        ((= const 8) 'mavros_msgs::GimbalDeviceSetAttitude::*GIMBAL_DEVICE_FLAGS_PITCH_LOCK*)
        ((= const 16) 'mavros_msgs::GimbalDeviceSetAttitude::*GIMBAL_DEVICE_FLAGS_YAW_LOCK*)
        (t nil)))

(defclass mavros_msgs::GimbalDeviceSetAttitude
  :super ros::object
  :slots (_target_system _target_component _flags _q _angular_velocity_x _angular_velocity_y _angular_velocity_z ))

(defmethod mavros_msgs::GimbalDeviceSetAttitude
  (:init
   (&key
    ((:target_system __target_system) 0)
    ((:target_component __target_component) 0)
    ((:flags __flags) 0)
    ((:q __q) (instance geometry_msgs::Quaternion :init))
    ((:angular_velocity_x __angular_velocity_x) 0.0)
    ((:angular_velocity_y __angular_velocity_y) 0.0)
    ((:angular_velocity_z __angular_velocity_z) 0.0)
    )
   (send-super :init)
   (setq _target_system (round __target_system))
   (setq _target_component (round __target_component))
   (setq _flags (round __flags))
   (setq _q __q)
   (setq _angular_velocity_x (float __angular_velocity_x))
   (setq _angular_velocity_y (float __angular_velocity_y))
   (setq _angular_velocity_z (float __angular_velocity_z))
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
  (:serialization-length
   ()
   (+
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
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
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
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
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
   ;;
   self)
  )

(setf (get mavros_msgs::GimbalDeviceSetAttitude :md5sum-) "87a55ddad0e67129d73063fdc10f2ffb")
(setf (get mavros_msgs::GimbalDeviceSetAttitude :datatype-) "mavros_msgs/GimbalDeviceSetAttitude")
(setf (get mavros_msgs::GimbalDeviceSetAttitude :definition-)
      "# MAVLink message: GIMBAL_DEVICE_SET_ATTITUDE
# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_SET_ATTITUDE

uint8 target_system         # System ID
uint8 target_component      # Component ID

uint16 flags                # Low level gimbal flags (bitwise) - See GIMBAL_DEVICE_FLAGS
#GIMBAL_DEVICE_FLAGS
uint16 GIMBAL_DEVICE_FLAGS_RETRACT = 1     # Based on GIMBAL_DEVICE_FLAGS_RETRACT
uint16 GIMBAL_DEVICE_FLAGS_NEUTRAL = 2     # Based on GIMBAL_DEVICE_FLAGS_NEUTRAL
uint16 GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4   # Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK
uint16 GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8  # Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK
uint16 GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16   # Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK

geometry_msgs/Quaternion q # Quaternion, x, y, z, w (0 0 0 1 is the null-rotation, the frame is depends on whether the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set)
float32 angular_velocity_x # X component of angular velocity, positive is rolling to the right, NaN to be ignored.
float32 angular_velocity_y # Y component of angular velocity, positive is pitching up, NaN to be ignored.
float32 angular_velocity_z # Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :mavros_msgs/GimbalDeviceSetAttitude "87a55ddad0e67129d73063fdc10f2ffb")


