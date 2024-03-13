;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::GimbalManagerPitchyaw)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'GimbalManagerPitchyaw (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAW")
  (make-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAW"))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWREQUEST")
  (make-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWREQUEST"))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWRESPONSE")
  (make-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWRESPONSE"))

(in-package "ROS")





(intern "*GIMBAL_MANAGER_FLAGS_RETRACT*" (find-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWREQUEST"))
(shadow '*GIMBAL_MANAGER_FLAGS_RETRACT* (find-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWREQUEST"))
(defconstant mavros_msgs::GimbalManagerPitchyawRequest::*GIMBAL_MANAGER_FLAGS_RETRACT* 1)
(intern "*GIMBAL_MANAGER_FLAGS_NEUTRAL*" (find-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWREQUEST"))
(shadow '*GIMBAL_MANAGER_FLAGS_NEUTRAL* (find-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWREQUEST"))
(defconstant mavros_msgs::GimbalManagerPitchyawRequest::*GIMBAL_MANAGER_FLAGS_NEUTRAL* 2)
(intern "*GIMBAL_MANAGER_FLAGS_ROLL_LOCK*" (find-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWREQUEST"))
(shadow '*GIMBAL_MANAGER_FLAGS_ROLL_LOCK* (find-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWREQUEST"))
(defconstant mavros_msgs::GimbalManagerPitchyawRequest::*GIMBAL_MANAGER_FLAGS_ROLL_LOCK* 4)
(intern "*GIMBAL_MANAGER_FLAGS_PITCH_LOCK*" (find-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWREQUEST"))
(shadow '*GIMBAL_MANAGER_FLAGS_PITCH_LOCK* (find-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWREQUEST"))
(defconstant mavros_msgs::GimbalManagerPitchyawRequest::*GIMBAL_MANAGER_FLAGS_PITCH_LOCK* 8)
(intern "*GIMBAL_MANAGER_FLAGS_YAW_LOCK*" (find-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWREQUEST"))
(shadow '*GIMBAL_MANAGER_FLAGS_YAW_LOCK* (find-package "MAVROS_MSGS::GIMBALMANAGERPITCHYAWREQUEST"))
(defconstant mavros_msgs::GimbalManagerPitchyawRequest::*GIMBAL_MANAGER_FLAGS_YAW_LOCK* 16)

(defun mavros_msgs::GimbalManagerPitchyawRequest-to-symbol (const)
  (cond
        ((= const 1) 'mavros_msgs::GimbalManagerPitchyawRequest::*GIMBAL_MANAGER_FLAGS_RETRACT*)
        ((= const 2) 'mavros_msgs::GimbalManagerPitchyawRequest::*GIMBAL_MANAGER_FLAGS_NEUTRAL*)
        ((= const 4) 'mavros_msgs::GimbalManagerPitchyawRequest::*GIMBAL_MANAGER_FLAGS_ROLL_LOCK*)
        ((= const 8) 'mavros_msgs::GimbalManagerPitchyawRequest::*GIMBAL_MANAGER_FLAGS_PITCH_LOCK*)
        ((= const 16) 'mavros_msgs::GimbalManagerPitchyawRequest::*GIMBAL_MANAGER_FLAGS_YAW_LOCK*)
        (t nil)))

(defclass mavros_msgs::GimbalManagerPitchyawRequest
  :super ros::object
  :slots (_pitch _yaw _pitch_rate _yaw_rate _flags _gimbal_device_id ))

(defmethod mavros_msgs::GimbalManagerPitchyawRequest
  (:init
   (&key
    ((:pitch __pitch) 0.0)
    ((:yaw __yaw) 0.0)
    ((:pitch_rate __pitch_rate) 0.0)
    ((:yaw_rate __yaw_rate) 0.0)
    ((:flags __flags) 0)
    ((:gimbal_device_id __gimbal_device_id) 0)
    )
   (send-super :init)
   (setq _pitch (float __pitch))
   (setq _yaw (float __yaw))
   (setq _pitch_rate (float __pitch_rate))
   (setq _yaw_rate (float __yaw_rate))
   (setq _flags (round __flags))
   (setq _gimbal_device_id (round __gimbal_device_id))
   self)
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
  (:flags
   (&optional __flags)
   (if __flags (setq _flags __flags)) _flags)
  (:gimbal_device_id
   (&optional __gimbal_device_id)
   (if __gimbal_device_id (setq _gimbal_device_id __gimbal_device_id)) _gimbal_device_id)
  (:serialization-length
   ()
   (+
    ;; float32 _pitch
    4
    ;; float32 _yaw
    4
    ;; float32 _pitch_rate
    4
    ;; float32 _yaw_rate
    4
    ;; uint32 _flags
    4
    ;; uint8 _gimbal_device_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_rate
       (sys::poke _pitch_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_rate
       (sys::poke _yaw_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _flags
       (write-long _flags s)
     ;; uint8 _gimbal_device_id
       (write-byte _gimbal_device_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_rate
     (setq _pitch_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_rate
     (setq _yaw_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _flags
     (setq _flags (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _gimbal_device_id
     (setq _gimbal_device_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mavros_msgs::GimbalManagerPitchyawResponse
  :super ros::object
  :slots (_success _result ))

(defmethod mavros_msgs::GimbalManagerPitchyawResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _success __success)
   (setq _result (round __result))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; uint8 _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _result
       (write-byte _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _result
     (setq _result (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mavros_msgs::GimbalManagerPitchyaw
  :super ros::object
  :slots ())

(setf (get mavros_msgs::GimbalManagerPitchyaw :md5sum-) "64c6b1297e0d0b5bed3f6fb79c8c51aa")
(setf (get mavros_msgs::GimbalManagerPitchyaw :datatype-) "mavros_msgs/GimbalManagerPitchyaw")
(setf (get mavros_msgs::GimbalManagerPitchyaw :request) mavros_msgs::GimbalManagerPitchyawRequest)
(setf (get mavros_msgs::GimbalManagerPitchyaw :response) mavros_msgs::GimbalManagerPitchyawResponse)

(defmethod mavros_msgs::GimbalManagerPitchyawRequest
  (:response () (instance mavros_msgs::GimbalManagerPitchyawResponse :init)))

(setf (get mavros_msgs::GimbalManagerPitchyawRequest :md5sum-) "64c6b1297e0d0b5bed3f6fb79c8c51aa")
(setf (get mavros_msgs::GimbalManagerPitchyawRequest :datatype-) "mavros_msgs/GimbalManagerPitchyawRequest")
(setf (get mavros_msgs::GimbalManagerPitchyawRequest :definition-)
      "# MAVLink commands: DO_GIMBAL_MANAGER_PITCHYAW
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW


float32 pitch # Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode). (-180 to 180 deg)
float32 yaw # Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode). (-180 to 180 deg)
float32 pitch_rate # Pitch rate (positive to pitch up). (deg/s)
float32 yaw_rate # Yaw rate (positive to yaw to the right). (deg/s)

uint32 flags # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS
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

---
bool success
# raw result returned by COMMAND_ACK
uint8 result
")

(setf (get mavros_msgs::GimbalManagerPitchyawResponse :md5sum-) "64c6b1297e0d0b5bed3f6fb79c8c51aa")
(setf (get mavros_msgs::GimbalManagerPitchyawResponse :datatype-) "mavros_msgs/GimbalManagerPitchyawResponse")
(setf (get mavros_msgs::GimbalManagerPitchyawResponse :definition-)
      "# MAVLink commands: DO_GIMBAL_MANAGER_PITCHYAW
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW


float32 pitch # Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode). (-180 to 180 deg)
float32 yaw # Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode). (-180 to 180 deg)
float32 pitch_rate # Pitch rate (positive to pitch up). (deg/s)
float32 yaw_rate # Yaw rate (positive to yaw to the right). (deg/s)

uint32 flags # High level gimbal manager flags to use - See GIMBAL_MANAGER_FLAGS
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

---
bool success
# raw result returned by COMMAND_ACK
uint8 result
")



(provide :mavros_msgs/GimbalManagerPitchyaw "64c6b1297e0d0b5bed3f6fb79c8c51aa")


