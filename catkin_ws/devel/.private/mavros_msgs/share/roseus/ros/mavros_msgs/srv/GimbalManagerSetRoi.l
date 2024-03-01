;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::GimbalManagerSetRoi)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'GimbalManagerSetRoi (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERSETROI")
  (make-package "MAVROS_MSGS::GIMBALMANAGERSETROI"))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERSETROIREQUEST")
  (make-package "MAVROS_MSGS::GIMBALMANAGERSETROIREQUEST"))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERSETROIRESPONSE")
  (make-package "MAVROS_MSGS::GIMBALMANAGERSETROIRESPONSE"))

(in-package "ROS")





(intern "*ROI_MODE_LOCATION*" (find-package "MAVROS_MSGS::GIMBALMANAGERSETROIREQUEST"))
(shadow '*ROI_MODE_LOCATION* (find-package "MAVROS_MSGS::GIMBALMANAGERSETROIREQUEST"))
(defconstant mavros_msgs::GimbalManagerSetRoiRequest::*ROI_MODE_LOCATION* 0)
(intern "*ROI_MODE_WP_NEXT_OFFSET*" (find-package "MAVROS_MSGS::GIMBALMANAGERSETROIREQUEST"))
(shadow '*ROI_MODE_WP_NEXT_OFFSET* (find-package "MAVROS_MSGS::GIMBALMANAGERSETROIREQUEST"))
(defconstant mavros_msgs::GimbalManagerSetRoiRequest::*ROI_MODE_WP_NEXT_OFFSET* 1)
(intern "*ROI_MODE_SYSID*" (find-package "MAVROS_MSGS::GIMBALMANAGERSETROIREQUEST"))
(shadow '*ROI_MODE_SYSID* (find-package "MAVROS_MSGS::GIMBALMANAGERSETROIREQUEST"))
(defconstant mavros_msgs::GimbalManagerSetRoiRequest::*ROI_MODE_SYSID* 2)
(intern "*ROI_MODE_NONE*" (find-package "MAVROS_MSGS::GIMBALMANAGERSETROIREQUEST"))
(shadow '*ROI_MODE_NONE* (find-package "MAVROS_MSGS::GIMBALMANAGERSETROIREQUEST"))
(defconstant mavros_msgs::GimbalManagerSetRoiRequest::*ROI_MODE_NONE* 3)

(defun mavros_msgs::GimbalManagerSetRoiRequest-to-symbol (const)
  (cond
        ((= const 0) 'mavros_msgs::GimbalManagerSetRoiRequest::*ROI_MODE_LOCATION*)
        ((= const 1) 'mavros_msgs::GimbalManagerSetRoiRequest::*ROI_MODE_WP_NEXT_OFFSET*)
        ((= const 2) 'mavros_msgs::GimbalManagerSetRoiRequest::*ROI_MODE_SYSID*)
        ((= const 3) 'mavros_msgs::GimbalManagerSetRoiRequest::*ROI_MODE_NONE*)
        (t nil)))

(defclass mavros_msgs::GimbalManagerSetRoiRequest
  :super ros::object
  :slots (_mode _gimbal_device_id _latitude _longitude _altitude _pitch_offset _roll_offset _yaw_offset _sysid ))

(defmethod mavros_msgs::GimbalManagerSetRoiRequest
  (:init
   (&key
    ((:mode __mode) 0)
    ((:gimbal_device_id __gimbal_device_id) 0)
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:altitude __altitude) 0.0)
    ((:pitch_offset __pitch_offset) 0.0)
    ((:roll_offset __roll_offset) 0.0)
    ((:yaw_offset __yaw_offset) 0.0)
    ((:sysid __sysid) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _gimbal_device_id (round __gimbal_device_id))
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _altitude (float __altitude))
   (setq _pitch_offset (float __pitch_offset))
   (setq _roll_offset (float __roll_offset))
   (setq _yaw_offset (float __yaw_offset))
   (setq _sysid (round __sysid))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:gimbal_device_id
   (&optional __gimbal_device_id)
   (if __gimbal_device_id (setq _gimbal_device_id __gimbal_device_id)) _gimbal_device_id)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:pitch_offset
   (&optional __pitch_offset)
   (if __pitch_offset (setq _pitch_offset __pitch_offset)) _pitch_offset)
  (:roll_offset
   (&optional __roll_offset)
   (if __roll_offset (setq _roll_offset __roll_offset)) _roll_offset)
  (:yaw_offset
   (&optional __yaw_offset)
   (if __yaw_offset (setq _yaw_offset __yaw_offset)) _yaw_offset)
  (:sysid
   (&optional __sysid)
   (if __sysid (setq _sysid __sysid)) _sysid)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ;; uint8 _gimbal_device_id
    1
    ;; float32 _latitude
    4
    ;; float32 _longitude
    4
    ;; float32 _altitude
    4
    ;; float32 _pitch_offset
    4
    ;; float32 _roll_offset
    4
    ;; float32 _yaw_offset
    4
    ;; uint8 _sysid
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;; uint8 _gimbal_device_id
       (write-byte _gimbal_device_id s)
     ;; float32 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch_offset
       (sys::poke _pitch_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _roll_offset
       (sys::poke _roll_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw_offset
       (sys::poke _yaw_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _sysid
       (write-byte _sysid s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gimbal_device_id
     (setq _gimbal_device_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _latitude
     (setq _latitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _longitude
     (setq _longitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _altitude
     (setq _altitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch_offset
     (setq _pitch_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _roll_offset
     (setq _roll_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw_offset
     (setq _yaw_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _sysid
     (setq _sysid (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mavros_msgs::GimbalManagerSetRoiResponse
  :super ros::object
  :slots (_success _result ))

(defmethod mavros_msgs::GimbalManagerSetRoiResponse
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

(defclass mavros_msgs::GimbalManagerSetRoi
  :super ros::object
  :slots ())

(setf (get mavros_msgs::GimbalManagerSetRoi :md5sum-) "0796c6a6cf0e18b0d829681b29f0828a")
(setf (get mavros_msgs::GimbalManagerSetRoi :datatype-) "mavros_msgs/GimbalManagerSetRoi")
(setf (get mavros_msgs::GimbalManagerSetRoi :request) mavros_msgs::GimbalManagerSetRoiRequest)
(setf (get mavros_msgs::GimbalManagerSetRoi :response) mavros_msgs::GimbalManagerSetRoiResponse)

(defmethod mavros_msgs::GimbalManagerSetRoiRequest
  (:response () (instance mavros_msgs::GimbalManagerSetRoiResponse :init)))

(setf (get mavros_msgs::GimbalManagerSetRoiRequest :md5sum-) "0796c6a6cf0e18b0d829681b29f0828a")
(setf (get mavros_msgs::GimbalManagerSetRoiRequest :datatype-) "mavros_msgs/GimbalManagerSetRoiRequest")
(setf (get mavros_msgs::GimbalManagerSetRoiRequest :definition-)
      "# MAVLink commands: DO_SET_ROI_LOCATION, DO_SET_ROI_WPNEXT_OFFSET, DO_SET_ROI_SYSID, DO_SET_ROI_NONE
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_LOCATION
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_SYSID
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_NONE

uint8 mode              # enumerator to indicate ROI mode setting - see ROI_MODE
#ROI_MODE
uint8 ROI_MODE_LOCATION = 0         # Sets the region of interest (ROI) to a location. [DO_SET_ROI_LOCATION]
uint8 ROI_MODE_WP_NEXT_OFFSET = 1   # Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. [DO_SET_ROI_WPNEXT_OFFSET]
uint8 ROI_MODE_SYSID = 2            # Mount tracks system with specified system ID [DO_SET_ROI_SYSID]
uint8 ROI_MODE_NONE = 3             # Cancels any previous ROI setting and returns vehicle to defaults [DO_SET_ROI_NONE]

uint8 gimbal_device_id  # Component ID of gimbal device to address 
                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
                        # components. Send command multiple times for more than
                        # one gimbal (but not all gimbals).  Default Mavlink gimbal 
                        # device ids: 154, 171-175

#For ROI_MODE_LOCATION
float32 latitude
float32 longitude
float32 altitude        # Meters

#For ROI_MODE_WP_NEXT_OFFSET
float32 pitch_offset    # Pitch offset from next waypoint, positive pitching up
float32 roll_offset     # Roll offset from next waypoint, positive rolling to the right
float32 yaw_offset      # Yaw offset from next waypoint, positive yawing to the right

#For ROI_MODE_SYSID
uint8 sysid             # System ID to track (min: 1, max: 255)

#ROI_MODE_NONE doesn't take extra parameters

---
bool success
# raw result returned by COMMAND_ACK
uint8 result
")

(setf (get mavros_msgs::GimbalManagerSetRoiResponse :md5sum-) "0796c6a6cf0e18b0d829681b29f0828a")
(setf (get mavros_msgs::GimbalManagerSetRoiResponse :datatype-) "mavros_msgs/GimbalManagerSetRoiResponse")
(setf (get mavros_msgs::GimbalManagerSetRoiResponse :definition-)
      "# MAVLink commands: DO_SET_ROI_LOCATION, DO_SET_ROI_WPNEXT_OFFSET, DO_SET_ROI_SYSID, DO_SET_ROI_NONE
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_LOCATION
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_SYSID
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_ROI_NONE

uint8 mode              # enumerator to indicate ROI mode setting - see ROI_MODE
#ROI_MODE
uint8 ROI_MODE_LOCATION = 0         # Sets the region of interest (ROI) to a location. [DO_SET_ROI_LOCATION]
uint8 ROI_MODE_WP_NEXT_OFFSET = 1   # Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. [DO_SET_ROI_WPNEXT_OFFSET]
uint8 ROI_MODE_SYSID = 2            # Mount tracks system with specified system ID [DO_SET_ROI_SYSID]
uint8 ROI_MODE_NONE = 3             # Cancels any previous ROI setting and returns vehicle to defaults [DO_SET_ROI_NONE]

uint8 gimbal_device_id  # Component ID of gimbal device to address 
                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
                        # components. Send command multiple times for more than
                        # one gimbal (but not all gimbals).  Default Mavlink gimbal 
                        # device ids: 154, 171-175

#For ROI_MODE_LOCATION
float32 latitude
float32 longitude
float32 altitude        # Meters

#For ROI_MODE_WP_NEXT_OFFSET
float32 pitch_offset    # Pitch offset from next waypoint, positive pitching up
float32 roll_offset     # Roll offset from next waypoint, positive rolling to the right
float32 yaw_offset      # Yaw offset from next waypoint, positive yawing to the right

#For ROI_MODE_SYSID
uint8 sysid             # System ID to track (min: 1, max: 255)

#ROI_MODE_NONE doesn't take extra parameters

---
bool success
# raw result returned by COMMAND_ACK
uint8 result
")



(provide :mavros_msgs/GimbalManagerSetRoi "0796c6a6cf0e18b0d829681b29f0828a")


