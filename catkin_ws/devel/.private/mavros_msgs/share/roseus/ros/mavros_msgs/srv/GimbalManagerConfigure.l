;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::GimbalManagerConfigure)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'GimbalManagerConfigure (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERCONFIGURE")
  (make-package "MAVROS_MSGS::GIMBALMANAGERCONFIGURE"))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERCONFIGUREREQUEST")
  (make-package "MAVROS_MSGS::GIMBALMANAGERCONFIGUREREQUEST"))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERCONFIGURERESPONSE")
  (make-package "MAVROS_MSGS::GIMBALMANAGERCONFIGURERESPONSE"))

(in-package "ROS")





(defclass mavros_msgs::GimbalManagerConfigureRequest
  :super ros::object
  :slots (_sysid_primary _compid_primary _sysid_secondary _compid_secondary _gimbal_device_id ))

(defmethod mavros_msgs::GimbalManagerConfigureRequest
  (:init
   (&key
    ((:sysid_primary __sysid_primary) 0)
    ((:compid_primary __compid_primary) 0)
    ((:sysid_secondary __sysid_secondary) 0)
    ((:compid_secondary __compid_secondary) 0)
    ((:gimbal_device_id __gimbal_device_id) 0)
    )
   (send-super :init)
   (setq _sysid_primary (round __sysid_primary))
   (setq _compid_primary (round __compid_primary))
   (setq _sysid_secondary (round __sysid_secondary))
   (setq _compid_secondary (round __compid_secondary))
   (setq _gimbal_device_id (round __gimbal_device_id))
   self)
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
  (:gimbal_device_id
   (&optional __gimbal_device_id)
   (if __gimbal_device_id (setq _gimbal_device_id __gimbal_device_id)) _gimbal_device_id)
  (:serialization-length
   ()
   (+
    ;; int16 _sysid_primary
    2
    ;; int16 _compid_primary
    2
    ;; int16 _sysid_secondary
    2
    ;; int16 _compid_secondary
    2
    ;; uint8 _gimbal_device_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _sysid_primary
       (write-word _sysid_primary s)
     ;; int16 _compid_primary
       (write-word _compid_primary s)
     ;; int16 _sysid_secondary
       (write-word _sysid_secondary s)
     ;; int16 _compid_secondary
       (write-word _compid_secondary s)
     ;; uint8 _gimbal_device_id
       (write-byte _gimbal_device_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _sysid_primary
     (setq _sysid_primary (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _compid_primary
     (setq _compid_primary (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _sysid_secondary
     (setq _sysid_secondary (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _compid_secondary
     (setq _compid_secondary (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _gimbal_device_id
     (setq _gimbal_device_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass mavros_msgs::GimbalManagerConfigureResponse
  :super ros::object
  :slots (_success _result ))

(defmethod mavros_msgs::GimbalManagerConfigureResponse
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

(defclass mavros_msgs::GimbalManagerConfigure
  :super ros::object
  :slots ())

(setf (get mavros_msgs::GimbalManagerConfigure :md5sum-) "33077e29a7a8218a30def4cf603efdb4")
(setf (get mavros_msgs::GimbalManagerConfigure :datatype-) "mavros_msgs/GimbalManagerConfigure")
(setf (get mavros_msgs::GimbalManagerConfigure :request) mavros_msgs::GimbalManagerConfigureRequest)
(setf (get mavros_msgs::GimbalManagerConfigure :response) mavros_msgs::GimbalManagerConfigureResponse)

(defmethod mavros_msgs::GimbalManagerConfigureRequest
  (:response () (instance mavros_msgs::GimbalManagerConfigureResponse :init)))

(setf (get mavros_msgs::GimbalManagerConfigureRequest :md5sum-) "33077e29a7a8218a30def4cf603efdb4")
(setf (get mavros_msgs::GimbalManagerConfigureRequest :datatype-) "mavros_msgs/GimbalManagerConfigureRequest")
(setf (get mavros_msgs::GimbalManagerConfigureRequest :definition-)
      "# MAVLink command: DO_GIMBAL_MANAGER_CONFIGURE
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
# Note: default MAV_COMP_ID_ONBOARD_COMPUTER = 191, see MAV_COMPONENT documentation
# https://mavlink.io/en/messages/common.html#MAV_COMPONENT

int16 sysid_primary      # Sysid for primary control (0: no one in control, 
                        # -1: leave unchanged, -2: set itself in control 
                        # (for missions where the own sysid is still unknown), 
                        # -3: remove control if currently in control).
int16 compid_primary     # Compid for primary control (0: no one in control, 
                        # -1: leave unchanged, -2: set itself in control 
                        # (for missions where the own sysid is still unknown), 
                        # -3: remove control if currently in control).
int16 sysid_secondary    # Sysid for secondary control (0: no one in control, 
                        # -1: leave unchanged, -2: set itself in control 
                        # (for missions where the own sysid is still unknown), 
                        # -3: remove control if currently in control).
int16 compid_secondary   # Compid for secondary control (0: no one in control, 
                        # -1: leave unchanged, -2: set itself in control 
                        # (for missions where the own sysid is still unknown), 
                        # -3: remove control if currently in control).

uint8 gimbal_device_id  # Component ID of gimbal device to address 
                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
                        # components. Send command multiple times for more than
                        # one gimbal (but not all gimbals).
                        # Note: Default Mavlink gimbal device ids: 154, 171-175

---
bool success
# raw result returned by COMMAND_ACK
uint8 result
")

(setf (get mavros_msgs::GimbalManagerConfigureResponse :md5sum-) "33077e29a7a8218a30def4cf603efdb4")
(setf (get mavros_msgs::GimbalManagerConfigureResponse :datatype-) "mavros_msgs/GimbalManagerConfigureResponse")
(setf (get mavros_msgs::GimbalManagerConfigureResponse :definition-)
      "# MAVLink command: DO_GIMBAL_MANAGER_CONFIGURE
# https://mavlink.io/en/messages/common.html#MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE
# Note: default MAV_COMP_ID_ONBOARD_COMPUTER = 191, see MAV_COMPONENT documentation
# https://mavlink.io/en/messages/common.html#MAV_COMPONENT

int16 sysid_primary      # Sysid for primary control (0: no one in control, 
                        # -1: leave unchanged, -2: set itself in control 
                        # (for missions where the own sysid is still unknown), 
                        # -3: remove control if currently in control).
int16 compid_primary     # Compid for primary control (0: no one in control, 
                        # -1: leave unchanged, -2: set itself in control 
                        # (for missions where the own sysid is still unknown), 
                        # -3: remove control if currently in control).
int16 sysid_secondary    # Sysid for secondary control (0: no one in control, 
                        # -1: leave unchanged, -2: set itself in control 
                        # (for missions where the own sysid is still unknown), 
                        # -3: remove control if currently in control).
int16 compid_secondary   # Compid for secondary control (0: no one in control, 
                        # -1: leave unchanged, -2: set itself in control 
                        # (for missions where the own sysid is still unknown), 
                        # -3: remove control if currently in control).

uint8 gimbal_device_id  # Component ID of gimbal device to address 
                        # (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device
                        # components. Send command multiple times for more than
                        # one gimbal (but not all gimbals).
                        # Note: Default Mavlink gimbal device ids: 154, 171-175

---
bool success
# raw result returned by COMMAND_ACK
uint8 result
")



(provide :mavros_msgs/GimbalManagerConfigure "33077e29a7a8218a30def4cf603efdb4")


