;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::GimbalGetInformation)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'GimbalGetInformation (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::GIMBALGETINFORMATION")
  (make-package "MAVROS_MSGS::GIMBALGETINFORMATION"))
(unless (find-package "MAVROS_MSGS::GIMBALGETINFORMATIONREQUEST")
  (make-package "MAVROS_MSGS::GIMBALGETINFORMATIONREQUEST"))
(unless (find-package "MAVROS_MSGS::GIMBALGETINFORMATIONRESPONSE")
  (make-package "MAVROS_MSGS::GIMBALGETINFORMATIONRESPONSE"))

(in-package "ROS")





(defclass mavros_msgs::GimbalGetInformationRequest
  :super ros::object
  :slots ())

(defmethod mavros_msgs::GimbalGetInformationRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass mavros_msgs::GimbalGetInformationResponse
  :super ros::object
  :slots (_success _result ))

(defmethod mavros_msgs::GimbalGetInformationResponse
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

(defclass mavros_msgs::GimbalGetInformation
  :super ros::object
  :slots ())

(setf (get mavros_msgs::GimbalGetInformation :md5sum-) "1cd894375e4e3d2861d2222772894fdb")
(setf (get mavros_msgs::GimbalGetInformation :datatype-) "mavros_msgs/GimbalGetInformation")
(setf (get mavros_msgs::GimbalGetInformation :request) mavros_msgs::GimbalGetInformationRequest)
(setf (get mavros_msgs::GimbalGetInformation :response) mavros_msgs::GimbalGetInformationResponse)

(defmethod mavros_msgs::GimbalGetInformationRequest
  (:response () (instance mavros_msgs::GimbalGetInformationResponse :init)))

(setf (get mavros_msgs::GimbalGetInformationRequest :md5sum-) "1cd894375e4e3d2861d2222772894fdb")
(setf (get mavros_msgs::GimbalGetInformationRequest :datatype-) "mavros_msgs/GimbalGetInformationRequest")
(setf (get mavros_msgs::GimbalGetInformationRequest :definition-)
      "# MAVLink command: MAV_CMD_REQUEST_MESSAGE
# https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE
# Specifically used to request Information messages from Gimbal Device and Gimbal Manager
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION
# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION

---
bool success
# raw result returned by COMMAND_ACK
uint8 result
")

(setf (get mavros_msgs::GimbalGetInformationResponse :md5sum-) "1cd894375e4e3d2861d2222772894fdb")
(setf (get mavros_msgs::GimbalGetInformationResponse :datatype-) "mavros_msgs/GimbalGetInformationResponse")
(setf (get mavros_msgs::GimbalGetInformationResponse :definition-)
      "# MAVLink command: MAV_CMD_REQUEST_MESSAGE
# https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE
# Specifically used to request Information messages from Gimbal Device and Gimbal Manager
# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION
# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION

---
bool success
# raw result returned by COMMAND_ACK
uint8 result
")



(provide :mavros_msgs/GimbalGetInformation "1cd894375e4e3d2861d2222772894fdb")


