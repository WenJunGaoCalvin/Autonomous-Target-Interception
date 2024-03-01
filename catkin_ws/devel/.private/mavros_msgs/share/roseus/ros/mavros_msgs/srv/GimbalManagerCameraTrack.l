;; Auto-generated. Do not edit!


(when (boundp 'mavros_msgs::GimbalManagerCameraTrack)
  (if (not (find-package "MAVROS_MSGS"))
    (make-package "MAVROS_MSGS"))
  (shadow 'GimbalManagerCameraTrack (find-package "MAVROS_MSGS")))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERCAMERATRACK")
  (make-package "MAVROS_MSGS::GIMBALMANAGERCAMERATRACK"))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERCAMERATRACKREQUEST")
  (make-package "MAVROS_MSGS::GIMBALMANAGERCAMERATRACKREQUEST"))
(unless (find-package "MAVROS_MSGS::GIMBALMANAGERCAMERATRACKRESPONSE")
  (make-package "MAVROS_MSGS::GIMBALMANAGERCAMERATRACKRESPONSE"))

(in-package "ROS")





(intern "*CAMERA_TRACK_MODE_POINT*" (find-package "MAVROS_MSGS::GIMBALMANAGERCAMERATRACKREQUEST"))
(shadow '*CAMERA_TRACK_MODE_POINT* (find-package "MAVROS_MSGS::GIMBALMANAGERCAMERATRACKREQUEST"))
(defconstant mavros_msgs::GimbalManagerCameraTrackRequest::*CAMERA_TRACK_MODE_POINT* 0)
(intern "*CAMERA_TRACK_MODE_RECTANGLE*" (find-package "MAVROS_MSGS::GIMBALMANAGERCAMERATRACKREQUEST"))
(shadow '*CAMERA_TRACK_MODE_RECTANGLE* (find-package "MAVROS_MSGS::GIMBALMANAGERCAMERATRACKREQUEST"))
(defconstant mavros_msgs::GimbalManagerCameraTrackRequest::*CAMERA_TRACK_MODE_RECTANGLE* 1)
(intern "*CAMERA_TRACK_MODE_STOP_TRACKING*" (find-package "MAVROS_MSGS::GIMBALMANAGERCAMERATRACKREQUEST"))
(shadow '*CAMERA_TRACK_MODE_STOP_TRACKING* (find-package "MAVROS_MSGS::GIMBALMANAGERCAMERATRACKREQUEST"))
(defconstant mavros_msgs::GimbalManagerCameraTrackRequest::*CAMERA_TRACK_MODE_STOP_TRACKING* 2)

(defun mavros_msgs::GimbalManagerCameraTrackRequest-to-symbol (const)
  (cond
        ((= const 0) 'mavros_msgs::GimbalManagerCameraTrackRequest::*CAMERA_TRACK_MODE_POINT*)
        ((= const 1) 'mavros_msgs::GimbalManagerCameraTrackRequest::*CAMERA_TRACK_MODE_RECTANGLE*)
        ((= const 2) 'mavros_msgs::GimbalManagerCameraTrackRequest::*CAMERA_TRACK_MODE_STOP_TRACKING*)
        (t nil)))

(defclass mavros_msgs::GimbalManagerCameraTrackRequest
  :super ros::object
  :slots (_mode _x _y _radius _top_left_x _top_left_y _bottom_right_x _bottom_right_y ))

(defmethod mavros_msgs::GimbalManagerCameraTrackRequest
  (:init
   (&key
    ((:mode __mode) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:radius __radius) 0.0)
    ((:top_left_x __top_left_x) 0.0)
    ((:top_left_y __top_left_y) 0.0)
    ((:bottom_right_x __bottom_right_x) 0.0)
    ((:bottom_right_y __bottom_right_y) 0.0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _radius (float __radius))
   (setq _top_left_x (float __top_left_x))
   (setq _top_left_y (float __top_left_y))
   (setq _bottom_right_x (float __bottom_right_x))
   (setq _bottom_right_y (float __bottom_right_y))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:top_left_x
   (&optional __top_left_x)
   (if __top_left_x (setq _top_left_x __top_left_x)) _top_left_x)
  (:top_left_y
   (&optional __top_left_y)
   (if __top_left_y (setq _top_left_y __top_left_y)) _top_left_y)
  (:bottom_right_x
   (&optional __bottom_right_x)
   (if __bottom_right_x (setq _bottom_right_x __bottom_right_x)) _bottom_right_x)
  (:bottom_right_y
   (&optional __bottom_right_y)
   (if __bottom_right_y (setq _bottom_right_y __bottom_right_y)) _bottom_right_y)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _radius
    4
    ;; float32 _top_left_x
    4
    ;; float32 _top_left_y
    4
    ;; float32 _bottom_right_x
    4
    ;; float32 _bottom_right_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _top_left_x
       (sys::poke _top_left_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _top_left_y
       (sys::poke _top_left_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bottom_right_x
       (sys::poke _bottom_right_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bottom_right_y
       (sys::poke _bottom_right_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _radius
     (setq _radius (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _top_left_x
     (setq _top_left_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _top_left_y
     (setq _top_left_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bottom_right_x
     (setq _bottom_right_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bottom_right_y
     (setq _bottom_right_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mavros_msgs::GimbalManagerCameraTrackResponse
  :super ros::object
  :slots (_success _result ))

(defmethod mavros_msgs::GimbalManagerCameraTrackResponse
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

(defclass mavros_msgs::GimbalManagerCameraTrack
  :super ros::object
  :slots ())

(setf (get mavros_msgs::GimbalManagerCameraTrack :md5sum-) "b75e6fe30849c24c4458c6930214e491")
(setf (get mavros_msgs::GimbalManagerCameraTrack :datatype-) "mavros_msgs/GimbalManagerCameraTrack")
(setf (get mavros_msgs::GimbalManagerCameraTrack :request) mavros_msgs::GimbalManagerCameraTrackRequest)
(setf (get mavros_msgs::GimbalManagerCameraTrack :response) mavros_msgs::GimbalManagerCameraTrackResponse)

(defmethod mavros_msgs::GimbalManagerCameraTrackRequest
  (:response () (instance mavros_msgs::GimbalManagerCameraTrackResponse :init)))

(setf (get mavros_msgs::GimbalManagerCameraTrackRequest :md5sum-) "b75e6fe30849c24c4458c6930214e491")
(setf (get mavros_msgs::GimbalManagerCameraTrackRequest :datatype-) "mavros_msgs/GimbalManagerCameraTrackRequest")
(setf (get mavros_msgs::GimbalManagerCameraTrackRequest :definition-)
      "# MAVLink commands: CAMERA_TRACK_POINT, CAMERA_TRACK_RECTANGLE, CAMERA_STOP_TRACKING
# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_POINT
# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_RECTANGLE
# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_STOP_TRACKING

uint8 mode      # enumerator to indicate camera track mode setting - see CAMERA_TRACK_MODE
#CAMERA_TRACK_MODE
uint8 CAMERA_TRACK_MODE_POINT = 0           # If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking. [CAMERA_TRACK_POINT]
uint8 CAMERA_TRACK_MODE_RECTANGLE = 1       # If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking. [CAMERA_TRACK_RECTANGLE]
uint8 CAMERA_TRACK_MODE_STOP_TRACKING = 2   # Stops ongoing tracking. [CAMERA_STOP_TRACKING]

#For CAMERA_TRACK_POINT
float32 x       # Point to track x value (normalized 0..1, 0 is left, 1 is right).
float32 y       # Point to track y value (normalized 0..1, 0 is top, 1 is bottom).
float32 radius  # Point radius (normalized 0..1, 0 is image left, 1 is image right).

#For CAMERA_TRACK_RECTANGLE
float32 top_left_x      # Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
float32 top_left_y      # Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
float32 bottom_right_x  # Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
float32 bottom_right_y  # Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).

#CAMERA_STOP_TRACKING doesn't take extra parameters

---
bool success
# raw result returned by COMMAND_ACK
uint8 result
")

(setf (get mavros_msgs::GimbalManagerCameraTrackResponse :md5sum-) "b75e6fe30849c24c4458c6930214e491")
(setf (get mavros_msgs::GimbalManagerCameraTrackResponse :datatype-) "mavros_msgs/GimbalManagerCameraTrackResponse")
(setf (get mavros_msgs::GimbalManagerCameraTrackResponse :definition-)
      "# MAVLink commands: CAMERA_TRACK_POINT, CAMERA_TRACK_RECTANGLE, CAMERA_STOP_TRACKING
# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_POINT
# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_RECTANGLE
# https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_STOP_TRACKING

uint8 mode      # enumerator to indicate camera track mode setting - see CAMERA_TRACK_MODE
#CAMERA_TRACK_MODE
uint8 CAMERA_TRACK_MODE_POINT = 0           # If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking. [CAMERA_TRACK_POINT]
uint8 CAMERA_TRACK_MODE_RECTANGLE = 1       # If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking. [CAMERA_TRACK_RECTANGLE]
uint8 CAMERA_TRACK_MODE_STOP_TRACKING = 2   # Stops ongoing tracking. [CAMERA_STOP_TRACKING]

#For CAMERA_TRACK_POINT
float32 x       # Point to track x value (normalized 0..1, 0 is left, 1 is right).
float32 y       # Point to track y value (normalized 0..1, 0 is top, 1 is bottom).
float32 radius  # Point radius (normalized 0..1, 0 is image left, 1 is image right).

#For CAMERA_TRACK_RECTANGLE
float32 top_left_x      # Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
float32 top_left_y      # Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).
float32 bottom_right_x  # Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).
float32 bottom_right_y  # Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).

#CAMERA_STOP_TRACKING doesn't take extra parameters

---
bool success
# raw result returned by COMMAND_ACK
uint8 result
")



(provide :mavros_msgs/GimbalManagerCameraTrack "b75e6fe30849c24c4458c6930214e491")


