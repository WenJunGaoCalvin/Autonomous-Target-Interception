; Auto-generated. Do not edit!


(cl:in-package mavros_msgs-srv)


;//! \htmlinclude GimbalGetInformation-request.msg.html

(cl:defclass <GimbalGetInformation-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GimbalGetInformation-request (<GimbalGetInformation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalGetInformation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalGetInformation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-srv:<GimbalGetInformation-request> is deprecated: use mavros_msgs-srv:GimbalGetInformation-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalGetInformation-request>) ostream)
  "Serializes a message object of type '<GimbalGetInformation-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalGetInformation-request>) istream)
  "Deserializes a message object of type '<GimbalGetInformation-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalGetInformation-request>)))
  "Returns string type for a service object of type '<GimbalGetInformation-request>"
  "mavros_msgs/GimbalGetInformationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalGetInformation-request)))
  "Returns string type for a service object of type 'GimbalGetInformation-request"
  "mavros_msgs/GimbalGetInformationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalGetInformation-request>)))
  "Returns md5sum for a message object of type '<GimbalGetInformation-request>"
  "1cd894375e4e3d2861d2222772894fdb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalGetInformation-request)))
  "Returns md5sum for a message object of type 'GimbalGetInformation-request"
  "1cd894375e4e3d2861d2222772894fdb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalGetInformation-request>)))
  "Returns full string definition for message of type '<GimbalGetInformation-request>"
  (cl:format cl:nil "# MAVLink command: MAV_CMD_REQUEST_MESSAGE~%# https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE~%# Specifically used to request Information messages from Gimbal Device and Gimbal Manager~%# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION~%# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalGetInformation-request)))
  "Returns full string definition for message of type 'GimbalGetInformation-request"
  (cl:format cl:nil "# MAVLink command: MAV_CMD_REQUEST_MESSAGE~%# https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE~%# Specifically used to request Information messages from Gimbal Device and Gimbal Manager~%# https://mavlink.io/en/messages/common.html#GIMBAL_MANAGER_INFORMATION~%# https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_INFORMATION~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalGetInformation-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalGetInformation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalGetInformation-request
))
;//! \htmlinclude GimbalGetInformation-response.msg.html

(cl:defclass <GimbalGetInformation-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GimbalGetInformation-response (<GimbalGetInformation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GimbalGetInformation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GimbalGetInformation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mavros_msgs-srv:<GimbalGetInformation-response> is deprecated: use mavros_msgs-srv:GimbalGetInformation-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GimbalGetInformation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:success-val is deprecated.  Use mavros_msgs-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <GimbalGetInformation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mavros_msgs-srv:result-val is deprecated.  Use mavros_msgs-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GimbalGetInformation-response>) ostream)
  "Serializes a message object of type '<GimbalGetInformation-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GimbalGetInformation-response>) istream)
  "Deserializes a message object of type '<GimbalGetInformation-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GimbalGetInformation-response>)))
  "Returns string type for a service object of type '<GimbalGetInformation-response>"
  "mavros_msgs/GimbalGetInformationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalGetInformation-response)))
  "Returns string type for a service object of type 'GimbalGetInformation-response"
  "mavros_msgs/GimbalGetInformationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GimbalGetInformation-response>)))
  "Returns md5sum for a message object of type '<GimbalGetInformation-response>"
  "1cd894375e4e3d2861d2222772894fdb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GimbalGetInformation-response)))
  "Returns md5sum for a message object of type 'GimbalGetInformation-response"
  "1cd894375e4e3d2861d2222772894fdb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GimbalGetInformation-response>)))
  "Returns full string definition for message of type '<GimbalGetInformation-response>"
  (cl:format cl:nil "bool success~%# raw result returned by COMMAND_ACK~%uint8 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GimbalGetInformation-response)))
  "Returns full string definition for message of type 'GimbalGetInformation-response"
  (cl:format cl:nil "bool success~%# raw result returned by COMMAND_ACK~%uint8 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GimbalGetInformation-response>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GimbalGetInformation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GimbalGetInformation-response
    (cl:cons ':success (success msg))
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GimbalGetInformation)))
  'GimbalGetInformation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GimbalGetInformation)))
  'GimbalGetInformation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GimbalGetInformation)))
  "Returns string type for a service object of type '<GimbalGetInformation>"
  "mavros_msgs/GimbalGetInformation")