; Auto-generated. Do not edit!


(cl:in-package data_service-srv)


;//! \htmlinclude RegisterSetting-request.msg.html

(cl:defclass <RegisterSetting-request> (roslisp-msg-protocol:ros-message)
  ((key
    :reader key
    :initarg :key
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:string
    :initform ""))
)

(cl:defclass RegisterSetting-request (<RegisterSetting-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegisterSetting-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegisterSetting-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_service-srv:<RegisterSetting-request> is deprecated: use data_service-srv:RegisterSetting-request instead.")))

(cl:ensure-generic-function 'key-val :lambda-list '(m))
(cl:defmethod key-val ((m <RegisterSetting-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:key-val is deprecated.  Use data_service-srv:key instead.")
  (key m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <RegisterSetting-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:value-val is deprecated.  Use data_service-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegisterSetting-request>) ostream)
  "Serializes a message object of type '<RegisterSetting-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'key))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'key))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegisterSetting-request>) istream)
  "Deserializes a message object of type '<RegisterSetting-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'key) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'key) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'value) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegisterSetting-request>)))
  "Returns string type for a service object of type '<RegisterSetting-request>"
  "data_service/RegisterSettingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterSetting-request)))
  "Returns string type for a service object of type 'RegisterSetting-request"
  "data_service/RegisterSettingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegisterSetting-request>)))
  "Returns md5sum for a message object of type '<RegisterSetting-request>"
  "c268a698e75a634c50aaf2d35ec4caf2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegisterSetting-request)))
  "Returns md5sum for a message object of type 'RegisterSetting-request"
  "c268a698e75a634c50aaf2d35ec4caf2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegisterSetting-request>)))
  "Returns full string definition for message of type '<RegisterSetting-request>"
  (cl:format cl:nil "string key~%string value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegisterSetting-request)))
  "Returns full string definition for message of type 'RegisterSetting-request"
  (cl:format cl:nil "string key~%string value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegisterSetting-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'key))
     4 (cl:length (cl:slot-value msg 'value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegisterSetting-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RegisterSetting-request
    (cl:cons ':key (key msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude RegisterSetting-response.msg.html

(cl:defclass <RegisterSetting-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass RegisterSetting-response (<RegisterSetting-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegisterSetting-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegisterSetting-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_service-srv:<RegisterSetting-response> is deprecated: use data_service-srv:RegisterSetting-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <RegisterSetting-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:response-val is deprecated.  Use data_service-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegisterSetting-response>) ostream)
  "Serializes a message object of type '<RegisterSetting-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegisterSetting-response>) istream)
  "Deserializes a message object of type '<RegisterSetting-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegisterSetting-response>)))
  "Returns string type for a service object of type '<RegisterSetting-response>"
  "data_service/RegisterSettingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterSetting-response)))
  "Returns string type for a service object of type 'RegisterSetting-response"
  "data_service/RegisterSettingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegisterSetting-response>)))
  "Returns md5sum for a message object of type '<RegisterSetting-response>"
  "c268a698e75a634c50aaf2d35ec4caf2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegisterSetting-response)))
  "Returns md5sum for a message object of type 'RegisterSetting-response"
  "c268a698e75a634c50aaf2d35ec4caf2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegisterSetting-response>)))
  "Returns full string definition for message of type '<RegisterSetting-response>"
  (cl:format cl:nil "string response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegisterSetting-response)))
  "Returns full string definition for message of type 'RegisterSetting-response"
  (cl:format cl:nil "string response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegisterSetting-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegisterSetting-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RegisterSetting-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RegisterSetting)))
  'RegisterSetting-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RegisterSetting)))
  'RegisterSetting-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterSetting)))
  "Returns string type for a service object of type '<RegisterSetting>"
  "data_service/RegisterSetting")