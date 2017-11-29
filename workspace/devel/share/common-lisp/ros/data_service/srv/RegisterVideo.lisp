; Auto-generated. Do not edit!


(cl:in-package data_service-srv)


;//! \htmlinclude RegisterVideo-request.msg.html

(cl:defclass <RegisterVideo-request> (roslisp-msg-protocol:ros-message)
  ((experiment
    :reader experiment
    :initarg :experiment
    :type cl:string
    :initform "")
   (path
    :reader path
    :initarg :path
    :type cl:string
    :initform ""))
)

(cl:defclass RegisterVideo-request (<RegisterVideo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegisterVideo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegisterVideo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_service-srv:<RegisterVideo-request> is deprecated: use data_service-srv:RegisterVideo-request instead.")))

(cl:ensure-generic-function 'experiment-val :lambda-list '(m))
(cl:defmethod experiment-val ((m <RegisterVideo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:experiment-val is deprecated.  Use data_service-srv:experiment instead.")
  (experiment m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <RegisterVideo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:path-val is deprecated.  Use data_service-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegisterVideo-request>) ostream)
  "Serializes a message object of type '<RegisterVideo-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'experiment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'experiment))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegisterVideo-request>) istream)
  "Deserializes a message object of type '<RegisterVideo-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'experiment) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'experiment) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegisterVideo-request>)))
  "Returns string type for a service object of type '<RegisterVideo-request>"
  "data_service/RegisterVideoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterVideo-request)))
  "Returns string type for a service object of type 'RegisterVideo-request"
  "data_service/RegisterVideoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegisterVideo-request>)))
  "Returns md5sum for a message object of type '<RegisterVideo-request>"
  "252bc0f1ff2ebd98b97d212fe2c3fb57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegisterVideo-request)))
  "Returns md5sum for a message object of type 'RegisterVideo-request"
  "252bc0f1ff2ebd98b97d212fe2c3fb57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegisterVideo-request>)))
  "Returns full string definition for message of type '<RegisterVideo-request>"
  (cl:format cl:nil "string experiment~%string path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegisterVideo-request)))
  "Returns full string definition for message of type 'RegisterVideo-request"
  (cl:format cl:nil "string experiment~%string path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegisterVideo-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'experiment))
     4 (cl:length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegisterVideo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RegisterVideo-request
    (cl:cons ':experiment (experiment msg))
    (cl:cons ':path (path msg))
))
;//! \htmlinclude RegisterVideo-response.msg.html

(cl:defclass <RegisterVideo-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass RegisterVideo-response (<RegisterVideo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegisterVideo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegisterVideo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_service-srv:<RegisterVideo-response> is deprecated: use data_service-srv:RegisterVideo-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <RegisterVideo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:response-val is deprecated.  Use data_service-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegisterVideo-response>) ostream)
  "Serializes a message object of type '<RegisterVideo-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegisterVideo-response>) istream)
  "Deserializes a message object of type '<RegisterVideo-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegisterVideo-response>)))
  "Returns string type for a service object of type '<RegisterVideo-response>"
  "data_service/RegisterVideoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterVideo-response)))
  "Returns string type for a service object of type 'RegisterVideo-response"
  "data_service/RegisterVideoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegisterVideo-response>)))
  "Returns md5sum for a message object of type '<RegisterVideo-response>"
  "252bc0f1ff2ebd98b97d212fe2c3fb57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegisterVideo-response)))
  "Returns md5sum for a message object of type 'RegisterVideo-response"
  "252bc0f1ff2ebd98b97d212fe2c3fb57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegisterVideo-response>)))
  "Returns full string definition for message of type '<RegisterVideo-response>"
  (cl:format cl:nil "string response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegisterVideo-response)))
  "Returns full string definition for message of type 'RegisterVideo-response"
  (cl:format cl:nil "string response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegisterVideo-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegisterVideo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RegisterVideo-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RegisterVideo)))
  'RegisterVideo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RegisterVideo)))
  'RegisterVideo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterVideo)))
  "Returns string type for a service object of type '<RegisterVideo>"
  "data_service/RegisterVideo")