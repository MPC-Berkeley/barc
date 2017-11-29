; Auto-generated. Do not edit!


(cl:in-package data_service-srv)


;//! \htmlinclude RegisterExperiment-request.msg.html

(cl:defclass <RegisterExperiment-request> (roslisp-msg-protocol:ros-message)
  ((experiment
    :reader experiment
    :initarg :experiment
    :type cl:string
    :initform ""))
)

(cl:defclass RegisterExperiment-request (<RegisterExperiment-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegisterExperiment-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegisterExperiment-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_service-srv:<RegisterExperiment-request> is deprecated: use data_service-srv:RegisterExperiment-request instead.")))

(cl:ensure-generic-function 'experiment-val :lambda-list '(m))
(cl:defmethod experiment-val ((m <RegisterExperiment-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:experiment-val is deprecated.  Use data_service-srv:experiment instead.")
  (experiment m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegisterExperiment-request>) ostream)
  "Serializes a message object of type '<RegisterExperiment-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'experiment))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'experiment))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegisterExperiment-request>) istream)
  "Deserializes a message object of type '<RegisterExperiment-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'experiment) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'experiment) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegisterExperiment-request>)))
  "Returns string type for a service object of type '<RegisterExperiment-request>"
  "data_service/RegisterExperimentRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterExperiment-request)))
  "Returns string type for a service object of type 'RegisterExperiment-request"
  "data_service/RegisterExperimentRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegisterExperiment-request>)))
  "Returns md5sum for a message object of type '<RegisterExperiment-request>"
  "23efeb8a860ffcd264ebd586a8c57078")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegisterExperiment-request)))
  "Returns md5sum for a message object of type 'RegisterExperiment-request"
  "23efeb8a860ffcd264ebd586a8c57078")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegisterExperiment-request>)))
  "Returns full string definition for message of type '<RegisterExperiment-request>"
  (cl:format cl:nil "string experiment~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegisterExperiment-request)))
  "Returns full string definition for message of type 'RegisterExperiment-request"
  (cl:format cl:nil "string experiment~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegisterExperiment-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'experiment))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegisterExperiment-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RegisterExperiment-request
    (cl:cons ':experiment (experiment msg))
))
;//! \htmlinclude RegisterExperiment-response.msg.html

(cl:defclass <RegisterExperiment-response> (roslisp-msg-protocol:ros-message)
  ((experiment_id
    :reader experiment_id
    :initarg :experiment_id
    :type cl:integer
    :initform 0))
)

(cl:defclass RegisterExperiment-response (<RegisterExperiment-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RegisterExperiment-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RegisterExperiment-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_service-srv:<RegisterExperiment-response> is deprecated: use data_service-srv:RegisterExperiment-response instead.")))

(cl:ensure-generic-function 'experiment_id-val :lambda-list '(m))
(cl:defmethod experiment_id-val ((m <RegisterExperiment-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:experiment_id-val is deprecated.  Use data_service-srv:experiment_id instead.")
  (experiment_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RegisterExperiment-response>) ostream)
  "Serializes a message object of type '<RegisterExperiment-response>"
  (cl:let* ((signed (cl:slot-value msg 'experiment_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RegisterExperiment-response>) istream)
  "Deserializes a message object of type '<RegisterExperiment-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'experiment_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RegisterExperiment-response>)))
  "Returns string type for a service object of type '<RegisterExperiment-response>"
  "data_service/RegisterExperimentResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterExperiment-response)))
  "Returns string type for a service object of type 'RegisterExperiment-response"
  "data_service/RegisterExperimentResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RegisterExperiment-response>)))
  "Returns md5sum for a message object of type '<RegisterExperiment-response>"
  "23efeb8a860ffcd264ebd586a8c57078")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RegisterExperiment-response)))
  "Returns md5sum for a message object of type 'RegisterExperiment-response"
  "23efeb8a860ffcd264ebd586a8c57078")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RegisterExperiment-response>)))
  "Returns full string definition for message of type '<RegisterExperiment-response>"
  (cl:format cl:nil "int32 experiment_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RegisterExperiment-response)))
  "Returns full string definition for message of type 'RegisterExperiment-response"
  (cl:format cl:nil "int32 experiment_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RegisterExperiment-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RegisterExperiment-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RegisterExperiment-response
    (cl:cons ':experiment_id (experiment_id msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RegisterExperiment)))
  'RegisterExperiment-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RegisterExperiment)))
  'RegisterExperiment-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RegisterExperiment)))
  "Returns string type for a service object of type '<RegisterExperiment>"
  "data_service/RegisterExperiment")