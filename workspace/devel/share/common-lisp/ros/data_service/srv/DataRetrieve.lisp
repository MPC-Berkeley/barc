; Auto-generated. Do not edit!


(cl:in-package data_service-srv)


;//! \htmlinclude DataRetrieve-request.msg.html

(cl:defclass <DataRetrieve-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:string
    :initform "")
   (is_time
    :reader is_time
    :initarg :is_time
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass DataRetrieve-request (<DataRetrieve-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DataRetrieve-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DataRetrieve-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_service-srv:<DataRetrieve-request> is deprecated: use data_service-srv:DataRetrieve-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <DataRetrieve-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:id-val is deprecated.  Use data_service-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'is_time-val :lambda-list '(m))
(cl:defmethod is_time-val ((m <DataRetrieve-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:is_time-val is deprecated.  Use data_service-srv:is_time instead.")
  (is_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DataRetrieve-request>) ostream)
  "Serializes a message object of type '<DataRetrieve-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_time) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DataRetrieve-request>) istream)
  "Deserializes a message object of type '<DataRetrieve-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'is_time) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DataRetrieve-request>)))
  "Returns string type for a service object of type '<DataRetrieve-request>"
  "data_service/DataRetrieveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataRetrieve-request)))
  "Returns string type for a service object of type 'DataRetrieve-request"
  "data_service/DataRetrieveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DataRetrieve-request>)))
  "Returns md5sum for a message object of type '<DataRetrieve-request>"
  "f58abbeb8edc41005a341386540c1d0b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DataRetrieve-request)))
  "Returns md5sum for a message object of type 'DataRetrieve-request"
  "f58abbeb8edc41005a341386540c1d0b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DataRetrieve-request>)))
  "Returns full string definition for message of type '<DataRetrieve-request>"
  (cl:format cl:nil "string id~%bool is_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DataRetrieve-request)))
  "Returns full string definition for message of type 'DataRetrieve-request"
  (cl:format cl:nil "string id~%bool is_time~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DataRetrieve-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'id))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DataRetrieve-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DataRetrieve-request
    (cl:cons ':id (id msg))
    (cl:cons ':is_time (is_time msg))
))
;//! \htmlinclude DataRetrieve-response.msg.html

(cl:defclass <DataRetrieve-response> (roslisp-msg-protocol:ros-message)
  ((signal
    :reader signal
    :initarg :signal
    :type data_service-msg:CustomSignal
    :initform (cl:make-instance 'data_service-msg:CustomSignal)))
)

(cl:defclass DataRetrieve-response (<DataRetrieve-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DataRetrieve-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DataRetrieve-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_service-srv:<DataRetrieve-response> is deprecated: use data_service-srv:DataRetrieve-response instead.")))

(cl:ensure-generic-function 'signal-val :lambda-list '(m))
(cl:defmethod signal-val ((m <DataRetrieve-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:signal-val is deprecated.  Use data_service-srv:signal instead.")
  (signal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DataRetrieve-response>) ostream)
  "Serializes a message object of type '<DataRetrieve-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'signal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DataRetrieve-response>) istream)
  "Deserializes a message object of type '<DataRetrieve-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'signal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DataRetrieve-response>)))
  "Returns string type for a service object of type '<DataRetrieve-response>"
  "data_service/DataRetrieveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataRetrieve-response)))
  "Returns string type for a service object of type 'DataRetrieve-response"
  "data_service/DataRetrieveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DataRetrieve-response>)))
  "Returns md5sum for a message object of type '<DataRetrieve-response>"
  "f58abbeb8edc41005a341386540c1d0b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DataRetrieve-response)))
  "Returns md5sum for a message object of type 'DataRetrieve-response"
  "f58abbeb8edc41005a341386540c1d0b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DataRetrieve-response>)))
  "Returns full string definition for message of type '<DataRetrieve-response>"
  (cl:format cl:nil "CustomSignal signal~%~%================================================================================~%MSG: data_service/CustomSignal~%string id~%string signal~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DataRetrieve-response)))
  "Returns full string definition for message of type 'DataRetrieve-response"
  (cl:format cl:nil "CustomSignal signal~%~%================================================================================~%MSG: data_service/CustomSignal~%string id~%string signal~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DataRetrieve-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'signal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DataRetrieve-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DataRetrieve-response
    (cl:cons ':signal (signal msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DataRetrieve)))
  'DataRetrieve-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DataRetrieve)))
  'DataRetrieve-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataRetrieve)))
  "Returns string type for a service object of type '<DataRetrieve>"
  "data_service/DataRetrieve")