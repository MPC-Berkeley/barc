; Auto-generated. Do not edit!


(cl:in-package data_service-srv)


;//! \htmlinclude DataForward-request.msg.html

(cl:defclass <DataForward-request> (roslisp-msg-protocol:ros-message)
  ((time_signal
    :reader time_signal
    :initarg :time_signal
    :type data_service-msg:TimeSignal
    :initform (cl:make-instance 'data_service-msg:TimeSignal))
   (custom_signal
    :reader custom_signal
    :initarg :custom_signal
    :type data_service-msg:CustomSignal
    :initform (cl:make-instance 'data_service-msg:CustomSignal))
   (experiment_name
    :reader experiment_name
    :initarg :experiment_name
    :type cl:string
    :initform ""))
)

(cl:defclass DataForward-request (<DataForward-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DataForward-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DataForward-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_service-srv:<DataForward-request> is deprecated: use data_service-srv:DataForward-request instead.")))

(cl:ensure-generic-function 'time_signal-val :lambda-list '(m))
(cl:defmethod time_signal-val ((m <DataForward-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:time_signal-val is deprecated.  Use data_service-srv:time_signal instead.")
  (time_signal m))

(cl:ensure-generic-function 'custom_signal-val :lambda-list '(m))
(cl:defmethod custom_signal-val ((m <DataForward-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:custom_signal-val is deprecated.  Use data_service-srv:custom_signal instead.")
  (custom_signal m))

(cl:ensure-generic-function 'experiment_name-val :lambda-list '(m))
(cl:defmethod experiment_name-val ((m <DataForward-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:experiment_name-val is deprecated.  Use data_service-srv:experiment_name instead.")
  (experiment_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DataForward-request>) ostream)
  "Serializes a message object of type '<DataForward-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'time_signal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'custom_signal) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'experiment_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'experiment_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DataForward-request>) istream)
  "Deserializes a message object of type '<DataForward-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'time_signal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'custom_signal) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'experiment_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'experiment_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DataForward-request>)))
  "Returns string type for a service object of type '<DataForward-request>"
  "data_service/DataForwardRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataForward-request)))
  "Returns string type for a service object of type 'DataForward-request"
  "data_service/DataForwardRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DataForward-request>)))
  "Returns md5sum for a message object of type '<DataForward-request>"
  "f8d627aa29376505cccbe1058f0ed9d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DataForward-request)))
  "Returns md5sum for a message object of type 'DataForward-request"
  "f8d627aa29376505cccbe1058f0ed9d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DataForward-request>)))
  "Returns full string definition for message of type '<DataForward-request>"
  (cl:format cl:nil "TimeSignal time_signal~%CustomSignal custom_signal~%string experiment_name~%~%================================================================================~%MSG: data_service/TimeSignal~%string name~%float64[] timestamps~%string signal~%~%================================================================================~%MSG: data_service/CustomSignal~%string id~%string signal~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DataForward-request)))
  "Returns full string definition for message of type 'DataForward-request"
  (cl:format cl:nil "TimeSignal time_signal~%CustomSignal custom_signal~%string experiment_name~%~%================================================================================~%MSG: data_service/TimeSignal~%string name~%float64[] timestamps~%string signal~%~%================================================================================~%MSG: data_service/CustomSignal~%string id~%string signal~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DataForward-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'time_signal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'custom_signal))
     4 (cl:length (cl:slot-value msg 'experiment_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DataForward-request>))
  "Converts a ROS message object to a list"
  (cl:list 'DataForward-request
    (cl:cons ':time_signal (time_signal msg))
    (cl:cons ':custom_signal (custom_signal msg))
    (cl:cons ':experiment_name (experiment_name msg))
))
;//! \htmlinclude DataForward-response.msg.html

(cl:defclass <DataForward-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass DataForward-response (<DataForward-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DataForward-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DataForward-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_service-srv:<DataForward-response> is deprecated: use data_service-srv:DataForward-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <DataForward-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-srv:response-val is deprecated.  Use data_service-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DataForward-response>) ostream)
  "Serializes a message object of type '<DataForward-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DataForward-response>) istream)
  "Deserializes a message object of type '<DataForward-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DataForward-response>)))
  "Returns string type for a service object of type '<DataForward-response>"
  "data_service/DataForwardResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataForward-response)))
  "Returns string type for a service object of type 'DataForward-response"
  "data_service/DataForwardResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DataForward-response>)))
  "Returns md5sum for a message object of type '<DataForward-response>"
  "f8d627aa29376505cccbe1058f0ed9d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DataForward-response)))
  "Returns md5sum for a message object of type 'DataForward-response"
  "f8d627aa29376505cccbe1058f0ed9d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DataForward-response>)))
  "Returns full string definition for message of type '<DataForward-response>"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DataForward-response)))
  "Returns full string definition for message of type 'DataForward-response"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DataForward-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DataForward-response>))
  "Converts a ROS message object to a list"
  (cl:list 'DataForward-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'DataForward)))
  'DataForward-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'DataForward)))
  'DataForward-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DataForward)))
  "Returns string type for a service object of type '<DataForward>"
  "data_service/DataForward")