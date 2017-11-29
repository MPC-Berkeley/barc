; Auto-generated. Do not edit!


(cl:in-package data_service-msg)


;//! \htmlinclude CustomSignal.msg.html

(cl:defclass <CustomSignal> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:string
    :initform "")
   (signal
    :reader signal
    :initarg :signal
    :type cl:string
    :initform ""))
)

(cl:defclass CustomSignal (<CustomSignal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CustomSignal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CustomSignal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_service-msg:<CustomSignal> is deprecated: use data_service-msg:CustomSignal instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <CustomSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-msg:id-val is deprecated.  Use data_service-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'signal-val :lambda-list '(m))
(cl:defmethod signal-val ((m <CustomSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-msg:signal-val is deprecated.  Use data_service-msg:signal instead.")
  (signal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CustomSignal>) ostream)
  "Serializes a message object of type '<CustomSignal>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'signal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'signal))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CustomSignal>) istream)
  "Deserializes a message object of type '<CustomSignal>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'signal) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'signal) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CustomSignal>)))
  "Returns string type for a message object of type '<CustomSignal>"
  "data_service/CustomSignal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CustomSignal)))
  "Returns string type for a message object of type 'CustomSignal"
  "data_service/CustomSignal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CustomSignal>)))
  "Returns md5sum for a message object of type '<CustomSignal>"
  "952635fef7c9f8f6266bdfc127ea52f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CustomSignal)))
  "Returns md5sum for a message object of type 'CustomSignal"
  "952635fef7c9f8f6266bdfc127ea52f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CustomSignal>)))
  "Returns full string definition for message of type '<CustomSignal>"
  (cl:format cl:nil "string id~%string signal~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CustomSignal)))
  "Returns full string definition for message of type 'CustomSignal"
  (cl:format cl:nil "string id~%string signal~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CustomSignal>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'id))
     4 (cl:length (cl:slot-value msg 'signal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CustomSignal>))
  "Converts a ROS message object to a list"
  (cl:list 'CustomSignal
    (cl:cons ':id (id msg))
    (cl:cons ':signal (signal msg))
))
