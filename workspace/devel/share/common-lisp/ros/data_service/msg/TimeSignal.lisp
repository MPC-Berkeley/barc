; Auto-generated. Do not edit!


(cl:in-package data_service-msg)


;//! \htmlinclude TimeSignal.msg.html

(cl:defclass <TimeSignal> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (timestamps
    :reader timestamps
    :initarg :timestamps
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (signal
    :reader signal
    :initarg :signal
    :type cl:string
    :initform ""))
)

(cl:defclass TimeSignal (<TimeSignal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TimeSignal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TimeSignal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_service-msg:<TimeSignal> is deprecated: use data_service-msg:TimeSignal instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <TimeSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-msg:name-val is deprecated.  Use data_service-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'timestamps-val :lambda-list '(m))
(cl:defmethod timestamps-val ((m <TimeSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-msg:timestamps-val is deprecated.  Use data_service-msg:timestamps instead.")
  (timestamps m))

(cl:ensure-generic-function 'signal-val :lambda-list '(m))
(cl:defmethod signal-val ((m <TimeSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-msg:signal-val is deprecated.  Use data_service-msg:signal instead.")
  (signal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TimeSignal>) ostream)
  "Serializes a message object of type '<TimeSignal>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'timestamps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'timestamps))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'signal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'signal))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TimeSignal>) istream)
  "Deserializes a message object of type '<TimeSignal>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'timestamps) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'timestamps)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TimeSignal>)))
  "Returns string type for a message object of type '<TimeSignal>"
  "data_service/TimeSignal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TimeSignal)))
  "Returns string type for a message object of type 'TimeSignal"
  "data_service/TimeSignal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TimeSignal>)))
  "Returns md5sum for a message object of type '<TimeSignal>"
  "242915a951390ccd66bdffda0979a29d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TimeSignal)))
  "Returns md5sum for a message object of type 'TimeSignal"
  "242915a951390ccd66bdffda0979a29d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TimeSignal>)))
  "Returns full string definition for message of type '<TimeSignal>"
  (cl:format cl:nil "string name~%float64[] timestamps~%string signal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TimeSignal)))
  "Returns full string definition for message of type 'TimeSignal"
  (cl:format cl:nil "string name~%float64[] timestamps~%string signal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TimeSignal>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'timestamps) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:length (cl:slot-value msg 'signal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TimeSignal>))
  "Converts a ROS message object to a list"
  (cl:list 'TimeSignal
    (cl:cons ':name (name msg))
    (cl:cons ':timestamps (timestamps msg))
    (cl:cons ':signal (signal msg))
))
