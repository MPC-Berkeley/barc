; Auto-generated. Do not edit!


(cl:in-package barc-msg)


;//! \htmlinclude Ultrasound.msg.html

(cl:defclass <Ultrasound> (roslisp-msg-protocol:ros-message)
  ((front
    :reader front
    :initarg :front
    :type cl:float
    :initform 0.0)
   (back
    :reader back
    :initarg :back
    :type cl:float
    :initform 0.0)
   (right
    :reader right
    :initarg :right
    :type cl:float
    :initform 0.0)
   (left
    :reader left
    :initarg :left
    :type cl:float
    :initform 0.0))
)

(cl:defclass Ultrasound (<Ultrasound>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Ultrasound>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Ultrasound)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name barc-msg:<Ultrasound> is deprecated: use barc-msg:Ultrasound instead.")))

(cl:ensure-generic-function 'front-val :lambda-list '(m))
(cl:defmethod front-val ((m <Ultrasound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:front-val is deprecated.  Use barc-msg:front instead.")
  (front m))

(cl:ensure-generic-function 'back-val :lambda-list '(m))
(cl:defmethod back-val ((m <Ultrasound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:back-val is deprecated.  Use barc-msg:back instead.")
  (back m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <Ultrasound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:right-val is deprecated.  Use barc-msg:right instead.")
  (right m))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <Ultrasound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:left-val is deprecated.  Use barc-msg:left instead.")
  (left m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Ultrasound>) ostream)
  "Serializes a message object of type '<Ultrasound>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'front))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'back))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Ultrasound>) istream)
  "Deserializes a message object of type '<Ultrasound>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'front) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'back) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Ultrasound>)))
  "Returns string type for a message object of type '<Ultrasound>"
  "barc/Ultrasound")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Ultrasound)))
  "Returns string type for a message object of type 'Ultrasound"
  "barc/Ultrasound")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Ultrasound>)))
  "Returns md5sum for a message object of type '<Ultrasound>"
  "a28144c3266e5b701faa1e4761aa50f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Ultrasound)))
  "Returns md5sum for a message object of type 'Ultrasound"
  "a28144c3266e5b701faa1e4761aa50f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Ultrasound>)))
  "Returns full string definition for message of type '<Ultrasound>"
  (cl:format cl:nil "float32 front~%float32 back~%float32 right~%float32 left~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Ultrasound)))
  "Returns full string definition for message of type 'Ultrasound"
  (cl:format cl:nil "float32 front~%float32 back~%float32 right~%float32 left~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Ultrasound>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Ultrasound>))
  "Converts a ROS message object to a list"
  (cl:list 'Ultrasound
    (cl:cons ':front (front msg))
    (cl:cons ':back (back msg))
    (cl:cons ':right (right msg))
    (cl:cons ':left (left msg))
))
