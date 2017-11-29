; Auto-generated. Do not edit!


(cl:in-package barc-msg)


;//! \htmlinclude Z_KinBkMdl.msg.html

(cl:defclass <Z_KinBkMdl> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (psi
    :reader psi
    :initarg :psi
    :type cl:float
    :initform 0.0)
   (v
    :reader v
    :initarg :v
    :type cl:float
    :initform 0.0))
)

(cl:defclass Z_KinBkMdl (<Z_KinBkMdl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Z_KinBkMdl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Z_KinBkMdl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name barc-msg:<Z_KinBkMdl> is deprecated: use barc-msg:Z_KinBkMdl instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Z_KinBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:x-val is deprecated.  Use barc-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Z_KinBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:y-val is deprecated.  Use barc-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'psi-val :lambda-list '(m))
(cl:defmethod psi-val ((m <Z_KinBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:psi-val is deprecated.  Use barc-msg:psi instead.")
  (psi m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <Z_KinBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:v-val is deprecated.  Use barc-msg:v instead.")
  (v m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Z_KinBkMdl>) ostream)
  "Serializes a message object of type '<Z_KinBkMdl>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Z_KinBkMdl>) istream)
  "Deserializes a message object of type '<Z_KinBkMdl>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psi) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Z_KinBkMdl>)))
  "Returns string type for a message object of type '<Z_KinBkMdl>"
  "barc/Z_KinBkMdl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Z_KinBkMdl)))
  "Returns string type for a message object of type 'Z_KinBkMdl"
  "barc/Z_KinBkMdl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Z_KinBkMdl>)))
  "Returns md5sum for a message object of type '<Z_KinBkMdl>"
  "7c719f8bc9903c4ed5c145b22b3badcc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Z_KinBkMdl)))
  "Returns md5sum for a message object of type 'Z_KinBkMdl"
  "7c719f8bc9903c4ed5c145b22b3badcc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Z_KinBkMdl>)))
  "Returns full string definition for message of type '<Z_KinBkMdl>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 psi~%float32 v~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Z_KinBkMdl)))
  "Returns full string definition for message of type 'Z_KinBkMdl"
  (cl:format cl:nil "float32 x~%float32 y~%float32 psi~%float32 v~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Z_KinBkMdl>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Z_KinBkMdl>))
  "Converts a ROS message object to a list"
  (cl:list 'Z_KinBkMdl
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':psi (psi msg))
    (cl:cons ':v (v msg))
))
