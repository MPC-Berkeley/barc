; Auto-generated. Do not edit!


(cl:in-package simulator-msg)


;//! \htmlinclude Z_DynBkMdl.msg.html

(cl:defclass <Z_DynBkMdl> (roslisp-msg-protocol:ros-message)
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
   (v_x
    :reader v_x
    :initarg :v_x
    :type cl:float
    :initform 0.0)
   (v_y
    :reader v_y
    :initarg :v_y
    :type cl:float
    :initform 0.0)
   (psi_dot
    :reader psi_dot
    :initarg :psi_dot
    :type cl:float
    :initform 0.0))
)

(cl:defclass Z_DynBkMdl (<Z_DynBkMdl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Z_DynBkMdl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Z_DynBkMdl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-msg:<Z_DynBkMdl> is deprecated: use simulator-msg:Z_DynBkMdl instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Z_DynBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:x-val is deprecated.  Use simulator-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Z_DynBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:y-val is deprecated.  Use simulator-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'psi-val :lambda-list '(m))
(cl:defmethod psi-val ((m <Z_DynBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:psi-val is deprecated.  Use simulator-msg:psi instead.")
  (psi m))

(cl:ensure-generic-function 'v_x-val :lambda-list '(m))
(cl:defmethod v_x-val ((m <Z_DynBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:v_x-val is deprecated.  Use simulator-msg:v_x instead.")
  (v_x m))

(cl:ensure-generic-function 'v_y-val :lambda-list '(m))
(cl:defmethod v_y-val ((m <Z_DynBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:v_y-val is deprecated.  Use simulator-msg:v_y instead.")
  (v_y m))

(cl:ensure-generic-function 'psi_dot-val :lambda-list '(m))
(cl:defmethod psi_dot-val ((m <Z_DynBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:psi_dot-val is deprecated.  Use simulator-msg:psi_dot instead.")
  (psi_dot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Z_DynBkMdl>) ostream)
  "Serializes a message object of type '<Z_DynBkMdl>"
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
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'v_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'psi_dot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Z_DynBkMdl>) istream)
  "Deserializes a message object of type '<Z_DynBkMdl>"
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
    (cl:setf (cl:slot-value msg 'v_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'v_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'psi_dot) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Z_DynBkMdl>)))
  "Returns string type for a message object of type '<Z_DynBkMdl>"
  "simulator/Z_DynBkMdl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Z_DynBkMdl)))
  "Returns string type for a message object of type 'Z_DynBkMdl"
  "simulator/Z_DynBkMdl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Z_DynBkMdl>)))
  "Returns md5sum for a message object of type '<Z_DynBkMdl>"
  "7fe12764dcdf3a7f6277d8c56c0c723c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Z_DynBkMdl)))
  "Returns md5sum for a message object of type 'Z_DynBkMdl"
  "7fe12764dcdf3a7f6277d8c56c0c723c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Z_DynBkMdl>)))
  "Returns full string definition for message of type '<Z_DynBkMdl>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 psi~%float32 v_x~%float32 v_y~%float32 psi_dot~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Z_DynBkMdl)))
  "Returns full string definition for message of type 'Z_DynBkMdl"
  (cl:format cl:nil "float32 x~%float32 y~%float32 psi~%float32 v_x~%float32 v_y~%float32 psi_dot~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Z_DynBkMdl>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Z_DynBkMdl>))
  "Converts a ROS message object to a list"
  (cl:list 'Z_DynBkMdl
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':psi (psi msg))
    (cl:cons ':v_x (v_x msg))
    (cl:cons ':v_y (v_y msg))
    (cl:cons ':psi_dot (psi_dot msg))
))
