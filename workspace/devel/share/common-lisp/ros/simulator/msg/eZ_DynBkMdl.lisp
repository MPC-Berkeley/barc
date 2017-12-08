; Auto-generated. Do not edit!


(cl:in-package simulator-msg)


;//! \htmlinclude eZ_DynBkMdl.msg.html

(cl:defclass <eZ_DynBkMdl> (roslisp-msg-protocol:ros-message)
  ((s
    :reader s
    :initarg :s
    :type cl:float
    :initform 0.0)
   (ey
    :reader ey
    :initarg :ey
    :type cl:float
    :initform 0.0)
   (epsi
    :reader epsi
    :initarg :epsi
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

(cl:defclass eZ_DynBkMdl (<eZ_DynBkMdl>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <eZ_DynBkMdl>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'eZ_DynBkMdl)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simulator-msg:<eZ_DynBkMdl> is deprecated: use simulator-msg:eZ_DynBkMdl instead.")))

(cl:ensure-generic-function 's-val :lambda-list '(m))
(cl:defmethod s-val ((m <eZ_DynBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:s-val is deprecated.  Use simulator-msg:s instead.")
  (s m))

(cl:ensure-generic-function 'ey-val :lambda-list '(m))
(cl:defmethod ey-val ((m <eZ_DynBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:ey-val is deprecated.  Use simulator-msg:ey instead.")
  (ey m))

(cl:ensure-generic-function 'epsi-val :lambda-list '(m))
(cl:defmethod epsi-val ((m <eZ_DynBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:epsi-val is deprecated.  Use simulator-msg:epsi instead.")
  (epsi m))

(cl:ensure-generic-function 'v_x-val :lambda-list '(m))
(cl:defmethod v_x-val ((m <eZ_DynBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:v_x-val is deprecated.  Use simulator-msg:v_x instead.")
  (v_x m))

(cl:ensure-generic-function 'v_y-val :lambda-list '(m))
(cl:defmethod v_y-val ((m <eZ_DynBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:v_y-val is deprecated.  Use simulator-msg:v_y instead.")
  (v_y m))

(cl:ensure-generic-function 'psi_dot-val :lambda-list '(m))
(cl:defmethod psi_dot-val ((m <eZ_DynBkMdl>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simulator-msg:psi_dot-val is deprecated.  Use simulator-msg:psi_dot instead.")
  (psi_dot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <eZ_DynBkMdl>) ostream)
  "Serializes a message object of type '<eZ_DynBkMdl>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 's))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ey))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'epsi))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <eZ_DynBkMdl>) istream)
  "Deserializes a message object of type '<eZ_DynBkMdl>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 's) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ey) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'epsi) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<eZ_DynBkMdl>)))
  "Returns string type for a message object of type '<eZ_DynBkMdl>"
  "simulator/eZ_DynBkMdl")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'eZ_DynBkMdl)))
  "Returns string type for a message object of type 'eZ_DynBkMdl"
  "simulator/eZ_DynBkMdl")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<eZ_DynBkMdl>)))
  "Returns md5sum for a message object of type '<eZ_DynBkMdl>"
  "19de479b90a68da73ee59a1fb6a50755")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'eZ_DynBkMdl)))
  "Returns md5sum for a message object of type 'eZ_DynBkMdl"
  "19de479b90a68da73ee59a1fb6a50755")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<eZ_DynBkMdl>)))
  "Returns full string definition for message of type '<eZ_DynBkMdl>"
  (cl:format cl:nil "float32 s~%float32 ey~%float32 epsi~%float32 v_x~%float32 v_y~%float32 psi_dot~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'eZ_DynBkMdl)))
  "Returns full string definition for message of type 'eZ_DynBkMdl"
  (cl:format cl:nil "float32 s~%float32 ey~%float32 epsi~%float32 v_x~%float32 v_y~%float32 psi_dot~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <eZ_DynBkMdl>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <eZ_DynBkMdl>))
  "Converts a ROS message object to a list"
  (cl:list 'eZ_DynBkMdl
    (cl:cons ':s (s msg))
    (cl:cons ':ey (ey msg))
    (cl:cons ':epsi (epsi msg))
    (cl:cons ':v_x (v_x msg))
    (cl:cons ':v_y (v_y msg))
    (cl:cons ':psi_dot (psi_dot msg))
))
