; Auto-generated. Do not edit!


(cl:in-package barc-msg)


;//! \htmlinclude ECU.msg.html

(cl:defclass <ECU> (roslisp-msg-protocol:ros-message)
  ((motor
    :reader motor
    :initarg :motor
    :type cl:float
    :initform 0.0)
   (servo
    :reader servo
    :initarg :servo
    :type cl:float
    :initform 0.0))
)

(cl:defclass ECU (<ECU>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ECU>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ECU)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name barc-msg:<ECU> is deprecated: use barc-msg:ECU instead.")))

(cl:ensure-generic-function 'motor-val :lambda-list '(m))
(cl:defmethod motor-val ((m <ECU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:motor-val is deprecated.  Use barc-msg:motor instead.")
  (motor m))

(cl:ensure-generic-function 'servo-val :lambda-list '(m))
(cl:defmethod servo-val ((m <ECU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:servo-val is deprecated.  Use barc-msg:servo instead.")
  (servo m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ECU>) ostream)
  "Serializes a message object of type '<ECU>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'servo))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ECU>) istream)
  "Deserializes a message object of type '<ECU>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'servo) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ECU>)))
  "Returns string type for a message object of type '<ECU>"
  "barc/ECU")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ECU)))
  "Returns string type for a message object of type 'ECU"
  "barc/ECU")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ECU>)))
  "Returns md5sum for a message object of type '<ECU>"
  "e60fd3690167c0df782fc50cceb5ce82")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ECU)))
  "Returns md5sum for a message object of type 'ECU"
  "e60fd3690167c0df782fc50cceb5ce82")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ECU>)))
  "Returns full string definition for message of type '<ECU>"
  (cl:format cl:nil "# This is a message to hold data for the ECU (electronic control unit)~%#~%# Units may vary depending on the topic~%# The motor controls the speeds of the vehicle through an input torque. (For input force, divide by radius of tire) ~%# The servo controls the steering angle~%#~%# For modeling and state estimation, motors units are [N], and servo units are [rad]~%# For actuator signals, both have units of PWM angle [deg]. This relates to the duty cycle~%float32 motor ~%float32 servo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ECU)))
  "Returns full string definition for message of type 'ECU"
  (cl:format cl:nil "# This is a message to hold data for the ECU (electronic control unit)~%#~%# Units may vary depending on the topic~%# The motor controls the speeds of the vehicle through an input torque. (For input force, divide by radius of tire) ~%# The servo controls the steering angle~%#~%# For modeling and state estimation, motors units are [N], and servo units are [rad]~%# For actuator signals, both have units of PWM angle [deg]. This relates to the duty cycle~%float32 motor ~%float32 servo~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ECU>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ECU>))
  "Converts a ROS message object to a list"
  (cl:list 'ECU
    (cl:cons ':motor (motor msg))
    (cl:cons ':servo (servo msg))
))
