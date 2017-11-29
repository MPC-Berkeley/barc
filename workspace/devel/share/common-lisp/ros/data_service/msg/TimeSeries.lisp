; Auto-generated. Do not edit!


(cl:in-package data_service-msg)


;//! \htmlinclude TimeSeries.msg.html

(cl:defclass <TimeSeries> (roslisp-msg-protocol:ros-message)
  ((series
    :reader series
    :initarg :series
    :type (cl:vector data_service-msg:TimeData)
   :initform (cl:make-array 0 :element-type 'data_service-msg:TimeData :initial-element (cl:make-instance 'data_service-msg:TimeData))))
)

(cl:defclass TimeSeries (<TimeSeries>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TimeSeries>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TimeSeries)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_service-msg:<TimeSeries> is deprecated: use data_service-msg:TimeSeries instead.")))

(cl:ensure-generic-function 'series-val :lambda-list '(m))
(cl:defmethod series-val ((m <TimeSeries>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_service-msg:series-val is deprecated.  Use data_service-msg:series instead.")
  (series m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TimeSeries>) ostream)
  "Serializes a message object of type '<TimeSeries>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'series))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'series))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TimeSeries>) istream)
  "Deserializes a message object of type '<TimeSeries>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'series) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'series)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'data_service-msg:TimeData))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TimeSeries>)))
  "Returns string type for a message object of type '<TimeSeries>"
  "data_service/TimeSeries")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TimeSeries)))
  "Returns string type for a message object of type 'TimeSeries"
  "data_service/TimeSeries")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TimeSeries>)))
  "Returns md5sum for a message object of type '<TimeSeries>"
  "ecf556e37ef3615e6c95590390588636")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TimeSeries)))
  "Returns md5sum for a message object of type 'TimeSeries"
  "ecf556e37ef3615e6c95590390588636")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TimeSeries>)))
  "Returns full string definition for message of type '<TimeSeries>"
  (cl:format cl:nil "TimeData[] series~%~%================================================================================~%MSG: data_service/TimeData~%float64 timestamp~%float64[] value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TimeSeries)))
  "Returns full string definition for message of type 'TimeSeries"
  (cl:format cl:nil "TimeData[] series~%~%================================================================================~%MSG: data_service/TimeData~%float64 timestamp~%float64[] value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TimeSeries>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'series) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TimeSeries>))
  "Converts a ROS message object to a list"
  (cl:list 'TimeSeries
    (cl:cons ':series (series msg))
))
