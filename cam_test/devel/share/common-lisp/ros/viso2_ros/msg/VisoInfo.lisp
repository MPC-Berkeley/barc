; Auto-generated. Do not edit!


(cl:in-package viso2_ros-msg)


;//! \htmlinclude VisoInfo.msg.html

(cl:defclass <VisoInfo> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (got_lost
    :reader got_lost
    :initarg :got_lost
    :type cl:boolean
    :initform cl:nil)
   (change_reference_frame
    :reader change_reference_frame
    :initarg :change_reference_frame
    :type cl:boolean
    :initform cl:nil)
   (motion_estimate_valid
    :reader motion_estimate_valid
    :initarg :motion_estimate_valid
    :type cl:boolean
    :initform cl:nil)
   (num_matches
    :reader num_matches
    :initarg :num_matches
    :type cl:integer
    :initform 0)
   (num_inliers
    :reader num_inliers
    :initarg :num_inliers
    :type cl:integer
    :initform 0)
   (runtime
    :reader runtime
    :initarg :runtime
    :type cl:float
    :initform 0.0))
)

(cl:defclass VisoInfo (<VisoInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisoInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisoInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name viso2_ros-msg:<VisoInfo> is deprecated: use viso2_ros-msg:VisoInfo instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VisoInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader viso2_ros-msg:header-val is deprecated.  Use viso2_ros-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'got_lost-val :lambda-list '(m))
(cl:defmethod got_lost-val ((m <VisoInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader viso2_ros-msg:got_lost-val is deprecated.  Use viso2_ros-msg:got_lost instead.")
  (got_lost m))

(cl:ensure-generic-function 'change_reference_frame-val :lambda-list '(m))
(cl:defmethod change_reference_frame-val ((m <VisoInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader viso2_ros-msg:change_reference_frame-val is deprecated.  Use viso2_ros-msg:change_reference_frame instead.")
  (change_reference_frame m))

(cl:ensure-generic-function 'motion_estimate_valid-val :lambda-list '(m))
(cl:defmethod motion_estimate_valid-val ((m <VisoInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader viso2_ros-msg:motion_estimate_valid-val is deprecated.  Use viso2_ros-msg:motion_estimate_valid instead.")
  (motion_estimate_valid m))

(cl:ensure-generic-function 'num_matches-val :lambda-list '(m))
(cl:defmethod num_matches-val ((m <VisoInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader viso2_ros-msg:num_matches-val is deprecated.  Use viso2_ros-msg:num_matches instead.")
  (num_matches m))

(cl:ensure-generic-function 'num_inliers-val :lambda-list '(m))
(cl:defmethod num_inliers-val ((m <VisoInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader viso2_ros-msg:num_inliers-val is deprecated.  Use viso2_ros-msg:num_inliers instead.")
  (num_inliers m))

(cl:ensure-generic-function 'runtime-val :lambda-list '(m))
(cl:defmethod runtime-val ((m <VisoInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader viso2_ros-msg:runtime-val is deprecated.  Use viso2_ros-msg:runtime instead.")
  (runtime m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisoInfo>) ostream)
  "Serializes a message object of type '<VisoInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'got_lost) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'change_reference_frame) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'motion_estimate_valid) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_matches)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num_inliers)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'runtime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisoInfo>) istream)
  "Deserializes a message object of type '<VisoInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'got_lost) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'change_reference_frame) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'motion_estimate_valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_matches) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_inliers) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'runtime) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisoInfo>)))
  "Returns string type for a message object of type '<VisoInfo>"
  "viso2_ros/VisoInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisoInfo)))
  "Returns string type for a message object of type 'VisoInfo"
  "viso2_ros/VisoInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisoInfo>)))
  "Returns md5sum for a message object of type '<VisoInfo>"
  "765500d8b83bf74f7715c6e2e8e89092")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisoInfo)))
  "Returns md5sum for a message object of type 'VisoInfo"
  "765500d8b83bf74f7715c6e2e8e89092")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisoInfo>)))
  "Returns full string definition for message of type '<VisoInfo>"
  (cl:format cl:nil "# Internal information on the~%# viso2 algorithm parameters~%# and results~%~%Header header~%~%# True if the previous iteration of viso2~%# was not able to complete the matching process~%# therefore the visual odometer is re-started.~%bool got_lost~%~%# True if in the next run the reference ~%# frame will be changed. This is the case~%# when the number of inliers drops below~%# a threshold or the previous motion estimate~%# failed in last motion estimation.~%bool change_reference_frame~%~%# info from motion estimator~%bool motion_estimate_valid~%int32 num_matches~%int32 num_inliers~%~%# runtime of last iteration in seconds~%float64 runtime~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisoInfo)))
  "Returns full string definition for message of type 'VisoInfo"
  (cl:format cl:nil "# Internal information on the~%# viso2 algorithm parameters~%# and results~%~%Header header~%~%# True if the previous iteration of viso2~%# was not able to complete the matching process~%# therefore the visual odometer is re-started.~%bool got_lost~%~%# True if in the next run the reference ~%# frame will be changed. This is the case~%# when the number of inliers drops below~%# a threshold or the previous motion estimate~%# failed in last motion estimation.~%bool change_reference_frame~%~%# info from motion estimator~%bool motion_estimate_valid~%int32 num_matches~%int32 num_inliers~%~%# runtime of last iteration in seconds~%float64 runtime~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisoInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisoInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'VisoInfo
    (cl:cons ':header (header msg))
    (cl:cons ':got_lost (got_lost msg))
    (cl:cons ':change_reference_frame (change_reference_frame msg))
    (cl:cons ':motion_estimate_valid (motion_estimate_valid msg))
    (cl:cons ':num_matches (num_matches msg))
    (cl:cons ':num_inliers (num_inliers msg))
    (cl:cons ':runtime (runtime msg))
))
