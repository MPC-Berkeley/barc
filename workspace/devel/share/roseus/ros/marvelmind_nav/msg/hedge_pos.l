;; Auto-generated. Do not edit!


(when (boundp 'marvelmind_nav::hedge_pos)
  (if (not (find-package "MARVELMIND_NAV"))
    (make-package "MARVELMIND_NAV"))
  (shadow 'hedge_pos (find-package "MARVELMIND_NAV")))
(unless (find-package "MARVELMIND_NAV::HEDGE_POS")
  (make-package "MARVELMIND_NAV::HEDGE_POS"))

(in-package "ROS")
;;//! \htmlinclude hedge_pos.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass marvelmind_nav::hedge_pos
  :super ros::object
  :slots (_header _timestamp_ms _x_m _y_m _z_m _flags ))

(defmethod marvelmind_nav::hedge_pos
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:timestamp_ms __timestamp_ms) 0)
    ((:x_m __x_m) 0.0)
    ((:y_m __y_m) 0.0)
    ((:z_m __z_m) 0.0)
    ((:flags __flags) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _timestamp_ms (round __timestamp_ms))
   (setq _x_m (float __x_m))
   (setq _y_m (float __y_m))
   (setq _z_m (float __z_m))
   (setq _flags (round __flags))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:timestamp_ms
   (&optional __timestamp_ms)
   (if __timestamp_ms (setq _timestamp_ms __timestamp_ms)) _timestamp_ms)
  (:x_m
   (&optional __x_m)
   (if __x_m (setq _x_m __x_m)) _x_m)
  (:y_m
   (&optional __y_m)
   (if __y_m (setq _y_m __y_m)) _y_m)
  (:z_m
   (&optional __z_m)
   (if __z_m (setq _z_m __z_m)) _z_m)
  (:flags
   (&optional __flags)
   (if __flags (setq _flags __flags)) _flags)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int64 _timestamp_ms
    8
    ;; float64 _x_m
    8
    ;; float64 _y_m
    8
    ;; float64 _z_m
    8
    ;; uint8 _flags
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int64 _timestamp_ms
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _timestamp_ms (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _timestamp_ms) (= (length (_timestamp_ms . bv)) 2)) ;; bignum
              (write-long (ash (elt (_timestamp_ms . bv) 0) 0) s)
              (write-long (ash (elt (_timestamp_ms . bv) 1) -1) s))
             ((and (class _timestamp_ms) (= (length (_timestamp_ms . bv)) 1)) ;; big1
              (write-long (elt (_timestamp_ms . bv) 0) s)
              (write-long (if (>= _timestamp_ms 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _timestamp_ms s)(write-long (if (>= _timestamp_ms 0) 0 #xffffffff) s)))
     ;; float64 _x_m
       (sys::poke _x_m (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_m
       (sys::poke _y_m (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_m
       (sys::poke _z_m (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _flags
       (write-byte _flags s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int64 _timestamp_ms
#+(or :alpha :irix6 :x86_64)
      (setf _timestamp_ms (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _timestamp_ms (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _x_m
     (setq _x_m (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_m
     (setq _y_m (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_m
     (setq _z_m (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _flags
     (setq _flags (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get marvelmind_nav::hedge_pos :md5sum-) "29d7a738a044fe2e89bd305ae8fa5f2b")
(setf (get marvelmind_nav::hedge_pos :datatype-) "marvelmind_nav/hedge_pos")
(setf (get marvelmind_nav::hedge_pos :definition-)
      "Header header
int64 timestamp_ms
float64 x_m
float64 y_m
float64 z_m
uint8 flags

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :marvelmind_nav/hedge_pos "29d7a738a044fe2e89bd305ae8fa5f2b")


