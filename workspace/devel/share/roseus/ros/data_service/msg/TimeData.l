;; Auto-generated. Do not edit!


(when (boundp 'data_service::TimeData)
  (if (not (find-package "DATA_SERVICE"))
    (make-package "DATA_SERVICE"))
  (shadow 'TimeData (find-package "DATA_SERVICE")))
(unless (find-package "DATA_SERVICE::TIMEDATA")
  (make-package "DATA_SERVICE::TIMEDATA"))

(in-package "ROS")
;;//! \htmlinclude TimeData.msg.html


(defclass data_service::TimeData
  :super ros::object
  :slots (_timestamp _value ))

(defmethod data_service::TimeData
  (:init
   (&key
    ((:timestamp __timestamp) 0.0)
    ((:value __value) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _timestamp (float __timestamp))
   (setq _value __value)
   self)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; float64 _timestamp
    8
    ;; float64[] _value
    (* 8    (length _value)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _value
     (write-long (length _value) s)
     (dotimes (i (length _value))
       (sys::poke (elt _value i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _value
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _value (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _value i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get data_service::TimeData :md5sum-) "806316970232f83725326f09cc93bc8a")
(setf (get data_service::TimeData :datatype-) "data_service/TimeData")
(setf (get data_service::TimeData :definition-)
      "float64 timestamp
float64[] value

")



(provide :data_service/TimeData "806316970232f83725326f09cc93bc8a")


