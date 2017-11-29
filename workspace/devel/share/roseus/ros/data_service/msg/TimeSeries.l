;; Auto-generated. Do not edit!


(when (boundp 'data_service::TimeSeries)
  (if (not (find-package "DATA_SERVICE"))
    (make-package "DATA_SERVICE"))
  (shadow 'TimeSeries (find-package "DATA_SERVICE")))
(unless (find-package "DATA_SERVICE::TIMESERIES")
  (make-package "DATA_SERVICE::TIMESERIES"))

(in-package "ROS")
;;//! \htmlinclude TimeSeries.msg.html


(defclass data_service::TimeSeries
  :super ros::object
  :slots (_series ))

(defmethod data_service::TimeSeries
  (:init
   (&key
    ((:series __series) (let (r) (dotimes (i 0) (push (instance data_service::TimeData :init) r)) r))
    )
   (send-super :init)
   (setq _series __series)
   self)
  (:series
   (&rest __series)
   (if (keywordp (car __series))
       (send* _series __series)
     (progn
       (if __series (setq _series (car __series)))
       _series)))
  (:serialization-length
   ()
   (+
    ;; data_service/TimeData[] _series
    (apply #'+ (send-all _series :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; data_service/TimeData[] _series
     (write-long (length _series) s)
     (dolist (elem _series)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; data_service/TimeData[] _series
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _series (let (r) (dotimes (i n) (push (instance data_service::TimeData :init) r)) r))
     (dolist (elem- _series)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get data_service::TimeSeries :md5sum-) "ecf556e37ef3615e6c95590390588636")
(setf (get data_service::TimeSeries :datatype-) "data_service/TimeSeries")
(setf (get data_service::TimeSeries :definition-)
      "TimeData[] series

================================================================================
MSG: data_service/TimeData
float64 timestamp
float64[] value

")



(provide :data_service/TimeSeries "ecf556e37ef3615e6c95590390588636")


