;; Auto-generated. Do not edit!


(when (boundp 'data_service::TimeSignal)
  (if (not (find-package "DATA_SERVICE"))
    (make-package "DATA_SERVICE"))
  (shadow 'TimeSignal (find-package "DATA_SERVICE")))
(unless (find-package "DATA_SERVICE::TIMESIGNAL")
  (make-package "DATA_SERVICE::TIMESIGNAL"))

(in-package "ROS")
;;//! \htmlinclude TimeSignal.msg.html


(defclass data_service::TimeSignal
  :super ros::object
  :slots (_name _timestamps _signal ))

(defmethod data_service::TimeSignal
  (:init
   (&key
    ((:name __name) "")
    ((:timestamps __timestamps) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:signal __signal) "")
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _timestamps __timestamps)
   (setq _signal (string __signal))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:timestamps
   (&optional __timestamps)
   (if __timestamps (setq _timestamps __timestamps)) _timestamps)
  (:signal
   (&optional __signal)
   (if __signal (setq _signal __signal)) _signal)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float64[] _timestamps
    (* 8    (length _timestamps)) 4
    ;; string _signal
    4 (length _signal)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float64[] _timestamps
     (write-long (length _timestamps) s)
     (dotimes (i (length _timestamps))
       (sys::poke (elt _timestamps i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; string _signal
       (write-long (length _signal) s) (princ _signal s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64[] _timestamps
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _timestamps (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _timestamps i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; string _signal
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _signal (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get data_service::TimeSignal :md5sum-) "242915a951390ccd66bdffda0979a29d")
(setf (get data_service::TimeSignal :datatype-) "data_service/TimeSignal")
(setf (get data_service::TimeSignal :definition-)
      "string name
float64[] timestamps
string signal

")



(provide :data_service/TimeSignal "242915a951390ccd66bdffda0979a29d")


