;; Auto-generated. Do not edit!


(when (boundp 'data_service::CustomSignal)
  (if (not (find-package "DATA_SERVICE"))
    (make-package "DATA_SERVICE"))
  (shadow 'CustomSignal (find-package "DATA_SERVICE")))
(unless (find-package "DATA_SERVICE::CUSTOMSIGNAL")
  (make-package "DATA_SERVICE::CUSTOMSIGNAL"))

(in-package "ROS")
;;//! \htmlinclude CustomSignal.msg.html


(defclass data_service::CustomSignal
  :super ros::object
  :slots (_id _signal ))

(defmethod data_service::CustomSignal
  (:init
   (&key
    ((:id __id) "")
    ((:signal __signal) "")
    )
   (send-super :init)
   (setq _id (string __id))
   (setq _signal (string __signal))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:signal
   (&optional __signal)
   (if __signal (setq _signal __signal)) _signal)
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ;; string _signal
    4 (length _signal)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; string _signal
       (write-long (length _signal) s) (princ _signal s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _signal
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _signal (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get data_service::CustomSignal :md5sum-) "952635fef7c9f8f6266bdfc127ea52f9")
(setf (get data_service::CustomSignal :datatype-) "data_service/CustomSignal")
(setf (get data_service::CustomSignal :definition-)
      "string id
string signal
")



(provide :data_service/CustomSignal "952635fef7c9f8f6266bdfc127ea52f9")


