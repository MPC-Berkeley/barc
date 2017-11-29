;; Auto-generated. Do not edit!


(when (boundp 'data_service::DataForward)
  (if (not (find-package "DATA_SERVICE"))
    (make-package "DATA_SERVICE"))
  (shadow 'DataForward (find-package "DATA_SERVICE")))
(unless (find-package "DATA_SERVICE::DATAFORWARD")
  (make-package "DATA_SERVICE::DATAFORWARD"))
(unless (find-package "DATA_SERVICE::DATAFORWARDREQUEST")
  (make-package "DATA_SERVICE::DATAFORWARDREQUEST"))
(unless (find-package "DATA_SERVICE::DATAFORWARDRESPONSE")
  (make-package "DATA_SERVICE::DATAFORWARDRESPONSE"))

(in-package "ROS")





(defclass data_service::DataForwardRequest
  :super ros::object
  :slots (_time_signal _custom_signal _experiment_name ))

(defmethod data_service::DataForwardRequest
  (:init
   (&key
    ((:time_signal __time_signal) (instance data_service::TimeSignal :init))
    ((:custom_signal __custom_signal) (instance data_service::CustomSignal :init))
    ((:experiment_name __experiment_name) "")
    )
   (send-super :init)
   (setq _time_signal __time_signal)
   (setq _custom_signal __custom_signal)
   (setq _experiment_name (string __experiment_name))
   self)
  (:time_signal
   (&rest __time_signal)
   (if (keywordp (car __time_signal))
       (send* _time_signal __time_signal)
     (progn
       (if __time_signal (setq _time_signal (car __time_signal)))
       _time_signal)))
  (:custom_signal
   (&rest __custom_signal)
   (if (keywordp (car __custom_signal))
       (send* _custom_signal __custom_signal)
     (progn
       (if __custom_signal (setq _custom_signal (car __custom_signal)))
       _custom_signal)))
  (:experiment_name
   (&optional __experiment_name)
   (if __experiment_name (setq _experiment_name __experiment_name)) _experiment_name)
  (:serialization-length
   ()
   (+
    ;; data_service/TimeSignal _time_signal
    (send _time_signal :serialization-length)
    ;; data_service/CustomSignal _custom_signal
    (send _custom_signal :serialization-length)
    ;; string _experiment_name
    4 (length _experiment_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; data_service/TimeSignal _time_signal
       (send _time_signal :serialize s)
     ;; data_service/CustomSignal _custom_signal
       (send _custom_signal :serialize s)
     ;; string _experiment_name
       (write-long (length _experiment_name) s) (princ _experiment_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; data_service/TimeSignal _time_signal
     (send _time_signal :deserialize buf ptr-) (incf ptr- (send _time_signal :serialization-length))
   ;; data_service/CustomSignal _custom_signal
     (send _custom_signal :deserialize buf ptr-) (incf ptr- (send _custom_signal :serialization-length))
   ;; string _experiment_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _experiment_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass data_service::DataForwardResponse
  :super ros::object
  :slots (_response ))

(defmethod data_service::DataForwardResponse
  (:init
   (&key
    ((:response __response) "")
    )
   (send-super :init)
   (setq _response (string __response))
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:serialization-length
   ()
   (+
    ;; string _response
    4 (length _response)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass data_service::DataForward
  :super ros::object
  :slots ())

(setf (get data_service::DataForward :md5sum-) "f8d627aa29376505cccbe1058f0ed9d2")
(setf (get data_service::DataForward :datatype-) "data_service/DataForward")
(setf (get data_service::DataForward :request) data_service::DataForwardRequest)
(setf (get data_service::DataForward :response) data_service::DataForwardResponse)

(defmethod data_service::DataForwardRequest
  (:response () (instance data_service::DataForwardResponse :init)))

(setf (get data_service::DataForwardRequest :md5sum-) "f8d627aa29376505cccbe1058f0ed9d2")
(setf (get data_service::DataForwardRequest :datatype-) "data_service/DataForwardRequest")
(setf (get data_service::DataForwardRequest :definition-)
      "TimeSignal time_signal
CustomSignal custom_signal
string experiment_name

================================================================================
MSG: data_service/TimeSignal
string name
float64[] timestamps
string signal

================================================================================
MSG: data_service/CustomSignal
string id
---
string response

")

(setf (get data_service::DataForwardResponse :md5sum-) "f8d627aa29376505cccbe1058f0ed9d2")
(setf (get data_service::DataForwardResponse :datatype-) "data_service/DataForwardResponse")
(setf (get data_service::DataForwardResponse :definition-)
      "TimeSignal time_signal
CustomSignal custom_signal
string experiment_name

================================================================================
MSG: data_service/TimeSignal
string name
float64[] timestamps
string signal

================================================================================
MSG: data_service/CustomSignal
string id
---
string response

")



(provide :data_service/DataForward "f8d627aa29376505cccbe1058f0ed9d2")


