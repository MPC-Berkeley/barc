;; Auto-generated. Do not edit!


(when (boundp 'data_service::DataRetrieve)
  (if (not (find-package "DATA_SERVICE"))
    (make-package "DATA_SERVICE"))
  (shadow 'DataRetrieve (find-package "DATA_SERVICE")))
(unless (find-package "DATA_SERVICE::DATARETRIEVE")
  (make-package "DATA_SERVICE::DATARETRIEVE"))
(unless (find-package "DATA_SERVICE::DATARETRIEVEREQUEST")
  (make-package "DATA_SERVICE::DATARETRIEVEREQUEST"))
(unless (find-package "DATA_SERVICE::DATARETRIEVERESPONSE")
  (make-package "DATA_SERVICE::DATARETRIEVERESPONSE"))

(in-package "ROS")





(defclass data_service::DataRetrieveRequest
  :super ros::object
  :slots (_id _is_time ))

(defmethod data_service::DataRetrieveRequest
  (:init
   (&key
    ((:id __id) "")
    ((:is_time __is_time) nil)
    )
   (send-super :init)
   (setq _id (string __id))
   (setq _is_time __is_time)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:is_time
   (&optional __is_time)
   (if __is_time (setq _is_time __is_time)) _is_time)
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ;; bool _is_time
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; bool _is_time
       (if _is_time (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _is_time
     (setq _is_time (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass data_service::DataRetrieveResponse
  :super ros::object
  :slots (_signal ))

(defmethod data_service::DataRetrieveResponse
  (:init
   (&key
    ((:signal __signal) (instance data_service::CustomSignal :init))
    )
   (send-super :init)
   (setq _signal __signal)
   self)
  (:signal
   (&rest __signal)
   (if (keywordp (car __signal))
       (send* _signal __signal)
     (progn
       (if __signal (setq _signal (car __signal)))
       _signal)))
  (:serialization-length
   ()
   (+
    ;; data_service/CustomSignal _signal
    (send _signal :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; data_service/CustomSignal _signal
       (send _signal :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; data_service/CustomSignal _signal
     (send _signal :deserialize buf ptr-) (incf ptr- (send _signal :serialization-length))
   ;;
   self)
  )

(defclass data_service::DataRetrieve
  :super ros::object
  :slots ())

(setf (get data_service::DataRetrieve :md5sum-) "f58abbeb8edc41005a341386540c1d0b")
(setf (get data_service::DataRetrieve :datatype-) "data_service/DataRetrieve")
(setf (get data_service::DataRetrieve :request) data_service::DataRetrieveRequest)
(setf (get data_service::DataRetrieve :response) data_service::DataRetrieveResponse)

(defmethod data_service::DataRetrieveRequest
  (:response () (instance data_service::DataRetrieveResponse :init)))

(setf (get data_service::DataRetrieveRequest :md5sum-) "f58abbeb8edc41005a341386540c1d0b")
(setf (get data_service::DataRetrieveRequest :datatype-) "data_service/DataRetrieveRequest")
(setf (get data_service::DataRetrieveRequest :definition-)
      "string id
bool is_time
---
CustomSignal signal

================================================================================
MSG: data_service/CustomSignal
string id
")

(setf (get data_service::DataRetrieveResponse :md5sum-) "f58abbeb8edc41005a341386540c1d0b")
(setf (get data_service::DataRetrieveResponse :datatype-) "data_service/DataRetrieveResponse")
(setf (get data_service::DataRetrieveResponse :definition-)
      "string id
bool is_time
---
CustomSignal signal

================================================================================
MSG: data_service/CustomSignal
string id
")



(provide :data_service/DataRetrieve "f58abbeb8edc41005a341386540c1d0b")


