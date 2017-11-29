;; Auto-generated. Do not edit!


(when (boundp 'data_service::RegisterVideo)
  (if (not (find-package "DATA_SERVICE"))
    (make-package "DATA_SERVICE"))
  (shadow 'RegisterVideo (find-package "DATA_SERVICE")))
(unless (find-package "DATA_SERVICE::REGISTERVIDEO")
  (make-package "DATA_SERVICE::REGISTERVIDEO"))
(unless (find-package "DATA_SERVICE::REGISTERVIDEOREQUEST")
  (make-package "DATA_SERVICE::REGISTERVIDEOREQUEST"))
(unless (find-package "DATA_SERVICE::REGISTERVIDEORESPONSE")
  (make-package "DATA_SERVICE::REGISTERVIDEORESPONSE"))

(in-package "ROS")





(defclass data_service::RegisterVideoRequest
  :super ros::object
  :slots (_experiment _path ))

(defmethod data_service::RegisterVideoRequest
  (:init
   (&key
    ((:experiment __experiment) "")
    ((:path __path) "")
    )
   (send-super :init)
   (setq _experiment (string __experiment))
   (setq _path (string __path))
   self)
  (:experiment
   (&optional __experiment)
   (if __experiment (setq _experiment __experiment)) _experiment)
  (:path
   (&optional __path)
   (if __path (setq _path __path)) _path)
  (:serialization-length
   ()
   (+
    ;; string _experiment
    4 (length _experiment)
    ;; string _path
    4 (length _path)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _experiment
       (write-long (length _experiment) s) (princ _experiment s)
     ;; string _path
       (write-long (length _path) s) (princ _path s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _experiment
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _experiment (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _path
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _path (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass data_service::RegisterVideoResponse
  :super ros::object
  :slots (_response ))

(defmethod data_service::RegisterVideoResponse
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

(defclass data_service::RegisterVideo
  :super ros::object
  :slots ())

(setf (get data_service::RegisterVideo :md5sum-) "252bc0f1ff2ebd98b97d212fe2c3fb57")
(setf (get data_service::RegisterVideo :datatype-) "data_service/RegisterVideo")
(setf (get data_service::RegisterVideo :request) data_service::RegisterVideoRequest)
(setf (get data_service::RegisterVideo :response) data_service::RegisterVideoResponse)

(defmethod data_service::RegisterVideoRequest
  (:response () (instance data_service::RegisterVideoResponse :init)))

(setf (get data_service::RegisterVideoRequest :md5sum-) "252bc0f1ff2ebd98b97d212fe2c3fb57")
(setf (get data_service::RegisterVideoRequest :datatype-) "data_service/RegisterVideoRequest")
(setf (get data_service::RegisterVideoRequest :definition-)
      "string experiment
string path
---
string response
")

(setf (get data_service::RegisterVideoResponse :md5sum-) "252bc0f1ff2ebd98b97d212fe2c3fb57")
(setf (get data_service::RegisterVideoResponse :datatype-) "data_service/RegisterVideoResponse")
(setf (get data_service::RegisterVideoResponse :definition-)
      "string experiment
string path
---
string response
")



(provide :data_service/RegisterVideo "252bc0f1ff2ebd98b97d212fe2c3fb57")


