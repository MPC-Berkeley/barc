;; Auto-generated. Do not edit!


(when (boundp 'data_service::RegisterExperiment)
  (if (not (find-package "DATA_SERVICE"))
    (make-package "DATA_SERVICE"))
  (shadow 'RegisterExperiment (find-package "DATA_SERVICE")))
(unless (find-package "DATA_SERVICE::REGISTEREXPERIMENT")
  (make-package "DATA_SERVICE::REGISTEREXPERIMENT"))
(unless (find-package "DATA_SERVICE::REGISTEREXPERIMENTREQUEST")
  (make-package "DATA_SERVICE::REGISTEREXPERIMENTREQUEST"))
(unless (find-package "DATA_SERVICE::REGISTEREXPERIMENTRESPONSE")
  (make-package "DATA_SERVICE::REGISTEREXPERIMENTRESPONSE"))

(in-package "ROS")





(defclass data_service::RegisterExperimentRequest
  :super ros::object
  :slots (_experiment ))

(defmethod data_service::RegisterExperimentRequest
  (:init
   (&key
    ((:experiment __experiment) "")
    )
   (send-super :init)
   (setq _experiment (string __experiment))
   self)
  (:experiment
   (&optional __experiment)
   (if __experiment (setq _experiment __experiment)) _experiment)
  (:serialization-length
   ()
   (+
    ;; string _experiment
    4 (length _experiment)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _experiment
       (write-long (length _experiment) s) (princ _experiment s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _experiment
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _experiment (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass data_service::RegisterExperimentResponse
  :super ros::object
  :slots (_experiment_id ))

(defmethod data_service::RegisterExperimentResponse
  (:init
   (&key
    ((:experiment_id __experiment_id) 0)
    )
   (send-super :init)
   (setq _experiment_id (round __experiment_id))
   self)
  (:experiment_id
   (&optional __experiment_id)
   (if __experiment_id (setq _experiment_id __experiment_id)) _experiment_id)
  (:serialization-length
   ()
   (+
    ;; int32 _experiment_id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _experiment_id
       (write-long _experiment_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _experiment_id
     (setq _experiment_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass data_service::RegisterExperiment
  :super ros::object
  :slots ())

(setf (get data_service::RegisterExperiment :md5sum-) "23efeb8a860ffcd264ebd586a8c57078")
(setf (get data_service::RegisterExperiment :datatype-) "data_service/RegisterExperiment")
(setf (get data_service::RegisterExperiment :request) data_service::RegisterExperimentRequest)
(setf (get data_service::RegisterExperiment :response) data_service::RegisterExperimentResponse)

(defmethod data_service::RegisterExperimentRequest
  (:response () (instance data_service::RegisterExperimentResponse :init)))

(setf (get data_service::RegisterExperimentRequest :md5sum-) "23efeb8a860ffcd264ebd586a8c57078")
(setf (get data_service::RegisterExperimentRequest :datatype-) "data_service/RegisterExperimentRequest")
(setf (get data_service::RegisterExperimentRequest :definition-)
      "string experiment
---
int32 experiment_id
")

(setf (get data_service::RegisterExperimentResponse :md5sum-) "23efeb8a860ffcd264ebd586a8c57078")
(setf (get data_service::RegisterExperimentResponse :datatype-) "data_service/RegisterExperimentResponse")
(setf (get data_service::RegisterExperimentResponse :definition-)
      "string experiment
---
int32 experiment_id
")



(provide :data_service/RegisterExperiment "23efeb8a860ffcd264ebd586a8c57078")


