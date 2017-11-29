;; Auto-generated. Do not edit!


(when (boundp 'data_service::RegisterSetting)
  (if (not (find-package "DATA_SERVICE"))
    (make-package "DATA_SERVICE"))
  (shadow 'RegisterSetting (find-package "DATA_SERVICE")))
(unless (find-package "DATA_SERVICE::REGISTERSETTING")
  (make-package "DATA_SERVICE::REGISTERSETTING"))
(unless (find-package "DATA_SERVICE::REGISTERSETTINGREQUEST")
  (make-package "DATA_SERVICE::REGISTERSETTINGREQUEST"))
(unless (find-package "DATA_SERVICE::REGISTERSETTINGRESPONSE")
  (make-package "DATA_SERVICE::REGISTERSETTINGRESPONSE"))

(in-package "ROS")





(defclass data_service::RegisterSettingRequest
  :super ros::object
  :slots (_key _value ))

(defmethod data_service::RegisterSettingRequest
  (:init
   (&key
    ((:key __key) "")
    ((:value __value) "")
    )
   (send-super :init)
   (setq _key (string __key))
   (setq _value (string __value))
   self)
  (:key
   (&optional __key)
   (if __key (setq _key __key)) _key)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _key
    4 (length _key)
    ;; string _value
    4 (length _value)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _key
       (write-long (length _key) s) (princ _key s)
     ;; string _value
       (write-long (length _value) s) (princ _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _key
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _key (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _value
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _value (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass data_service::RegisterSettingResponse
  :super ros::object
  :slots (_response ))

(defmethod data_service::RegisterSettingResponse
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

(defclass data_service::RegisterSetting
  :super ros::object
  :slots ())

(setf (get data_service::RegisterSetting :md5sum-) "c268a698e75a634c50aaf2d35ec4caf2")
(setf (get data_service::RegisterSetting :datatype-) "data_service/RegisterSetting")
(setf (get data_service::RegisterSetting :request) data_service::RegisterSettingRequest)
(setf (get data_service::RegisterSetting :response) data_service::RegisterSettingResponse)

(defmethod data_service::RegisterSettingRequest
  (:response () (instance data_service::RegisterSettingResponse :init)))

(setf (get data_service::RegisterSettingRequest :md5sum-) "c268a698e75a634c50aaf2d35ec4caf2")
(setf (get data_service::RegisterSettingRequest :datatype-) "data_service/RegisterSettingRequest")
(setf (get data_service::RegisterSettingRequest :definition-)
      "string key
string value
---
string response
")

(setf (get data_service::RegisterSettingResponse :md5sum-) "c268a698e75a634c50aaf2d35ec4caf2")
(setf (get data_service::RegisterSettingResponse :datatype-) "data_service/RegisterSettingResponse")
(setf (get data_service::RegisterSettingResponse :definition-)
      "string key
string value
---
string response
")



(provide :data_service/RegisterSetting "c268a698e75a634c50aaf2d35ec4caf2")


