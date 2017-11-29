
(cl:in-package :asdf)

(defsystem "data_service-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CustomSignal" :depends-on ("_package_CustomSignal"))
    (:file "_package_CustomSignal" :depends-on ("_package"))
    (:file "TimeData" :depends-on ("_package_TimeData"))
    (:file "_package_TimeData" :depends-on ("_package"))
    (:file "TimeSeries" :depends-on ("_package_TimeSeries"))
    (:file "_package_TimeSeries" :depends-on ("_package"))
    (:file "TimeSignal" :depends-on ("_package_TimeSignal"))
    (:file "_package_TimeSignal" :depends-on ("_package"))
  ))