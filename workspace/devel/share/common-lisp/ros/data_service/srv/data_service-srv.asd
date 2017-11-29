
(cl:in-package :asdf)

(defsystem "data_service-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :data_service-msg
)
  :components ((:file "_package")
    (:file "DataForward" :depends-on ("_package_DataForward"))
    (:file "_package_DataForward" :depends-on ("_package"))
    (:file "DataRetrieve" :depends-on ("_package_DataRetrieve"))
    (:file "_package_DataRetrieve" :depends-on ("_package"))
    (:file "RegisterExperiment" :depends-on ("_package_RegisterExperiment"))
    (:file "_package_RegisterExperiment" :depends-on ("_package"))
    (:file "RegisterSetting" :depends-on ("_package_RegisterSetting"))
    (:file "_package_RegisterSetting" :depends-on ("_package"))
    (:file "RegisterVideo" :depends-on ("_package_RegisterVideo"))
    (:file "_package_RegisterVideo" :depends-on ("_package"))
  ))