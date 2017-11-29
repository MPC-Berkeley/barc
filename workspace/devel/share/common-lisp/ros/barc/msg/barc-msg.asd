
(cl:in-package :asdf)

(defsystem "barc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ECU" :depends-on ("_package_ECU"))
    (:file "_package_ECU" :depends-on ("_package"))
    (:file "Encoder" :depends-on ("_package_Encoder"))
    (:file "_package_Encoder" :depends-on ("_package"))
    (:file "Ultrasound" :depends-on ("_package_Ultrasound"))
    (:file "_package_Ultrasound" :depends-on ("_package"))
    (:file "Z_KinBkMdl" :depends-on ("_package_Z_KinBkMdl"))
    (:file "_package_Z_KinBkMdl" :depends-on ("_package"))
  ))