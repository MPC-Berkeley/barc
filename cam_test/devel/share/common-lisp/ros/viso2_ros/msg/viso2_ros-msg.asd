
(cl:in-package :asdf)

(defsystem "viso2_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "VisoInfo" :depends-on ("_package_VisoInfo"))
    (:file "_package_VisoInfo" :depends-on ("_package"))
  ))