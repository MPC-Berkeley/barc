
(cl:in-package :asdf)

(defsystem "marvelmind_nav-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "hedge_pos" :depends-on ("_package_hedge_pos"))
    (:file "_package_hedge_pos" :depends-on ("_package"))
  ))