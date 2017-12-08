
(cl:in-package :asdf)

(defsystem "simulator-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Z_DynBkMdl" :depends-on ("_package_Z_DynBkMdl"))
    (:file "_package_Z_DynBkMdl" :depends-on ("_package"))
    (:file "eZ_DynBkMdl" :depends-on ("_package_eZ_DynBkMdl"))
    (:file "_package_eZ_DynBkMdl" :depends-on ("_package"))
  ))