;; Auto-generated. Do not edit!


(when (boundp 'barc::Z_KinBkMdl)
  (if (not (find-package "BARC"))
    (make-package "BARC"))
  (shadow 'Z_KinBkMdl (find-package "BARC")))
(unless (find-package "BARC::Z_KINBKMDL")
  (make-package "BARC::Z_KINBKMDL"))

(in-package "ROS")
;;//! \htmlinclude Z_KinBkMdl.msg.html


(defclass barc::Z_KinBkMdl
  :super ros::object
  :slots (_x _y _psi _v ))

(defmethod barc::Z_KinBkMdl
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:psi __psi) 0.0)
    ((:v __v) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _psi (float __psi))
   (setq _v (float __v))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _psi
    4
    ;; float32 _v
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psi
       (sys::poke _psi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _v
       (sys::poke _v (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psi
     (setq _psi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _v
     (setq _v (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get barc::Z_KinBkMdl :md5sum-) "7c719f8bc9903c4ed5c145b22b3badcc")
(setf (get barc::Z_KinBkMdl :datatype-) "barc/Z_KinBkMdl")
(setf (get barc::Z_KinBkMdl :definition-)
      "float32 x
float32 y
float32 psi
float32 v

")



(provide :barc/Z_KinBkMdl "7c719f8bc9903c4ed5c145b22b3badcc")


