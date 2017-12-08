;; Auto-generated. Do not edit!


(when (boundp 'simulator::Z_DynBkMdl)
  (if (not (find-package "SIMULATOR"))
    (make-package "SIMULATOR"))
  (shadow 'Z_DynBkMdl (find-package "SIMULATOR")))
(unless (find-package "SIMULATOR::Z_DYNBKMDL")
  (make-package "SIMULATOR::Z_DYNBKMDL"))

(in-package "ROS")
;;//! \htmlinclude Z_DynBkMdl.msg.html


(defclass simulator::Z_DynBkMdl
  :super ros::object
  :slots (_x _y _psi _v_x _v_y _psi_dot ))

(defmethod simulator::Z_DynBkMdl
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:psi __psi) 0.0)
    ((:v_x __v_x) 0.0)
    ((:v_y __v_y) 0.0)
    ((:psi_dot __psi_dot) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _psi (float __psi))
   (setq _v_x (float __v_x))
   (setq _v_y (float __v_y))
   (setq _psi_dot (float __psi_dot))
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
  (:v_x
   (&optional __v_x)
   (if __v_x (setq _v_x __v_x)) _v_x)
  (:v_y
   (&optional __v_y)
   (if __v_y (setq _v_y __v_y)) _v_y)
  (:psi_dot
   (&optional __psi_dot)
   (if __psi_dot (setq _psi_dot __psi_dot)) _psi_dot)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _psi
    4
    ;; float32 _v_x
    4
    ;; float32 _v_y
    4
    ;; float32 _psi_dot
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
     ;; float32 _v_x
       (sys::poke _v_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _v_y
       (sys::poke _v_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psi_dot
       (sys::poke _psi_dot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _v_x
     (setq _v_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _v_y
     (setq _v_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psi_dot
     (setq _psi_dot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get simulator::Z_DynBkMdl :md5sum-) "7fe12764dcdf3a7f6277d8c56c0c723c")
(setf (get simulator::Z_DynBkMdl :datatype-) "simulator/Z_DynBkMdl")
(setf (get simulator::Z_DynBkMdl :definition-)
      "float32 x
float32 y
float32 psi
float32 v_x
float32 v_y
float32 psi_dot

")



(provide :simulator/Z_DynBkMdl "7fe12764dcdf3a7f6277d8c56c0c723c")


