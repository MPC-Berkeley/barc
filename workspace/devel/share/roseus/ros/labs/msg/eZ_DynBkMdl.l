;; Auto-generated. Do not edit!


(when (boundp 'labs::eZ_DynBkMdl)
  (if (not (find-package "LABS"))
    (make-package "LABS"))
  (shadow 'eZ_DynBkMdl (find-package "LABS")))
(unless (find-package "LABS::EZ_DYNBKMDL")
  (make-package "LABS::EZ_DYNBKMDL"))

(in-package "ROS")
;;//! \htmlinclude eZ_DynBkMdl.msg.html


(defclass labs::eZ_DynBkMdl
  :super ros::object
  :slots (_s _ey _epsi _v_x _v_y _psi_dot ))

(defmethod labs::eZ_DynBkMdl
  (:init
   (&key
    ((:s __s) 0.0)
    ((:ey __ey) 0.0)
    ((:epsi __epsi) 0.0)
    ((:v_x __v_x) 0.0)
    ((:v_y __v_y) 0.0)
    ((:psi_dot __psi_dot) 0.0)
    )
   (send-super :init)
   (setq _s (float __s))
   (setq _ey (float __ey))
   (setq _epsi (float __epsi))
   (setq _v_x (float __v_x))
   (setq _v_y (float __v_y))
   (setq _psi_dot (float __psi_dot))
   self)
  (:s
   (&optional __s)
   (if __s (setq _s __s)) _s)
  (:ey
   (&optional __ey)
   (if __ey (setq _ey __ey)) _ey)
  (:epsi
   (&optional __epsi)
   (if __epsi (setq _epsi __epsi)) _epsi)
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
    ;; float32 _s
    4
    ;; float32 _ey
    4
    ;; float32 _epsi
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
     ;; float32 _s
       (sys::poke _s (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ey
       (sys::poke _ey (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _epsi
       (sys::poke _epsi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
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
   ;; float32 _s
     (setq _s (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ey
     (setq _ey (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _epsi
     (setq _epsi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _v_x
     (setq _v_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _v_y
     (setq _v_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psi_dot
     (setq _psi_dot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get labs::eZ_DynBkMdl :md5sum-) "19de479b90a68da73ee59a1fb6a50755")
(setf (get labs::eZ_DynBkMdl :datatype-) "labs/eZ_DynBkMdl")
(setf (get labs::eZ_DynBkMdl :definition-)
      "float32 s
float32 ey
float32 epsi
float32 v_x
float32 v_y
float32 psi_dot

")



(provide :labs/eZ_DynBkMdl "19de479b90a68da73ee59a1fb6a50755")


