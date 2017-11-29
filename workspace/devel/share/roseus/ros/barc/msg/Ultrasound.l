;; Auto-generated. Do not edit!


(when (boundp 'barc::Ultrasound)
  (if (not (find-package "BARC"))
    (make-package "BARC"))
  (shadow 'Ultrasound (find-package "BARC")))
(unless (find-package "BARC::ULTRASOUND")
  (make-package "BARC::ULTRASOUND"))

(in-package "ROS")
;;//! \htmlinclude Ultrasound.msg.html


(defclass barc::Ultrasound
  :super ros::object
  :slots (_front _back _right _left ))

(defmethod barc::Ultrasound
  (:init
   (&key
    ((:front __front) 0.0)
    ((:back __back) 0.0)
    ((:right __right) 0.0)
    ((:left __left) 0.0)
    )
   (send-super :init)
   (setq _front (float __front))
   (setq _back (float __back))
   (setq _right (float __right))
   (setq _left (float __left))
   self)
  (:front
   (&optional __front)
   (if __front (setq _front __front)) _front)
  (:back
   (&optional __back)
   (if __back (setq _back __back)) _back)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:serialization-length
   ()
   (+
    ;; float32 _front
    4
    ;; float32 _back
    4
    ;; float32 _right
    4
    ;; float32 _left
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _front
       (sys::poke _front (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _back
       (sys::poke _back (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right
       (sys::poke _right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left
       (sys::poke _left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _front
     (setq _front (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _back
     (setq _back (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right
     (setq _right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left
     (setq _left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get barc::Ultrasound :md5sum-) "a28144c3266e5b701faa1e4761aa50f9")
(setf (get barc::Ultrasound :datatype-) "barc/Ultrasound")
(setf (get barc::Ultrasound :definition-)
      "float32 front
float32 back
float32 right
float32 left

")



(provide :barc/Ultrasound "a28144c3266e5b701faa1e4761aa50f9")


