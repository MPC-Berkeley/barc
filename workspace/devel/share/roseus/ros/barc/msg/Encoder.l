;; Auto-generated. Do not edit!


(when (boundp 'barc::Encoder)
  (if (not (find-package "BARC"))
    (make-package "BARC"))
  (shadow 'Encoder (find-package "BARC")))
(unless (find-package "BARC::ENCODER")
  (make-package "BARC::ENCODER"))

(in-package "ROS")
;;//! \htmlinclude Encoder.msg.html


(defclass barc::Encoder
  :super ros::object
  :slots (_FL _FR _BL _BR ))

(defmethod barc::Encoder
  (:init
   (&key
    ((:FL __FL) 0.0)
    ((:FR __FR) 0.0)
    ((:BL __BL) 0.0)
    ((:BR __BR) 0.0)
    )
   (send-super :init)
   (setq _FL (float __FL))
   (setq _FR (float __FR))
   (setq _BL (float __BL))
   (setq _BR (float __BR))
   self)
  (:FL
   (&optional __FL)
   (if __FL (setq _FL __FL)) _FL)
  (:FR
   (&optional __FR)
   (if __FR (setq _FR __FR)) _FR)
  (:BL
   (&optional __BL)
   (if __BL (setq _BL __BL)) _BL)
  (:BR
   (&optional __BR)
   (if __BR (setq _BR __BR)) _BR)
  (:serialization-length
   ()
   (+
    ;; float32 _FL
    4
    ;; float32 _FR
    4
    ;; float32 _BL
    4
    ;; float32 _BR
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _FL
       (sys::poke _FL (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _FR
       (sys::poke _FR (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _BL
       (sys::poke _BL (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _BR
       (sys::poke _BR (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _FL
     (setq _FL (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _FR
     (setq _FR (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _BL
     (setq _BL (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _BR
     (setq _BR (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get barc::Encoder :md5sum-) "32ae9320a3544d34b8e4e844525161ab")
(setf (get barc::Encoder :datatype-) "barc/Encoder")
(setf (get barc::Encoder :definition-)
      "float32 FL 
float32 FR
float32 BL
float32 BR

")



(provide :barc/Encoder "32ae9320a3544d34b8e4e844525161ab")


