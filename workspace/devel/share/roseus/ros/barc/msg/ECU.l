;; Auto-generated. Do not edit!


(when (boundp 'barc::ECU)
  (if (not (find-package "BARC"))
    (make-package "BARC"))
  (shadow 'ECU (find-package "BARC")))
(unless (find-package "BARC::ECU")
  (make-package "BARC::ECU"))

(in-package "ROS")
;;//! \htmlinclude ECU.msg.html


(defclass barc::ECU
  :super ros::object
  :slots (_motor _servo ))

(defmethod barc::ECU
  (:init
   (&key
    ((:motor __motor) 0.0)
    ((:servo __servo) 0.0)
    )
   (send-super :init)
   (setq _motor (float __motor))
   (setq _servo (float __servo))
   self)
  (:motor
   (&optional __motor)
   (if __motor (setq _motor __motor)) _motor)
  (:servo
   (&optional __servo)
   (if __servo (setq _servo __servo)) _servo)
  (:serialization-length
   ()
   (+
    ;; float32 _motor
    4
    ;; float32 _servo
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _motor
       (sys::poke _motor (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _servo
       (sys::poke _servo (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _motor
     (setq _motor (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _servo
     (setq _servo (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get barc::ECU :md5sum-) "e60fd3690167c0df782fc50cceb5ce82")
(setf (get barc::ECU :datatype-) "barc/ECU")
(setf (get barc::ECU :definition-)
      "# This is a message to hold data for the ECU (electronic control unit)
#
# Units may vary depending on the topic
# The motor controls the speeds of the vehicle through an input torque. (For input force, divide by radius of tire) 
# The servo controls the steering angle
#
# For modeling and state estimation, motors units are [N], and servo units are [rad]
# For actuator signals, both have units of PWM angle [deg]. This relates to the duty cycle
float32 motor 
float32 servo

")



(provide :barc/ECU "e60fd3690167c0df782fc50cceb5ce82")


