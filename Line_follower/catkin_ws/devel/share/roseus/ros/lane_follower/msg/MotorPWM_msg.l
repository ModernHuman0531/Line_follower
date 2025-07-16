;; Auto-generated. Do not edit!


(when (boundp 'lane_follower::MotorPWM_msg)
  (if (not (find-package "LANE_FOLLOWER"))
    (make-package "LANE_FOLLOWER"))
  (shadow 'MotorPWM_msg (find-package "LANE_FOLLOWER")))
(unless (find-package "LANE_FOLLOWER::MOTORPWM_MSG")
  (make-package "LANE_FOLLOWER::MOTORPWM_MSG"))

(in-package "ROS")
;;//! \htmlinclude MotorPWM_msg.msg.html


(defclass lane_follower::MotorPWM_msg
  :super ros::object
  :slots (_left_pwm _right_pwm ))

(defmethod lane_follower::MotorPWM_msg
  (:init
   (&key
    ((:left_pwm __left_pwm) 0)
    ((:right_pwm __right_pwm) 0)
    )
   (send-super :init)
   (setq _left_pwm (round __left_pwm))
   (setq _right_pwm (round __right_pwm))
   self)
  (:left_pwm
   (&optional __left_pwm)
   (if __left_pwm (setq _left_pwm __left_pwm)) _left_pwm)
  (:right_pwm
   (&optional __right_pwm)
   (if __right_pwm (setq _right_pwm __right_pwm)) _right_pwm)
  (:serialization-length
   ()
   (+
    ;; int16 _left_pwm
    2
    ;; int16 _right_pwm
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _left_pwm
       (write-word _left_pwm s)
     ;; int16 _right_pwm
       (write-word _right_pwm s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _left_pwm
     (setq _left_pwm (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _right_pwm
     (setq _right_pwm (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get lane_follower::MotorPWM_msg :md5sum-) "1bbcb2731ff8485f26f1b76809762437")
(setf (get lane_follower::MotorPWM_msg :datatype-) "lane_follower/MotorPWM_msg")
(setf (get lane_follower::MotorPWM_msg :definition-)
      "int16 left_pwm
int16 right_pwm
")



(provide :lane_follower/MotorPWM_msg "1bbcb2731ff8485f26f1b76809762437")


