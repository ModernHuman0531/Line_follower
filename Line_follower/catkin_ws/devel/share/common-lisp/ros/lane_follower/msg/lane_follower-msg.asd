
(cl:in-package :asdf)

(defsystem "lane_follower-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MotorPWM_msg" :depends-on ("_package_MotorPWM_msg"))
    (:file "_package_MotorPWM_msg" :depends-on ("_package"))
  ))