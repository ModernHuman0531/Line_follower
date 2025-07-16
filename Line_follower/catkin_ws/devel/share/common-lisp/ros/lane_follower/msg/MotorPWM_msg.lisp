; Auto-generated. Do not edit!


(cl:in-package lane_follower-msg)


;//! \htmlinclude MotorPWM_msg.msg.html

(cl:defclass <MotorPWM_msg> (roslisp-msg-protocol:ros-message)
  ((left_pwm
    :reader left_pwm
    :initarg :left_pwm
    :type cl:fixnum
    :initform 0)
   (right_pwm
    :reader right_pwm
    :initarg :right_pwm
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MotorPWM_msg (<MotorPWM_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorPWM_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorPWM_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lane_follower-msg:<MotorPWM_msg> is deprecated: use lane_follower-msg:MotorPWM_msg instead.")))

(cl:ensure-generic-function 'left_pwm-val :lambda-list '(m))
(cl:defmethod left_pwm-val ((m <MotorPWM_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lane_follower-msg:left_pwm-val is deprecated.  Use lane_follower-msg:left_pwm instead.")
  (left_pwm m))

(cl:ensure-generic-function 'right_pwm-val :lambda-list '(m))
(cl:defmethod right_pwm-val ((m <MotorPWM_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lane_follower-msg:right_pwm-val is deprecated.  Use lane_follower-msg:right_pwm instead.")
  (right_pwm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorPWM_msg>) ostream)
  "Serializes a message object of type '<MotorPWM_msg>"
  (cl:let* ((signed (cl:slot-value msg 'left_pwm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'right_pwm)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorPWM_msg>) istream)
  "Deserializes a message object of type '<MotorPWM_msg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'left_pwm) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'right_pwm) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorPWM_msg>)))
  "Returns string type for a message object of type '<MotorPWM_msg>"
  "lane_follower/MotorPWM_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorPWM_msg)))
  "Returns string type for a message object of type 'MotorPWM_msg"
  "lane_follower/MotorPWM_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorPWM_msg>)))
  "Returns md5sum for a message object of type '<MotorPWM_msg>"
  "1bbcb2731ff8485f26f1b76809762437")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorPWM_msg)))
  "Returns md5sum for a message object of type 'MotorPWM_msg"
  "1bbcb2731ff8485f26f1b76809762437")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorPWM_msg>)))
  "Returns full string definition for message of type '<MotorPWM_msg>"
  (cl:format cl:nil "int16 left_pwm~%int16 right_pwm~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorPWM_msg)))
  "Returns full string definition for message of type 'MotorPWM_msg"
  (cl:format cl:nil "int16 left_pwm~%int16 right_pwm~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorPWM_msg>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorPWM_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorPWM_msg
    (cl:cons ':left_pwm (left_pwm msg))
    (cl:cons ':right_pwm (right_pwm msg))
))
