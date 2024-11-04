; Auto-generated. Do not edit!


(cl:in-package FP_Magang-msg)


;//! \htmlinclude PC2BS.msg.html

(cl:defclass <PC2BS> (roslisp-msg-protocol:ros-message)
  ((bola_x
    :reader bola_x
    :initarg :bola_x
    :type cl:float
    :initform 0.0)
   (bola_y
    :reader bola_y
    :initarg :bola_y
    :type cl:float
    :initform 0.0)
   (motor1
    :reader motor1
    :initarg :motor1
    :type cl:float
    :initform 0.0)
   (motor2
    :reader motor2
    :initarg :motor2
    :type cl:float
    :initform 0.0)
   (motor3
    :reader motor3
    :initarg :motor3
    :type cl:float
    :initform 0.0))
)

(cl:defclass PC2BS (<PC2BS>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PC2BS>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PC2BS)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name FP_Magang-msg:<PC2BS> is deprecated: use FP_Magang-msg:PC2BS instead.")))

(cl:ensure-generic-function 'bola_x-val :lambda-list '(m))
(cl:defmethod bola_x-val ((m <PC2BS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader FP_Magang-msg:bola_x-val is deprecated.  Use FP_Magang-msg:bola_x instead.")
  (bola_x m))

(cl:ensure-generic-function 'bola_y-val :lambda-list '(m))
(cl:defmethod bola_y-val ((m <PC2BS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader FP_Magang-msg:bola_y-val is deprecated.  Use FP_Magang-msg:bola_y instead.")
  (bola_y m))

(cl:ensure-generic-function 'motor1-val :lambda-list '(m))
(cl:defmethod motor1-val ((m <PC2BS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader FP_Magang-msg:motor1-val is deprecated.  Use FP_Magang-msg:motor1 instead.")
  (motor1 m))

(cl:ensure-generic-function 'motor2-val :lambda-list '(m))
(cl:defmethod motor2-val ((m <PC2BS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader FP_Magang-msg:motor2-val is deprecated.  Use FP_Magang-msg:motor2 instead.")
  (motor2 m))

(cl:ensure-generic-function 'motor3-val :lambda-list '(m))
(cl:defmethod motor3-val ((m <PC2BS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader FP_Magang-msg:motor3-val is deprecated.  Use FP_Magang-msg:motor3 instead.")
  (motor3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PC2BS>) ostream)
  "Serializes a message object of type '<PC2BS>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bola_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bola_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motor1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motor2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'motor3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PC2BS>) istream)
  "Deserializes a message object of type '<PC2BS>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bola_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bola_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'motor3) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PC2BS>)))
  "Returns string type for a message object of type '<PC2BS>"
  "FP_Magang/PC2BS")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PC2BS)))
  "Returns string type for a message object of type 'PC2BS"
  "FP_Magang/PC2BS")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PC2BS>)))
  "Returns md5sum for a message object of type '<PC2BS>"
  "414e07ab141b8d50059180f242899e66")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PC2BS)))
  "Returns md5sum for a message object of type 'PC2BS"
  "414e07ab141b8d50059180f242899e66")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PC2BS>)))
  "Returns full string definition for message of type '<PC2BS>"
  (cl:format cl:nil "float32 bola_x~%float32 bola_y~%float32 motor1~%float32 motor2~%float32 motor3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PC2BS)))
  "Returns full string definition for message of type 'PC2BS"
  (cl:format cl:nil "float32 bola_x~%float32 bola_y~%float32 motor1~%float32 motor2~%float32 motor3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PC2BS>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PC2BS>))
  "Converts a ROS message object to a list"
  (cl:list 'PC2BS
    (cl:cons ':bola_x (bola_x msg))
    (cl:cons ':bola_y (bola_y msg))
    (cl:cons ':motor1 (motor1 msg))
    (cl:cons ':motor2 (motor2 msg))
    (cl:cons ':motor3 (motor3 msg))
))
