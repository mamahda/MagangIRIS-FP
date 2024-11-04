; Auto-generated. Do not edit!


(cl:in-package FP_Magang-msg)


;//! \htmlinclude BS2PC.msg.html

(cl:defclass <BS2PC> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:float
    :initform 0.0)
   (tujuan_x
    :reader tujuan_x
    :initarg :tujuan_x
    :type cl:float
    :initform 0.0)
   (tujuan_y
    :reader tujuan_y
    :initarg :tujuan_y
    :type cl:float
    :initform 0.0)
   (enc_left
    :reader enc_left
    :initarg :enc_left
    :type cl:float
    :initform 0.0)
   (enc_right
    :reader enc_right
    :initarg :enc_right
    :type cl:float
    :initform 0.0)
   (th
    :reader th
    :initarg :th
    :type cl:float
    :initform 0.0))
)

(cl:defclass BS2PC (<BS2PC>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BS2PC>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BS2PC)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name FP_Magang-msg:<BS2PC> is deprecated: use FP_Magang-msg:BS2PC instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <BS2PC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader FP_Magang-msg:status-val is deprecated.  Use FP_Magang-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'tujuan_x-val :lambda-list '(m))
(cl:defmethod tujuan_x-val ((m <BS2PC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader FP_Magang-msg:tujuan_x-val is deprecated.  Use FP_Magang-msg:tujuan_x instead.")
  (tujuan_x m))

(cl:ensure-generic-function 'tujuan_y-val :lambda-list '(m))
(cl:defmethod tujuan_y-val ((m <BS2PC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader FP_Magang-msg:tujuan_y-val is deprecated.  Use FP_Magang-msg:tujuan_y instead.")
  (tujuan_y m))

(cl:ensure-generic-function 'enc_left-val :lambda-list '(m))
(cl:defmethod enc_left-val ((m <BS2PC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader FP_Magang-msg:enc_left-val is deprecated.  Use FP_Magang-msg:enc_left instead.")
  (enc_left m))

(cl:ensure-generic-function 'enc_right-val :lambda-list '(m))
(cl:defmethod enc_right-val ((m <BS2PC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader FP_Magang-msg:enc_right-val is deprecated.  Use FP_Magang-msg:enc_right instead.")
  (enc_right m))

(cl:ensure-generic-function 'th-val :lambda-list '(m))
(cl:defmethod th-val ((m <BS2PC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader FP_Magang-msg:th-val is deprecated.  Use FP_Magang-msg:th instead.")
  (th m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BS2PC>) ostream)
  "Serializes a message object of type '<BS2PC>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tujuan_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tujuan_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'enc_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'enc_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'th))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BS2PC>) istream)
  "Deserializes a message object of type '<BS2PC>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'status) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tujuan_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tujuan_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'enc_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'enc_right) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'th) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BS2PC>)))
  "Returns string type for a message object of type '<BS2PC>"
  "FP_Magang/BS2PC")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BS2PC)))
  "Returns string type for a message object of type 'BS2PC"
  "FP_Magang/BS2PC")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BS2PC>)))
  "Returns md5sum for a message object of type '<BS2PC>"
  "6713ba3a23152bdfb17a6636f933f589")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BS2PC)))
  "Returns md5sum for a message object of type 'BS2PC"
  "6713ba3a23152bdfb17a6636f933f589")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BS2PC>)))
  "Returns full string definition for message of type '<BS2PC>"
  (cl:format cl:nil "float32 status~%float32 tujuan_x~%float32 tujuan_y~%float32 enc_left~%float32 enc_right~%float32 th~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BS2PC)))
  "Returns full string definition for message of type 'BS2PC"
  (cl:format cl:nil "float32 status~%float32 tujuan_x~%float32 tujuan_y~%float32 enc_left~%float32 enc_right~%float32 th~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BS2PC>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BS2PC>))
  "Converts a ROS message object to a list"
  (cl:list 'BS2PC
    (cl:cons ':status (status msg))
    (cl:cons ':tujuan_x (tujuan_x msg))
    (cl:cons ':tujuan_y (tujuan_y msg))
    (cl:cons ':enc_left (enc_left msg))
    (cl:cons ':enc_right (enc_right msg))
    (cl:cons ':th (th msg))
))
