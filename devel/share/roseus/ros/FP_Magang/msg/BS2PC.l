;; Auto-generated. Do not edit!


(when (boundp 'FP_Magang::BS2PC)
  (if (not (find-package "FP_MAGANG"))
    (make-package "FP_MAGANG"))
  (shadow 'BS2PC (find-package "FP_MAGANG")))
(unless (find-package "FP_MAGANG::BS2PC")
  (make-package "FP_MAGANG::BS2PC"))

(in-package "ROS")
;;//! \htmlinclude BS2PC.msg.html


(defclass FP_Magang::BS2PC
  :super ros::object
  :slots (_status _tujuan_x _tujuan_y _enc_left _enc_right _th ))

(defmethod FP_Magang::BS2PC
  (:init
   (&key
    ((:status __status) 0.0)
    ((:tujuan_x __tujuan_x) 0.0)
    ((:tujuan_y __tujuan_y) 0.0)
    ((:enc_left __enc_left) 0.0)
    ((:enc_right __enc_right) 0.0)
    ((:th __th) 0.0)
    )
   (send-super :init)
   (setq _status (float __status))
   (setq _tujuan_x (float __tujuan_x))
   (setq _tujuan_y (float __tujuan_y))
   (setq _enc_left (float __enc_left))
   (setq _enc_right (float __enc_right))
   (setq _th (float __th))
   self)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:tujuan_x
   (&optional __tujuan_x)
   (if __tujuan_x (setq _tujuan_x __tujuan_x)) _tujuan_x)
  (:tujuan_y
   (&optional __tujuan_y)
   (if __tujuan_y (setq _tujuan_y __tujuan_y)) _tujuan_y)
  (:enc_left
   (&optional __enc_left)
   (if __enc_left (setq _enc_left __enc_left)) _enc_left)
  (:enc_right
   (&optional __enc_right)
   (if __enc_right (setq _enc_right __enc_right)) _enc_right)
  (:th
   (&optional __th)
   (if __th (setq _th __th)) _th)
  (:serialization-length
   ()
   (+
    ;; float32 _status
    4
    ;; float32 _tujuan_x
    4
    ;; float32 _tujuan_y
    4
    ;; float32 _enc_left
    4
    ;; float32 _enc_right
    4
    ;; float32 _th
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _status
       (sys::poke _status (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tujuan_x
       (sys::poke _tujuan_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _tujuan_y
       (sys::poke _tujuan_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _enc_left
       (sys::poke _enc_left (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _enc_right
       (sys::poke _enc_right (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _th
       (sys::poke _th (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _status
     (setq _status (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tujuan_x
     (setq _tujuan_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _tujuan_y
     (setq _tujuan_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _enc_left
     (setq _enc_left (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _enc_right
     (setq _enc_right (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _th
     (setq _th (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get FP_Magang::BS2PC :md5sum-) "6713ba3a23152bdfb17a6636f933f589")
(setf (get FP_Magang::BS2PC :datatype-) "FP_Magang/BS2PC")
(setf (get FP_Magang::BS2PC :definition-)
      "float32 status
float32 tujuan_x
float32 tujuan_y
float32 enc_left
float32 enc_right
float32 th
")



(provide :FP_Magang/BS2PC "6713ba3a23152bdfb17a6636f933f589")


