;; Auto-generated. Do not edit!


(when (boundp 'FP_Magang::PC2BS)
  (if (not (find-package "FP_MAGANG"))
    (make-package "FP_MAGANG"))
  (shadow 'PC2BS (find-package "FP_MAGANG")))
(unless (find-package "FP_MAGANG::PC2BS")
  (make-package "FP_MAGANG::PC2BS"))

(in-package "ROS")
;;//! \htmlinclude PC2BS.msg.html


(defclass FP_Magang::PC2BS
  :super ros::object
  :slots (_bola_x _bola_y _motor1 _motor2 _motor3 ))

(defmethod FP_Magang::PC2BS
  (:init
   (&key
    ((:bola_x __bola_x) 0.0)
    ((:bola_y __bola_y) 0.0)
    ((:motor1 __motor1) 0.0)
    ((:motor2 __motor2) 0.0)
    ((:motor3 __motor3) 0.0)
    )
   (send-super :init)
   (setq _bola_x (float __bola_x))
   (setq _bola_y (float __bola_y))
   (setq _motor1 (float __motor1))
   (setq _motor2 (float __motor2))
   (setq _motor3 (float __motor3))
   self)
  (:bola_x
   (&optional __bola_x)
   (if __bola_x (setq _bola_x __bola_x)) _bola_x)
  (:bola_y
   (&optional __bola_y)
   (if __bola_y (setq _bola_y __bola_y)) _bola_y)
  (:motor1
   (&optional __motor1)
   (if __motor1 (setq _motor1 __motor1)) _motor1)
  (:motor2
   (&optional __motor2)
   (if __motor2 (setq _motor2 __motor2)) _motor2)
  (:motor3
   (&optional __motor3)
   (if __motor3 (setq _motor3 __motor3)) _motor3)
  (:serialization-length
   ()
   (+
    ;; float32 _bola_x
    4
    ;; float32 _bola_y
    4
    ;; float32 _motor1
    4
    ;; float32 _motor2
    4
    ;; float32 _motor3
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _bola_x
       (sys::poke _bola_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bola_y
       (sys::poke _bola_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _motor1
       (sys::poke _motor1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _motor2
       (sys::poke _motor2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _motor3
       (sys::poke _motor3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _bola_x
     (setq _bola_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bola_y
     (setq _bola_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _motor1
     (setq _motor1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _motor2
     (setq _motor2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _motor3
     (setq _motor3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get FP_Magang::PC2BS :md5sum-) "414e07ab141b8d50059180f242899e66")
(setf (get FP_Magang::PC2BS :datatype-) "FP_Magang/PC2BS")
(setf (get FP_Magang::PC2BS :definition-)
      "float32 bola_x
float32 bola_y
float32 motor1
float32 motor2
float32 motor3

")



(provide :FP_Magang/PC2BS "414e07ab141b8d50059180f242899e66")


