;; Auto-generated. Do not edit!


(when (boundp 'iris_ir_sitl::InfraredSignal)
  (if (not (find-package "IRIS_IR_SITL"))
    (make-package "IRIS_IR_SITL"))
  (shadow 'InfraredSignal (find-package "IRIS_IR_SITL")))
(unless (find-package "IRIS_IR_SITL::INFRAREDSIGNAL")
  (make-package "IRIS_IR_SITL::INFRAREDSIGNAL"))

(in-package "ROS")
;;//! \htmlinclude InfraredSignal.msg.html


(defclass iris_ir_sitl::InfraredSignal
  :super ros::object
  :slots (_sender_id _x _y _z _strength _timestamp ))

(defmethod iris_ir_sitl::InfraredSignal
  (:init
   (&key
    ((:sender_id __sender_id) "")
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    ((:strength __strength) 0.0)
    ((:timestamp __timestamp) 0.0)
    )
   (send-super :init)
   (setq _sender_id (string __sender_id))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   (setq _strength (float __strength))
   (setq _timestamp (float __timestamp))
   self)
  (:sender_id
   (&optional __sender_id)
   (if __sender_id (setq _sender_id __sender_id)) _sender_id)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:strength
   (&optional __strength)
   (if __strength (setq _strength __strength)) _strength)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:serialization-length
   ()
   (+
    ;; string _sender_id
    4 (length _sender_id)
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ;; float64 _strength
    8
    ;; float64 _timestamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _sender_id
       (write-long (length _sender_id) s) (princ _sender_id s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _strength
       (sys::poke _strength (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _sender_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _sender_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _strength
     (setq _strength (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get iris_ir_sitl::InfraredSignal :md5sum-) "4bfc02816f81c84155a6b91d14604f20")
(setf (get iris_ir_sitl::InfraredSignal :datatype-) "iris_ir_sitl/InfraredSignal")
(setf (get iris_ir_sitl::InfraredSignal :definition-)
      "string sender_id
float64 x
float64 y
float64 z
float64 strength
float64 timestamp
")



(provide :iris_ir_sitl/InfraredSignal "4bfc02816f81c84155a6b91d14604f20")


