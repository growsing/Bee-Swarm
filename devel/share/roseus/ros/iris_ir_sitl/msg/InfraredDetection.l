;; Auto-generated. Do not edit!


(when (boundp 'iris_ir_sitl::InfraredDetection)
  (if (not (find-package "IRIS_IR_SITL"))
    (make-package "IRIS_IR_SITL"))
  (shadow 'InfraredDetection (find-package "IRIS_IR_SITL")))
(unless (find-package "IRIS_IR_SITL::INFRAREDDETECTION")
  (make-package "IRIS_IR_SITL::INFRAREDDETECTION"))

(in-package "ROS")
;;//! \htmlinclude InfraredDetection.msg.html


(defclass iris_ir_sitl::InfraredDetection
  :super ros::object
  :slots (_receiver_id _detected_ids _distances _strengths _timestamp ))

(defmethod iris_ir_sitl::InfraredDetection
  (:init
   (&key
    ((:receiver_id __receiver_id) "")
    ((:detected_ids __detected_ids) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:distances __distances) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:strengths __strengths) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:timestamp __timestamp) 0.0)
    )
   (send-super :init)
   (setq _receiver_id (string __receiver_id))
   (setq _detected_ids __detected_ids)
   (setq _distances __distances)
   (setq _strengths __strengths)
   (setq _timestamp (float __timestamp))
   self)
  (:receiver_id
   (&optional __receiver_id)
   (if __receiver_id (setq _receiver_id __receiver_id)) _receiver_id)
  (:detected_ids
   (&optional __detected_ids)
   (if __detected_ids (setq _detected_ids __detected_ids)) _detected_ids)
  (:distances
   (&optional __distances)
   (if __distances (setq _distances __distances)) _distances)
  (:strengths
   (&optional __strengths)
   (if __strengths (setq _strengths __strengths)) _strengths)
  (:timestamp
   (&optional __timestamp)
   (if __timestamp (setq _timestamp __timestamp)) _timestamp)
  (:serialization-length
   ()
   (+
    ;; string _receiver_id
    4 (length _receiver_id)
    ;; string[] _detected_ids
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _detected_ids)) 4
    ;; float64[] _distances
    (* 8    (length _distances)) 4
    ;; float64[] _strengths
    (* 8    (length _strengths)) 4
    ;; float64 _timestamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _receiver_id
       (write-long (length _receiver_id) s) (princ _receiver_id s)
     ;; string[] _detected_ids
     (write-long (length _detected_ids) s)
     (dolist (elem _detected_ids)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _distances
     (write-long (length _distances) s)
     (dotimes (i (length _distances))
       (sys::poke (elt _distances i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _strengths
     (write-long (length _strengths) s)
     (dotimes (i (length _strengths))
       (sys::poke (elt _strengths i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _timestamp
       (sys::poke _timestamp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _receiver_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _receiver_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _detected_ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detected_ids (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _detected_ids i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _distances
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _distances (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _distances i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _strengths
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _strengths (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _strengths i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _timestamp
     (setq _timestamp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get iris_ir_sitl::InfraredDetection :md5sum-) "16244eb9ef16a053d5fdf4543baba6bc")
(setf (get iris_ir_sitl::InfraredDetection :datatype-) "iris_ir_sitl/InfraredDetection")
(setf (get iris_ir_sitl::InfraredDetection :definition-)
      "string receiver_id
string[] detected_ids
float64[] distances
float64[] strengths
float64 timestamp
")



(provide :iris_ir_sitl/InfraredDetection "16244eb9ef16a053d5fdf4543baba6bc")


