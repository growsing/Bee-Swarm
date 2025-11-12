;; Auto-generated. Do not edit!


(when (boundp 'drone_infrared_system::InfraredSignal)
  (if (not (find-package "DRONE_INFRARED_SYSTEM"))
    (make-package "DRONE_INFRARED_SYSTEM"))
  (shadow 'InfraredSignal (find-package "DRONE_INFRARED_SYSTEM")))
(unless (find-package "DRONE_INFRARED_SYSTEM::INFRAREDSIGNAL")
  (make-package "DRONE_INFRARED_SYSTEM::INFRAREDSIGNAL"))

(in-package "ROS")
;;//! \htmlinclude InfraredSignal.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass drone_infrared_system::InfraredSignal
  :super ros::object
  :slots (_header _robot_name _transmission_power _wavelength _source_pose _received_power _estimated_distance _emitter_id ))

(defmethod drone_infrared_system::InfraredSignal
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:robot_name __robot_name) "")
    ((:transmission_power __transmission_power) 0.0)
    ((:wavelength __wavelength) 0.0)
    ((:source_pose __source_pose) (instance geometry_msgs::Pose :init))
    ((:received_power __received_power) 0.0)
    ((:estimated_distance __estimated_distance) 0.0)
    ((:emitter_id __emitter_id) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _robot_name (string __robot_name))
   (setq _transmission_power (float __transmission_power))
   (setq _wavelength (float __wavelength))
   (setq _source_pose __source_pose)
   (setq _received_power (float __received_power))
   (setq _estimated_distance (float __estimated_distance))
   (setq _emitter_id (string __emitter_id))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:robot_name
   (&optional __robot_name)
   (if __robot_name (setq _robot_name __robot_name)) _robot_name)
  (:transmission_power
   (&optional __transmission_power)
   (if __transmission_power (setq _transmission_power __transmission_power)) _transmission_power)
  (:wavelength
   (&optional __wavelength)
   (if __wavelength (setq _wavelength __wavelength)) _wavelength)
  (:source_pose
   (&rest __source_pose)
   (if (keywordp (car __source_pose))
       (send* _source_pose __source_pose)
     (progn
       (if __source_pose (setq _source_pose (car __source_pose)))
       _source_pose)))
  (:received_power
   (&optional __received_power)
   (if __received_power (setq _received_power __received_power)) _received_power)
  (:estimated_distance
   (&optional __estimated_distance)
   (if __estimated_distance (setq _estimated_distance __estimated_distance)) _estimated_distance)
  (:emitter_id
   (&optional __emitter_id)
   (if __emitter_id (setq _emitter_id __emitter_id)) _emitter_id)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _robot_name
    4 (length _robot_name)
    ;; float64 _transmission_power
    8
    ;; float64 _wavelength
    8
    ;; geometry_msgs/Pose _source_pose
    (send _source_pose :serialization-length)
    ;; float64 _received_power
    8
    ;; float64 _estimated_distance
    8
    ;; string _emitter_id
    4 (length _emitter_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _robot_name
       (write-long (length _robot_name) s) (princ _robot_name s)
     ;; float64 _transmission_power
       (sys::poke _transmission_power (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _wavelength
       (sys::poke _wavelength (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Pose _source_pose
       (send _source_pose :serialize s)
     ;; float64 _received_power
       (sys::poke _received_power (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _estimated_distance
       (sys::poke _estimated_distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _emitter_id
       (write-long (length _emitter_id) s) (princ _emitter_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _robot_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _robot_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _transmission_power
     (setq _transmission_power (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _wavelength
     (setq _wavelength (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Pose _source_pose
     (send _source_pose :deserialize buf ptr-) (incf ptr- (send _source_pose :serialization-length))
   ;; float64 _received_power
     (setq _received_power (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _estimated_distance
     (setq _estimated_distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _emitter_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _emitter_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get drone_infrared_system::InfraredSignal :md5sum-) "229d00d508a2c64dc70d2cc763410fb1")
(setf (get drone_infrared_system::InfraredSignal :datatype-) "drone_infrared_system/InfraredSignal")
(setf (get drone_infrared_system::InfraredSignal :definition-)
      "Header header
string robot_name
float64 transmission_power
float64 wavelength
geometry_msgs/Pose source_pose
float64 received_power
float64 estimated_distance
string emitter_id
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :drone_infrared_system/InfraredSignal "229d00d508a2c64dc70d2cc763410fb1")


