; Auto-generated. Do not edit!


(cl:in-package drone_infrared_system-msg)


;//! \htmlinclude InfraredSignal.msg.html

(cl:defclass <InfraredSignal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (robot_name
    :reader robot_name
    :initarg :robot_name
    :type cl:string
    :initform "")
   (transmission_power
    :reader transmission_power
    :initarg :transmission_power
    :type cl:float
    :initform 0.0)
   (wavelength
    :reader wavelength
    :initarg :wavelength
    :type cl:float
    :initform 0.0)
   (source_pose
    :reader source_pose
    :initarg :source_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (received_power
    :reader received_power
    :initarg :received_power
    :type cl:float
    :initform 0.0)
   (estimated_distance
    :reader estimated_distance
    :initarg :estimated_distance
    :type cl:float
    :initform 0.0)
   (emitter_id
    :reader emitter_id
    :initarg :emitter_id
    :type cl:string
    :initform ""))
)

(cl:defclass InfraredSignal (<InfraredSignal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InfraredSignal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InfraredSignal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_infrared_system-msg:<InfraredSignal> is deprecated: use drone_infrared_system-msg:InfraredSignal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <InfraredSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_infrared_system-msg:header-val is deprecated.  Use drone_infrared_system-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'robot_name-val :lambda-list '(m))
(cl:defmethod robot_name-val ((m <InfraredSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_infrared_system-msg:robot_name-val is deprecated.  Use drone_infrared_system-msg:robot_name instead.")
  (robot_name m))

(cl:ensure-generic-function 'transmission_power-val :lambda-list '(m))
(cl:defmethod transmission_power-val ((m <InfraredSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_infrared_system-msg:transmission_power-val is deprecated.  Use drone_infrared_system-msg:transmission_power instead.")
  (transmission_power m))

(cl:ensure-generic-function 'wavelength-val :lambda-list '(m))
(cl:defmethod wavelength-val ((m <InfraredSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_infrared_system-msg:wavelength-val is deprecated.  Use drone_infrared_system-msg:wavelength instead.")
  (wavelength m))

(cl:ensure-generic-function 'source_pose-val :lambda-list '(m))
(cl:defmethod source_pose-val ((m <InfraredSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_infrared_system-msg:source_pose-val is deprecated.  Use drone_infrared_system-msg:source_pose instead.")
  (source_pose m))

(cl:ensure-generic-function 'received_power-val :lambda-list '(m))
(cl:defmethod received_power-val ((m <InfraredSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_infrared_system-msg:received_power-val is deprecated.  Use drone_infrared_system-msg:received_power instead.")
  (received_power m))

(cl:ensure-generic-function 'estimated_distance-val :lambda-list '(m))
(cl:defmethod estimated_distance-val ((m <InfraredSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_infrared_system-msg:estimated_distance-val is deprecated.  Use drone_infrared_system-msg:estimated_distance instead.")
  (estimated_distance m))

(cl:ensure-generic-function 'emitter_id-val :lambda-list '(m))
(cl:defmethod emitter_id-val ((m <InfraredSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_infrared_system-msg:emitter_id-val is deprecated.  Use drone_infrared_system-msg:emitter_id instead.")
  (emitter_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InfraredSignal>) ostream)
  "Serializes a message object of type '<InfraredSignal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'robot_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'robot_name))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'transmission_power))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wavelength))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'source_pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'received_power))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'estimated_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'emitter_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'emitter_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InfraredSignal>) istream)
  "Deserializes a message object of type '<InfraredSignal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'robot_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'transmission_power) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wavelength) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'source_pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'received_power) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'estimated_distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'emitter_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'emitter_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InfraredSignal>)))
  "Returns string type for a message object of type '<InfraredSignal>"
  "drone_infrared_system/InfraredSignal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InfraredSignal)))
  "Returns string type for a message object of type 'InfraredSignal"
  "drone_infrared_system/InfraredSignal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InfraredSignal>)))
  "Returns md5sum for a message object of type '<InfraredSignal>"
  "229d00d508a2c64dc70d2cc763410fb1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InfraredSignal)))
  "Returns md5sum for a message object of type 'InfraredSignal"
  "229d00d508a2c64dc70d2cc763410fb1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InfraredSignal>)))
  "Returns full string definition for message of type '<InfraredSignal>"
  (cl:format cl:nil "Header header~%string robot_name~%float64 transmission_power~%float64 wavelength~%geometry_msgs/Pose source_pose~%float64 received_power~%float64 estimated_distance~%string emitter_id~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InfraredSignal)))
  "Returns full string definition for message of type 'InfraredSignal"
  (cl:format cl:nil "Header header~%string robot_name~%float64 transmission_power~%float64 wavelength~%geometry_msgs/Pose source_pose~%float64 received_power~%float64 estimated_distance~%string emitter_id~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InfraredSignal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'robot_name))
     8
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'source_pose))
     8
     8
     4 (cl:length (cl:slot-value msg 'emitter_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InfraredSignal>))
  "Converts a ROS message object to a list"
  (cl:list 'InfraredSignal
    (cl:cons ':header (header msg))
    (cl:cons ':robot_name (robot_name msg))
    (cl:cons ':transmission_power (transmission_power msg))
    (cl:cons ':wavelength (wavelength msg))
    (cl:cons ':source_pose (source_pose msg))
    (cl:cons ':received_power (received_power msg))
    (cl:cons ':estimated_distance (estimated_distance msg))
    (cl:cons ':emitter_id (emitter_id msg))
))
