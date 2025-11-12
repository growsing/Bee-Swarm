
(cl:in-package :asdf)

(defsystem "drone_infrared_system-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "InfraredSignal" :depends-on ("_package_InfraredSignal"))
    (:file "_package_InfraredSignal" :depends-on ("_package"))
  ))