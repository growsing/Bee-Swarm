
(cl:in-package :asdf)

(defsystem "iris_ir_sitl-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "InfraredDetection" :depends-on ("_package_InfraredDetection"))
    (:file "_package_InfraredDetection" :depends-on ("_package"))
    (:file "InfraredSignal" :depends-on ("_package_InfraredSignal"))
    (:file "_package_InfraredSignal" :depends-on ("_package"))
  ))