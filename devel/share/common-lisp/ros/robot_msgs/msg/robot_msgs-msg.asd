
(cl:in-package :asdf)

(defsystem "robot_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ik" :depends-on ("_package_ik"))
    (:file "_package_ik" :depends-on ("_package"))
    (:file "omega" :depends-on ("_package_omega"))
    (:file "_package_omega" :depends-on ("_package"))
  ))