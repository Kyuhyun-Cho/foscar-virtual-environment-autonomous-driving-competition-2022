
(cl:in-package :asdf)

(defsystem "obstacleDetection-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Boundingbox" :depends-on ("_package_Boundingbox"))
    (:file "_package_Boundingbox" :depends-on ("_package"))
  ))