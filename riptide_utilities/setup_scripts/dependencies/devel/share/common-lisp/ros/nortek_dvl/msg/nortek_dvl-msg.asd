
(cl:in-package :asdf)

(defsystem "nortek_dvl-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Dvl" :depends-on ("_package_Dvl"))
    (:file "_package_Dvl" :depends-on ("_package"))
    (:file "DvlStatus" :depends-on ("_package_DvlStatus"))
    (:file "_package_DvlStatus" :depends-on ("_package"))
  ))