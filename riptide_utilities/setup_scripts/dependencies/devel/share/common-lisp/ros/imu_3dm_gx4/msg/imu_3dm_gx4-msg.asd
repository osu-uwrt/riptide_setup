
(cl:in-package :asdf)

(defsystem "imu_3dm_gx4-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FilterOutput" :depends-on ("_package_FilterOutput"))
    (:file "_package_FilterOutput" :depends-on ("_package"))
    (:file "MagFieldCF" :depends-on ("_package_MagFieldCF"))
    (:file "_package_MagFieldCF" :depends-on ("_package"))
    (:file "MagFieldKF" :depends-on ("_package_MagFieldKF"))
    (:file "_package_MagFieldKF" :depends-on ("_package"))
  ))