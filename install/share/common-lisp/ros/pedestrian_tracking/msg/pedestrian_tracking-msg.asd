
(cl:in-package :asdf)

(defsystem "pedestrian_tracking-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PedestrianPose" :depends-on ("_package_PedestrianPose"))
    (:file "_package_PedestrianPose" :depends-on ("_package"))
    (:file "PedestrianPoseList" :depends-on ("_package_PedestrianPoseList"))
    (:file "_package_PedestrianPoseList" :depends-on ("_package"))
  ))