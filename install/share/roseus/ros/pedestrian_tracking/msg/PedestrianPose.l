;; Auto-generated. Do not edit!


(when (boundp 'pedestrian_tracking::PedestrianPose)
  (if (not (find-package "PEDESTRIAN_TRACKING"))
    (make-package "PEDESTRIAN_TRACKING"))
  (shadow 'PedestrianPose (find-package "PEDESTRIAN_TRACKING")))
(unless (find-package "PEDESTRIAN_TRACKING::PEDESTRIANPOSE")
  (make-package "PEDESTRIAN_TRACKING::PEDESTRIANPOSE"))

(in-package "ROS")
;;//! \htmlinclude PedestrianPose.msg.html


(defclass pedestrian_tracking::PedestrianPose
  :super ros::object
  :slots (_pedID _frameID _x _y ))

(defmethod pedestrian_tracking::PedestrianPose
  (:init
   (&key
    ((:pedID __pedID) 0)
    ((:frameID __frameID) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _pedID (round __pedID))
   (setq _frameID (round __frameID))
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:pedID
   (&optional __pedID)
   (if __pedID (setq _pedID __pedID)) _pedID)
  (:frameID
   (&optional __frameID)
   (if __frameID (setq _frameID __frameID)) _frameID)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; uint32 _pedID
    4
    ;; uint32 _frameID
    4
    ;; float64 _x
    8
    ;; float64 _y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _pedID
       (write-long _pedID s)
     ;; uint32 _frameID
       (write-long _frameID s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _pedID
     (setq _pedID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _frameID
     (setq _frameID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get pedestrian_tracking::PedestrianPose :md5sum-) "8c6b22c503c8ea4c695da904ced715cd")
(setf (get pedestrian_tracking::PedestrianPose :datatype-) "pedestrian_tracking/PedestrianPose")
(setf (get pedestrian_tracking::PedestrianPose :definition-)
      "uint32 pedID
uint32 frameID
float64 x
float64 y

#undecided data type

")



(provide :pedestrian_tracking/PedestrianPose "8c6b22c503c8ea4c695da904ced715cd")


