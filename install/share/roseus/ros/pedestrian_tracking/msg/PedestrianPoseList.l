;; Auto-generated. Do not edit!


(when (boundp 'pedestrian_tracking::PedestrianPoseList)
  (if (not (find-package "PEDESTRIAN_TRACKING"))
    (make-package "PEDESTRIAN_TRACKING"))
  (shadow 'PedestrianPoseList (find-package "PEDESTRIAN_TRACKING")))
(unless (find-package "PEDESTRIAN_TRACKING::PEDESTRIANPOSELIST")
  (make-package "PEDESTRIAN_TRACKING::PEDESTRIANPOSELIST"))

(in-package "ROS")
;;//! \htmlinclude PedestrianPoseList.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass pedestrian_tracking::PedestrianPoseList
  :super ros::object
  :slots (_header _poses _frameID ))

(defmethod pedestrian_tracking::PedestrianPoseList
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:poses __poses) (let (r) (dotimes (i 0) (push (instance pedestrian_tracking::PedestrianPose :init) r)) r))
    ((:frameID __frameID) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _poses __poses)
   (setq _frameID (round __frameID))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:frameID
   (&optional __frameID)
   (if __frameID (setq _frameID __frameID)) _frameID)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; pedestrian_tracking/PedestrianPose[] _poses
    (apply #'+ (send-all _poses :serialization-length)) 4
    ;; uint32 _frameID
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; pedestrian_tracking/PedestrianPose[] _poses
     (write-long (length _poses) s)
     (dolist (elem _poses)
       (send elem :serialize s)
       )
     ;; uint32 _frameID
       (write-long _frameID s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; pedestrian_tracking/PedestrianPose[] _poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poses (let (r) (dotimes (i n) (push (instance pedestrian_tracking::PedestrianPose :init) r)) r))
     (dolist (elem- _poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint32 _frameID
     (setq _frameID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get pedestrian_tracking::PedestrianPoseList :md5sum-) "bf65e910d3db530c8960dc8ffd5c7934")
(setf (get pedestrian_tracking::PedestrianPoseList :datatype-) "pedestrian_tracking/PedestrianPoseList")
(setf (get pedestrian_tracking::PedestrianPoseList :definition-)
      "Header header
PedestrianPose[] poses
uint32 frameID
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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: pedestrian_tracking/PedestrianPose
uint32 pedID
uint32 frameID
float64 x
float64 y

#undecided data type

")



(provide :pedestrian_tracking/PedestrianPoseList "bf65e910d3db530c8960dc8ffd5c7934")


