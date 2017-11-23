; Auto-generated. Do not edit!


(cl:in-package pedestrian_tracking-msg)


;//! \htmlinclude PedestrianPoseList.msg.html

(cl:defclass <PedestrianPoseList> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (poses
    :reader poses
    :initarg :poses
    :type (cl:vector pedestrian_tracking-msg:PedestrianPose)
   :initform (cl:make-array 0 :element-type 'pedestrian_tracking-msg:PedestrianPose :initial-element (cl:make-instance 'pedestrian_tracking-msg:PedestrianPose)))
   (frameID
    :reader frameID
    :initarg :frameID
    :type cl:integer
    :initform 0))
)

(cl:defclass PedestrianPoseList (<PedestrianPoseList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PedestrianPoseList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PedestrianPoseList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedestrian_tracking-msg:<PedestrianPoseList> is deprecated: use pedestrian_tracking-msg:PedestrianPoseList instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PedestrianPoseList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedestrian_tracking-msg:header-val is deprecated.  Use pedestrian_tracking-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <PedestrianPoseList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedestrian_tracking-msg:poses-val is deprecated.  Use pedestrian_tracking-msg:poses instead.")
  (poses m))

(cl:ensure-generic-function 'frameID-val :lambda-list '(m))
(cl:defmethod frameID-val ((m <PedestrianPoseList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedestrian_tracking-msg:frameID-val is deprecated.  Use pedestrian_tracking-msg:frameID instead.")
  (frameID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PedestrianPoseList>) ostream)
  "Serializes a message object of type '<PedestrianPoseList>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frameID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frameID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frameID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frameID)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PedestrianPoseList>) istream)
  "Deserializes a message object of type '<PedestrianPoseList>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pedestrian_tracking-msg:PedestrianPose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frameID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frameID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frameID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frameID)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PedestrianPoseList>)))
  "Returns string type for a message object of type '<PedestrianPoseList>"
  "pedestrian_tracking/PedestrianPoseList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PedestrianPoseList)))
  "Returns string type for a message object of type 'PedestrianPoseList"
  "pedestrian_tracking/PedestrianPoseList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PedestrianPoseList>)))
  "Returns md5sum for a message object of type '<PedestrianPoseList>"
  "bf65e910d3db530c8960dc8ffd5c7934")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PedestrianPoseList)))
  "Returns md5sum for a message object of type 'PedestrianPoseList"
  "bf65e910d3db530c8960dc8ffd5c7934")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PedestrianPoseList>)))
  "Returns full string definition for message of type '<PedestrianPoseList>"
  (cl:format cl:nil "Header header~%PedestrianPose[] poses~%uint32 frameID~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: pedestrian_tracking/PedestrianPose~%uint32 pedID~%uint32 frameID~%float64 x~%float64 y~%~%#undecided data type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PedestrianPoseList)))
  "Returns full string definition for message of type 'PedestrianPoseList"
  (cl:format cl:nil "Header header~%PedestrianPose[] poses~%uint32 frameID~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: pedestrian_tracking/PedestrianPose~%uint32 pedID~%uint32 frameID~%float64 x~%float64 y~%~%#undecided data type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PedestrianPoseList>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PedestrianPoseList>))
  "Converts a ROS message object to a list"
  (cl:list 'PedestrianPoseList
    (cl:cons ':header (header msg))
    (cl:cons ':poses (poses msg))
    (cl:cons ':frameID (frameID msg))
))
