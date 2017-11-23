; Auto-generated. Do not edit!


(cl:in-package pedestrian_tracking-msg)


;//! \htmlinclude PedestrianPose.msg.html

(cl:defclass <PedestrianPose> (roslisp-msg-protocol:ros-message)
  ((pedID
    :reader pedID
    :initarg :pedID
    :type cl:integer
    :initform 0)
   (frameID
    :reader frameID
    :initarg :frameID
    :type cl:integer
    :initform 0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass PedestrianPose (<PedestrianPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PedestrianPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PedestrianPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pedestrian_tracking-msg:<PedestrianPose> is deprecated: use pedestrian_tracking-msg:PedestrianPose instead.")))

(cl:ensure-generic-function 'pedID-val :lambda-list '(m))
(cl:defmethod pedID-val ((m <PedestrianPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedestrian_tracking-msg:pedID-val is deprecated.  Use pedestrian_tracking-msg:pedID instead.")
  (pedID m))

(cl:ensure-generic-function 'frameID-val :lambda-list '(m))
(cl:defmethod frameID-val ((m <PedestrianPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedestrian_tracking-msg:frameID-val is deprecated.  Use pedestrian_tracking-msg:frameID instead.")
  (frameID m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <PedestrianPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedestrian_tracking-msg:x-val is deprecated.  Use pedestrian_tracking-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <PedestrianPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pedestrian_tracking-msg:y-val is deprecated.  Use pedestrian_tracking-msg:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PedestrianPose>) ostream)
  "Serializes a message object of type '<PedestrianPose>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pedID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pedID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pedID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pedID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frameID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frameID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frameID)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frameID)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PedestrianPose>) istream)
  "Deserializes a message object of type '<PedestrianPose>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pedID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'pedID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'pedID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'pedID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'frameID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'frameID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'frameID)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'frameID)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PedestrianPose>)))
  "Returns string type for a message object of type '<PedestrianPose>"
  "pedestrian_tracking/PedestrianPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PedestrianPose)))
  "Returns string type for a message object of type 'PedestrianPose"
  "pedestrian_tracking/PedestrianPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PedestrianPose>)))
  "Returns md5sum for a message object of type '<PedestrianPose>"
  "8c6b22c503c8ea4c695da904ced715cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PedestrianPose)))
  "Returns md5sum for a message object of type 'PedestrianPose"
  "8c6b22c503c8ea4c695da904ced715cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PedestrianPose>)))
  "Returns full string definition for message of type '<PedestrianPose>"
  (cl:format cl:nil "uint32 pedID~%uint32 frameID~%float64 x~%float64 y~%~%#undecided data type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PedestrianPose)))
  "Returns full string definition for message of type 'PedestrianPose"
  (cl:format cl:nil "uint32 pedID~%uint32 frameID~%float64 x~%float64 y~%~%#undecided data type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PedestrianPose>))
  (cl:+ 0
     4
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PedestrianPose>))
  "Converts a ROS message object to a list"
  (cl:list 'PedestrianPose
    (cl:cons ':pedID (pedID msg))
    (cl:cons ':frameID (frameID msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
