;; Auto-generated. Do not edit!


(when (boundp 'velocity2state2::truth_rotation)
  (if (not (find-package "VELOCITY2STATE2"))
    (make-package "VELOCITY2STATE2"))
  (shadow 'truth_rotation (find-package "VELOCITY2STATE2")))
(unless (find-package "VELOCITY2STATE2::TRUTH_ROTATION")
  (make-package "VELOCITY2STATE2::TRUTH_ROTATION"))

(in-package "ROS")
;;//! \htmlinclude truth_rotation.msg.html
(if (not (find-package "TF"))
  (ros::roseus-add-msgs "tf"))


(defclass velocity2state2::truth_rotation
  :super ros::object
  :slots (_send_transform ))

(defmethod velocity2state2::truth_rotation
  (:init
   (&key
    ((:send_transform __send_transform) (instance tf::tfMessage :init))
    )
   (send-super :init)
   (setq _send_transform __send_transform)
   self)
  (:send_transform
   (&rest __send_transform)
   (if (keywordp (car __send_transform))
       (send* _send_transform __send_transform)
     (progn
       (if __send_transform (setq _send_transform (car __send_transform)))
       _send_transform)))
  (:serialization-length
   ()
   (+
    ;; tf/tfMessage _send_transform
    (send _send_transform :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; tf/tfMessage _send_transform
       (send _send_transform :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; tf/tfMessage _send_transform
     (send _send_transform :deserialize buf ptr-) (incf ptr- (send _send_transform :serialization-length))
   ;;
   self)
  )

(setf (get velocity2state2::truth_rotation :md5sum-) "57c643213939b2acaf9cdeb59405057e")
(setf (get velocity2state2::truth_rotation :datatype-) "velocity2state2/truth_rotation")
(setf (get velocity2state2::truth_rotation :definition-)
      "tf/tfMessage send_transform

================================================================================
MSG: tf/tfMessage
geometry_msgs/TransformStamped[] transforms

================================================================================
MSG: geometry_msgs/TransformStamped
# This expresses a transform from coordinate frame header.frame_id
# to the coordinate frame child_frame_id
#
# This message is mostly used by the 
# <a href=\"http://wiki.ros.org/tf\">tf</a> package. 
# See its documentation for more information.

Header header
string child_frame_id # the frame id of the child frame
Transform transform

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
string frame_id

================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :velocity2state2/truth_rotation "57c643213939b2acaf9cdeb59405057e")


