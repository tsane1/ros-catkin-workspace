; Auto-generated. Do not edit!


(cl:in-package tsane_mmlamare_mwpiazza_lab4-srv)


;//! \htmlinclude AStar-request.msg.html

(cl:defclass <AStar-request> (roslisp-msg-protocol:ros-message)
  ((frameID
    :reader frameID
    :initarg :frameID
    :type std_msgs-msg:String
    :initform (cl:make-instance 'std_msgs-msg:String))
   (map
    :reader map
    :initarg :map
    :type nav_msgs-msg:OccupancyGrid
    :initform (cl:make-instance 'nav_msgs-msg:OccupancyGrid))
   (costMap
    :reader costMap
    :initarg :costMap
    :type nav_msgs-msg:OccupancyGrid
    :initform (cl:make-instance 'nav_msgs-msg:OccupancyGrid))
   (start
    :reader start
    :initarg :start
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (goal
    :reader goal
    :initarg :goal
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass AStar-request (<AStar-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AStar-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AStar-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tsane_mmlamare_mwpiazza_lab4-srv:<AStar-request> is deprecated: use tsane_mmlamare_mwpiazza_lab4-srv:AStar-request instead.")))

(cl:ensure-generic-function 'frameID-val :lambda-list '(m))
(cl:defmethod frameID-val ((m <AStar-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tsane_mmlamare_mwpiazza_lab4-srv:frameID-val is deprecated.  Use tsane_mmlamare_mwpiazza_lab4-srv:frameID instead.")
  (frameID m))

(cl:ensure-generic-function 'map-val :lambda-list '(m))
(cl:defmethod map-val ((m <AStar-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tsane_mmlamare_mwpiazza_lab4-srv:map-val is deprecated.  Use tsane_mmlamare_mwpiazza_lab4-srv:map instead.")
  (map m))

(cl:ensure-generic-function 'costMap-val :lambda-list '(m))
(cl:defmethod costMap-val ((m <AStar-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tsane_mmlamare_mwpiazza_lab4-srv:costMap-val is deprecated.  Use tsane_mmlamare_mwpiazza_lab4-srv:costMap instead.")
  (costMap m))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <AStar-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tsane_mmlamare_mwpiazza_lab4-srv:start-val is deprecated.  Use tsane_mmlamare_mwpiazza_lab4-srv:start instead.")
  (start m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <AStar-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tsane_mmlamare_mwpiazza_lab4-srv:goal-val is deprecated.  Use tsane_mmlamare_mwpiazza_lab4-srv:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AStar-request>) ostream)
  "Serializes a message object of type '<AStar-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'frameID) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'map) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'costMap) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AStar-request>) istream)
  "Deserializes a message object of type '<AStar-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'frameID) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'map) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'costMap) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AStar-request>)))
  "Returns string type for a service object of type '<AStar-request>"
  "tsane_mmlamare_mwpiazza_lab4/AStarRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AStar-request)))
  "Returns string type for a service object of type 'AStar-request"
  "tsane_mmlamare_mwpiazza_lab4/AStarRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AStar-request>)))
  "Returns md5sum for a message object of type '<AStar-request>"
  "0e3cd40f90d5ac4d7658bc4cfa360931")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AStar-request)))
  "Returns md5sum for a message object of type 'AStar-request"
  "0e3cd40f90d5ac4d7658bc4cfa360931")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AStar-request>)))
  "Returns full string definition for message of type '<AStar-request>"
  (cl:format cl:nil "std_msgs/String frameID~%nav_msgs/OccupancyGrid map~%nav_msgs/OccupancyGrid costMap~%geometry_msgs/Pose start~%geometry_msgs/Pose goal~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: nav_msgs/OccupancyGrid~%# This represents a 2-D grid map, in which each cell represents the probability of~%# occupancy.~%~%Header header ~%~%#MetaData for the map~%MapMetaData info~%~%# The map data, in row-major order, starting with (0,0).  Occupancy~%# probabilities are in the range [0,100].  Unknown is -1.~%int8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AStar-request)))
  "Returns full string definition for message of type 'AStar-request"
  (cl:format cl:nil "std_msgs/String frameID~%nav_msgs/OccupancyGrid map~%nav_msgs/OccupancyGrid costMap~%geometry_msgs/Pose start~%geometry_msgs/Pose goal~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: nav_msgs/OccupancyGrid~%# This represents a 2-D grid map, in which each cell represents the probability of~%# occupancy.~%~%Header header ~%~%#MetaData for the map~%MapMetaData info~%~%# The map data, in row-major order, starting with (0,0).  Occupancy~%# probabilities are in the range [0,100].  Unknown is -1.~%int8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AStar-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'frameID))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'map))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'costMap))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AStar-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AStar-request
    (cl:cons ':frameID (frameID msg))
    (cl:cons ':map (map msg))
    (cl:cons ':costMap (costMap msg))
    (cl:cons ':start (start msg))
    (cl:cons ':goal (goal msg))
))
;//! \htmlinclude AStar-response.msg.html

(cl:defclass <AStar-response> (roslisp-msg-protocol:ros-message)
  ((waypoints
    :reader waypoints
    :initarg :waypoints
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass AStar-response (<AStar-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AStar-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AStar-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tsane_mmlamare_mwpiazza_lab4-srv:<AStar-response> is deprecated: use tsane_mmlamare_mwpiazza_lab4-srv:AStar-response instead.")))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <AStar-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tsane_mmlamare_mwpiazza_lab4-srv:waypoints-val is deprecated.  Use tsane_mmlamare_mwpiazza_lab4-srv:waypoints instead.")
  (waypoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AStar-response>) ostream)
  "Serializes a message object of type '<AStar-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'waypoints) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AStar-response>) istream)
  "Deserializes a message object of type '<AStar-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'waypoints) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AStar-response>)))
  "Returns string type for a service object of type '<AStar-response>"
  "tsane_mmlamare_mwpiazza_lab4/AStarResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AStar-response)))
  "Returns string type for a service object of type 'AStar-response"
  "tsane_mmlamare_mwpiazza_lab4/AStarResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AStar-response>)))
  "Returns md5sum for a message object of type '<AStar-response>"
  "0e3cd40f90d5ac4d7658bc4cfa360931")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AStar-response)))
  "Returns md5sum for a message object of type 'AStar-response"
  "0e3cd40f90d5ac4d7658bc4cfa360931")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AStar-response>)))
  "Returns full string definition for message of type '<AStar-response>"
  (cl:format cl:nil "nav_msgs/Path waypoints~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AStar-response)))
  "Returns full string definition for message of type 'AStar-response"
  (cl:format cl:nil "nav_msgs/Path waypoints~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AStar-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'waypoints))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AStar-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AStar-response
    (cl:cons ':waypoints (waypoints msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AStar)))
  'AStar-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AStar)))
  'AStar-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AStar)))
  "Returns string type for a service object of type '<AStar>"
  "tsane_mmlamare_mwpiazza_lab4/AStar")