# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tsane_mmlamare_mwpiazza_final: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/indigo/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tsane_mmlamare_mwpiazza_final_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mwpiazza/Ros_ws/src/tsane_mmlamare_mwpiazza_final/srv/AStar.srv" NAME_WE)
add_custom_target(_tsane_mmlamare_mwpiazza_final_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tsane_mmlamare_mwpiazza_final" "/home/mwpiazza/Ros_ws/src/tsane_mmlamare_mwpiazza_final/srv/AStar.srv" "geometry_msgs/Point:geometry_msgs/Quaternion:nav_msgs/Path:std_msgs/String:std_msgs/Header:nav_msgs/OccupancyGrid:geometry_msgs/PoseStamped:nav_msgs/MapMetaData:geometry_msgs/Pose"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(tsane_mmlamare_mwpiazza_final
  "/home/mwpiazza/Ros_ws/src/tsane_mmlamare_mwpiazza_final/srv/AStar.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/String.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsane_mmlamare_mwpiazza_final
)

### Generating Module File
_generate_module_cpp(tsane_mmlamare_mwpiazza_final
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsane_mmlamare_mwpiazza_final
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tsane_mmlamare_mwpiazza_final_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages tsane_mmlamare_mwpiazza_final_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mwpiazza/Ros_ws/src/tsane_mmlamare_mwpiazza_final/srv/AStar.srv" NAME_WE)
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages_cpp _tsane_mmlamare_mwpiazza_final_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tsane_mmlamare_mwpiazza_final_gencpp)
add_dependencies(tsane_mmlamare_mwpiazza_final_gencpp tsane_mmlamare_mwpiazza_final_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tsane_mmlamare_mwpiazza_final_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(tsane_mmlamare_mwpiazza_final
  "/home/mwpiazza/Ros_ws/src/tsane_mmlamare_mwpiazza_final/srv/AStar.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/String.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsane_mmlamare_mwpiazza_final
)

### Generating Module File
_generate_module_lisp(tsane_mmlamare_mwpiazza_final
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsane_mmlamare_mwpiazza_final
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tsane_mmlamare_mwpiazza_final_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages tsane_mmlamare_mwpiazza_final_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mwpiazza/Ros_ws/src/tsane_mmlamare_mwpiazza_final/srv/AStar.srv" NAME_WE)
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages_lisp _tsane_mmlamare_mwpiazza_final_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tsane_mmlamare_mwpiazza_final_genlisp)
add_dependencies(tsane_mmlamare_mwpiazza_final_genlisp tsane_mmlamare_mwpiazza_final_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tsane_mmlamare_mwpiazza_final_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(tsane_mmlamare_mwpiazza_final
  "/home/mwpiazza/Ros_ws/src/tsane_mmlamare_mwpiazza_final/srv/AStar.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/String.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/OccupancyGrid.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/nav_msgs/cmake/../msg/MapMetaData.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsane_mmlamare_mwpiazza_final
)

### Generating Module File
_generate_module_py(tsane_mmlamare_mwpiazza_final
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsane_mmlamare_mwpiazza_final
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tsane_mmlamare_mwpiazza_final_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages tsane_mmlamare_mwpiazza_final_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mwpiazza/Ros_ws/src/tsane_mmlamare_mwpiazza_final/srv/AStar.srv" NAME_WE)
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages_py _tsane_mmlamare_mwpiazza_final_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tsane_mmlamare_mwpiazza_final_genpy)
add_dependencies(tsane_mmlamare_mwpiazza_final_genpy tsane_mmlamare_mwpiazza_final_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tsane_mmlamare_mwpiazza_final_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsane_mmlamare_mwpiazza_final)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tsane_mmlamare_mwpiazza_final
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages_cpp nav_msgs_generate_messages_cpp)
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsane_mmlamare_mwpiazza_final)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tsane_mmlamare_mwpiazza_final
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages_lisp nav_msgs_generate_messages_lisp)
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsane_mmlamare_mwpiazza_final)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsane_mmlamare_mwpiazza_final\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tsane_mmlamare_mwpiazza_final
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages_py nav_msgs_generate_messages_py)
add_dependencies(tsane_mmlamare_mwpiazza_final_generate_messages_py geometry_msgs_generate_messages_py)
