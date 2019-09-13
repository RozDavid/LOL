# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "laser_slam_ros: 0 messages, 1 services")

set(MSG_I_FLAGS "-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Iroscpp:/opt/ros/kinetic/share/roscpp/cmake/../msg;-Itf:/opt/ros/kinetic/share/tf/cmake/../msg;-Ivisualization_msgs:/opt/ros/kinetic/share/visualization_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(laser_slam_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/david/Catkin/cnn_loam_segmap/src/laser_slam/laser_slam_ros/srv/GetLaserTrackSrv.srv" NAME_WE)
add_custom_target(_laser_slam_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "laser_slam_ros" "/home/david/Catkin/cnn_loam_segmap/src/laser_slam/laser_slam_ros/srv/GetLaserTrackSrv.srv" "geometry_msgs/TransformStamped:std_msgs/Header:sensor_msgs/PointField:geometry_msgs/Vector3:geometry_msgs/Transform:geometry_msgs/Quaternion:sensor_msgs/PointCloud2"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(laser_slam_ros
  "/home/david/Catkin/cnn_loam_segmap/src/laser_slam/laser_slam_ros/srv/GetLaserTrackSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_slam_ros
)

### Generating Module File
_generate_module_cpp(laser_slam_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_slam_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(laser_slam_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(laser_slam_ros_generate_messages laser_slam_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/david/Catkin/cnn_loam_segmap/src/laser_slam/laser_slam_ros/srv/GetLaserTrackSrv.srv" NAME_WE)
add_dependencies(laser_slam_ros_generate_messages_cpp _laser_slam_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(laser_slam_ros_gencpp)
add_dependencies(laser_slam_ros_gencpp laser_slam_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS laser_slam_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(laser_slam_ros
  "/home/david/Catkin/cnn_loam_segmap/src/laser_slam/laser_slam_ros/srv/GetLaserTrackSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/laser_slam_ros
)

### Generating Module File
_generate_module_eus(laser_slam_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/laser_slam_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(laser_slam_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(laser_slam_ros_generate_messages laser_slam_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/david/Catkin/cnn_loam_segmap/src/laser_slam/laser_slam_ros/srv/GetLaserTrackSrv.srv" NAME_WE)
add_dependencies(laser_slam_ros_generate_messages_eus _laser_slam_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(laser_slam_ros_geneus)
add_dependencies(laser_slam_ros_geneus laser_slam_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS laser_slam_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(laser_slam_ros
  "/home/david/Catkin/cnn_loam_segmap/src/laser_slam/laser_slam_ros/srv/GetLaserTrackSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_slam_ros
)

### Generating Module File
_generate_module_lisp(laser_slam_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_slam_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(laser_slam_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(laser_slam_ros_generate_messages laser_slam_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/david/Catkin/cnn_loam_segmap/src/laser_slam/laser_slam_ros/srv/GetLaserTrackSrv.srv" NAME_WE)
add_dependencies(laser_slam_ros_generate_messages_lisp _laser_slam_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(laser_slam_ros_genlisp)
add_dependencies(laser_slam_ros_genlisp laser_slam_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS laser_slam_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(laser_slam_ros
  "/home/david/Catkin/cnn_loam_segmap/src/laser_slam/laser_slam_ros/srv/GetLaserTrackSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/laser_slam_ros
)

### Generating Module File
_generate_module_nodejs(laser_slam_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/laser_slam_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(laser_slam_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(laser_slam_ros_generate_messages laser_slam_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/david/Catkin/cnn_loam_segmap/src/laser_slam/laser_slam_ros/srv/GetLaserTrackSrv.srv" NAME_WE)
add_dependencies(laser_slam_ros_generate_messages_nodejs _laser_slam_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(laser_slam_ros_gennodejs)
add_dependencies(laser_slam_ros_gennodejs laser_slam_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS laser_slam_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(laser_slam_ros
  "/home/david/Catkin/cnn_loam_segmap/src/laser_slam/laser_slam_ros/srv/GetLaserTrackSrv.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_slam_ros
)

### Generating Module File
_generate_module_py(laser_slam_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_slam_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(laser_slam_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(laser_slam_ros_generate_messages laser_slam_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/david/Catkin/cnn_loam_segmap/src/laser_slam/laser_slam_ros/srv/GetLaserTrackSrv.srv" NAME_WE)
add_dependencies(laser_slam_ros_generate_messages_py _laser_slam_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(laser_slam_ros_genpy)
add_dependencies(laser_slam_ros_genpy laser_slam_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS laser_slam_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_slam_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/laser_slam_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(laser_slam_ros_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET roscpp_generate_messages_cpp)
  add_dependencies(laser_slam_ros_generate_messages_cpp roscpp_generate_messages_cpp)
endif()
if(TARGET tf_generate_messages_cpp)
  add_dependencies(laser_slam_ros_generate_messages_cpp tf_generate_messages_cpp)
endif()
if(TARGET visualization_msgs_generate_messages_cpp)
  add_dependencies(laser_slam_ros_generate_messages_cpp visualization_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/laser_slam_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/laser_slam_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(laser_slam_ros_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET roscpp_generate_messages_eus)
  add_dependencies(laser_slam_ros_generate_messages_eus roscpp_generate_messages_eus)
endif()
if(TARGET tf_generate_messages_eus)
  add_dependencies(laser_slam_ros_generate_messages_eus tf_generate_messages_eus)
endif()
if(TARGET visualization_msgs_generate_messages_eus)
  add_dependencies(laser_slam_ros_generate_messages_eus visualization_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_slam_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/laser_slam_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(laser_slam_ros_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET roscpp_generate_messages_lisp)
  add_dependencies(laser_slam_ros_generate_messages_lisp roscpp_generate_messages_lisp)
endif()
if(TARGET tf_generate_messages_lisp)
  add_dependencies(laser_slam_ros_generate_messages_lisp tf_generate_messages_lisp)
endif()
if(TARGET visualization_msgs_generate_messages_lisp)
  add_dependencies(laser_slam_ros_generate_messages_lisp visualization_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/laser_slam_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/laser_slam_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(laser_slam_ros_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET roscpp_generate_messages_nodejs)
  add_dependencies(laser_slam_ros_generate_messages_nodejs roscpp_generate_messages_nodejs)
endif()
if(TARGET tf_generate_messages_nodejs)
  add_dependencies(laser_slam_ros_generate_messages_nodejs tf_generate_messages_nodejs)
endif()
if(TARGET visualization_msgs_generate_messages_nodejs)
  add_dependencies(laser_slam_ros_generate_messages_nodejs visualization_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_slam_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_slam_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/laser_slam_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(laser_slam_ros_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET roscpp_generate_messages_py)
  add_dependencies(laser_slam_ros_generate_messages_py roscpp_generate_messages_py)
endif()
if(TARGET tf_generate_messages_py)
  add_dependencies(laser_slam_ros_generate_messages_py tf_generate_messages_py)
endif()
if(TARGET visualization_msgs_generate_messages_py)
  add_dependencies(laser_slam_ros_generate_messages_py visualization_msgs_generate_messages_py)
endif()
