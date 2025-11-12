# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "drone_infrared_system: 1 messages, 0 services")

set(MSG_I_FLAGS "-Idrone_infrared_system:/home/zsj/catkin_ws/src/drone_infrared_system/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(drone_infrared_system_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zsj/catkin_ws/src/drone_infrared_system/msg/InfraredSignal.msg" NAME_WE)
add_custom_target(_drone_infrared_system_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "drone_infrared_system" "/home/zsj/catkin_ws/src/drone_infrared_system/msg/InfraredSignal.msg" "geometry_msgs/Pose:geometry_msgs/Point:geometry_msgs/Quaternion:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(drone_infrared_system
  "/home/zsj/catkin_ws/src/drone_infrared_system/msg/InfraredSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_infrared_system
)

### Generating Services

### Generating Module File
_generate_module_cpp(drone_infrared_system
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_infrared_system
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(drone_infrared_system_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(drone_infrared_system_generate_messages drone_infrared_system_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zsj/catkin_ws/src/drone_infrared_system/msg/InfraredSignal.msg" NAME_WE)
add_dependencies(drone_infrared_system_generate_messages_cpp _drone_infrared_system_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_infrared_system_gencpp)
add_dependencies(drone_infrared_system_gencpp drone_infrared_system_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_infrared_system_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(drone_infrared_system
  "/home/zsj/catkin_ws/src/drone_infrared_system/msg/InfraredSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_infrared_system
)

### Generating Services

### Generating Module File
_generate_module_eus(drone_infrared_system
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_infrared_system
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(drone_infrared_system_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(drone_infrared_system_generate_messages drone_infrared_system_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zsj/catkin_ws/src/drone_infrared_system/msg/InfraredSignal.msg" NAME_WE)
add_dependencies(drone_infrared_system_generate_messages_eus _drone_infrared_system_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_infrared_system_geneus)
add_dependencies(drone_infrared_system_geneus drone_infrared_system_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_infrared_system_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(drone_infrared_system
  "/home/zsj/catkin_ws/src/drone_infrared_system/msg/InfraredSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_infrared_system
)

### Generating Services

### Generating Module File
_generate_module_lisp(drone_infrared_system
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_infrared_system
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(drone_infrared_system_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(drone_infrared_system_generate_messages drone_infrared_system_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zsj/catkin_ws/src/drone_infrared_system/msg/InfraredSignal.msg" NAME_WE)
add_dependencies(drone_infrared_system_generate_messages_lisp _drone_infrared_system_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_infrared_system_genlisp)
add_dependencies(drone_infrared_system_genlisp drone_infrared_system_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_infrared_system_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(drone_infrared_system
  "/home/zsj/catkin_ws/src/drone_infrared_system/msg/InfraredSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_infrared_system
)

### Generating Services

### Generating Module File
_generate_module_nodejs(drone_infrared_system
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_infrared_system
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(drone_infrared_system_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(drone_infrared_system_generate_messages drone_infrared_system_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zsj/catkin_ws/src/drone_infrared_system/msg/InfraredSignal.msg" NAME_WE)
add_dependencies(drone_infrared_system_generate_messages_nodejs _drone_infrared_system_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_infrared_system_gennodejs)
add_dependencies(drone_infrared_system_gennodejs drone_infrared_system_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_infrared_system_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(drone_infrared_system
  "/home/zsj/catkin_ws/src/drone_infrared_system/msg/InfraredSignal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_infrared_system
)

### Generating Services

### Generating Module File
_generate_module_py(drone_infrared_system
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_infrared_system
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(drone_infrared_system_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(drone_infrared_system_generate_messages drone_infrared_system_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zsj/catkin_ws/src/drone_infrared_system/msg/InfraredSignal.msg" NAME_WE)
add_dependencies(drone_infrared_system_generate_messages_py _drone_infrared_system_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(drone_infrared_system_genpy)
add_dependencies(drone_infrared_system_genpy drone_infrared_system_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS drone_infrared_system_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_infrared_system)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/drone_infrared_system
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(drone_infrared_system_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(drone_infrared_system_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_infrared_system)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/drone_infrared_system
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(drone_infrared_system_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(drone_infrared_system_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_infrared_system)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/drone_infrared_system
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(drone_infrared_system_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(drone_infrared_system_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_infrared_system)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/drone_infrared_system
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(drone_infrared_system_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(drone_infrared_system_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_infrared_system)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_infrared_system\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/drone_infrared_system
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(drone_infrared_system_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(drone_infrared_system_generate_messages_py geometry_msgs_generate_messages_py)
endif()
