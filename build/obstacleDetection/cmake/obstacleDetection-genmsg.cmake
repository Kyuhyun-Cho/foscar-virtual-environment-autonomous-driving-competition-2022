# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "obstacleDetection: 1 messages, 0 services")

set(MSG_I_FLAGS "-IobstacleDetection:/home/foscar/VEAC_2023/src/obstacleDetection/msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(obstacleDetection_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/foscar/VEAC_2023/src/obstacleDetection/msg/Boundingbox.msg" NAME_WE)
add_custom_target(_obstacleDetection_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "obstacleDetection" "/home/foscar/VEAC_2023/src/obstacleDetection/msg/Boundingbox.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(obstacleDetection
  "/home/foscar/VEAC_2023/src/obstacleDetection/msg/Boundingbox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/obstacleDetection
)

### Generating Services

### Generating Module File
_generate_module_cpp(obstacleDetection
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/obstacleDetection
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(obstacleDetection_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(obstacleDetection_generate_messages obstacleDetection_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/foscar/VEAC_2023/src/obstacleDetection/msg/Boundingbox.msg" NAME_WE)
add_dependencies(obstacleDetection_generate_messages_cpp _obstacleDetection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(obstacleDetection_gencpp)
add_dependencies(obstacleDetection_gencpp obstacleDetection_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS obstacleDetection_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(obstacleDetection
  "/home/foscar/VEAC_2023/src/obstacleDetection/msg/Boundingbox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/obstacleDetection
)

### Generating Services

### Generating Module File
_generate_module_eus(obstacleDetection
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/obstacleDetection
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(obstacleDetection_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(obstacleDetection_generate_messages obstacleDetection_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/foscar/VEAC_2023/src/obstacleDetection/msg/Boundingbox.msg" NAME_WE)
add_dependencies(obstacleDetection_generate_messages_eus _obstacleDetection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(obstacleDetection_geneus)
add_dependencies(obstacleDetection_geneus obstacleDetection_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS obstacleDetection_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(obstacleDetection
  "/home/foscar/VEAC_2023/src/obstacleDetection/msg/Boundingbox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/obstacleDetection
)

### Generating Services

### Generating Module File
_generate_module_lisp(obstacleDetection
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/obstacleDetection
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(obstacleDetection_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(obstacleDetection_generate_messages obstacleDetection_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/foscar/VEAC_2023/src/obstacleDetection/msg/Boundingbox.msg" NAME_WE)
add_dependencies(obstacleDetection_generate_messages_lisp _obstacleDetection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(obstacleDetection_genlisp)
add_dependencies(obstacleDetection_genlisp obstacleDetection_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS obstacleDetection_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(obstacleDetection
  "/home/foscar/VEAC_2023/src/obstacleDetection/msg/Boundingbox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/obstacleDetection
)

### Generating Services

### Generating Module File
_generate_module_nodejs(obstacleDetection
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/obstacleDetection
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(obstacleDetection_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(obstacleDetection_generate_messages obstacleDetection_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/foscar/VEAC_2023/src/obstacleDetection/msg/Boundingbox.msg" NAME_WE)
add_dependencies(obstacleDetection_generate_messages_nodejs _obstacleDetection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(obstacleDetection_gennodejs)
add_dependencies(obstacleDetection_gennodejs obstacleDetection_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS obstacleDetection_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(obstacleDetection
  "/home/foscar/VEAC_2023/src/obstacleDetection/msg/Boundingbox.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/obstacleDetection
)

### Generating Services

### Generating Module File
_generate_module_py(obstacleDetection
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/obstacleDetection
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(obstacleDetection_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(obstacleDetection_generate_messages obstacleDetection_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/foscar/VEAC_2023/src/obstacleDetection/msg/Boundingbox.msg" NAME_WE)
add_dependencies(obstacleDetection_generate_messages_py _obstacleDetection_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(obstacleDetection_genpy)
add_dependencies(obstacleDetection_genpy obstacleDetection_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS obstacleDetection_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/obstacleDetection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/obstacleDetection
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(obstacleDetection_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(obstacleDetection_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(obstacleDetection_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(obstacleDetection_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/obstacleDetection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/obstacleDetection
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(obstacleDetection_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(obstacleDetection_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(obstacleDetection_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(obstacleDetection_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/obstacleDetection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/obstacleDetection
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(obstacleDetection_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(obstacleDetection_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(obstacleDetection_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(obstacleDetection_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/obstacleDetection)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/obstacleDetection
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(obstacleDetection_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(obstacleDetection_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(obstacleDetection_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(obstacleDetection_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/obstacleDetection)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/obstacleDetection\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/obstacleDetection
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(obstacleDetection_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(obstacleDetection_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(obstacleDetection_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(obstacleDetection_generate_messages_py nav_msgs_generate_messages_py)
endif()
