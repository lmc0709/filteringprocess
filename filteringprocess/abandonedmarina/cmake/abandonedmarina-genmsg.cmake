# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "abandonedmarina: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iabandonedmarina:/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(abandonedmarina_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amDVL.msg" NAME_WE)
add_custom_target(_abandonedmarina_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "abandonedmarina" "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amDVL.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amMTi.msg" NAME_WE)
add_custom_target(_abandonedmarina_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "abandonedmarina" "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amMTi.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(abandonedmarina
  "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amDVL.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/abandonedmarina
)
_generate_msg_cpp(abandonedmarina
  "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amMTi.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/abandonedmarina
)

### Generating Services

### Generating Module File
_generate_module_cpp(abandonedmarina
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/abandonedmarina
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(abandonedmarina_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(abandonedmarina_generate_messages abandonedmarina_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amDVL.msg" NAME_WE)
add_dependencies(abandonedmarina_generate_messages_cpp _abandonedmarina_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amMTi.msg" NAME_WE)
add_dependencies(abandonedmarina_generate_messages_cpp _abandonedmarina_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(abandonedmarina_gencpp)
add_dependencies(abandonedmarina_gencpp abandonedmarina_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS abandonedmarina_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(abandonedmarina
  "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amDVL.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/abandonedmarina
)
_generate_msg_lisp(abandonedmarina
  "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amMTi.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/abandonedmarina
)

### Generating Services

### Generating Module File
_generate_module_lisp(abandonedmarina
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/abandonedmarina
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(abandonedmarina_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(abandonedmarina_generate_messages abandonedmarina_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amDVL.msg" NAME_WE)
add_dependencies(abandonedmarina_generate_messages_lisp _abandonedmarina_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amMTi.msg" NAME_WE)
add_dependencies(abandonedmarina_generate_messages_lisp _abandonedmarina_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(abandonedmarina_genlisp)
add_dependencies(abandonedmarina_genlisp abandonedmarina_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS abandonedmarina_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(abandonedmarina
  "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amDVL.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/abandonedmarina
)
_generate_msg_py(abandonedmarina
  "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amMTi.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/abandonedmarina
)

### Generating Services

### Generating Module File
_generate_module_py(abandonedmarina
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/abandonedmarina
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(abandonedmarina_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(abandonedmarina_generate_messages abandonedmarina_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amDVL.msg" NAME_WE)
add_dependencies(abandonedmarina_generate_messages_py _abandonedmarina_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/clarisse/catkin_ws/src/filteringprocess/filteringprocess/abandonedmarina/msg/amMTi.msg" NAME_WE)
add_dependencies(abandonedmarina_generate_messages_py _abandonedmarina_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(abandonedmarina_genpy)
add_dependencies(abandonedmarina_genpy abandonedmarina_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS abandonedmarina_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/abandonedmarina)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/abandonedmarina
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(abandonedmarina_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/abandonedmarina)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/abandonedmarina
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(abandonedmarina_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/abandonedmarina)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/abandonedmarina\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/abandonedmarina
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(abandonedmarina_generate_messages_py std_msgs_generate_messages_py)
