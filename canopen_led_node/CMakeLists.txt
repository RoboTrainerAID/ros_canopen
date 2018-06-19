cmake_minimum_required(VERSION 2.8.3)
project(canopen_led_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  canopen_chain_node
  canopen_401
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Led.msg
   BankMapping.msg 
   GlobalMapping.msg 
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES can_led_node
  CATKIN_DEPENDS message_runtime
  DEPENDS system_lib std_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_executable(canopen_led_node
  src/canopen_led_layer.cpp
  src/led_chain_node.cpp
)
target_link_libraries(canopen_led_node
  ${catkin_LIBRARIES}
) 
add_dependencies(canopen_led_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS canopen_led_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

add_executable(send_led_test test/src/send_led_test.cpp)
target_link_libraries(send_led_test ${catkin_LIBRARIES})
add_dependencies(send_led_test ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(send test/src/send_script.cpp)
target_link_libraries(send ${catkin_LIBRARIES})
add_dependencies(send ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

